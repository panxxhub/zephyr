/*
 * Copyright (c) 2018 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(net_coap, CONFIG_COAP_LOG_LEVEL);

#include <stdlib.h>
#include <stddef.h>
#include <zephyr/types.h>
#include <string.h>
#include <stdbool.h>
#include <errno.h>

#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/hash_function.h>
#include <zephyr/sys/math_extras.h>
#include <zephyr/sys/util.h>

#include <zephyr/sys/printk.h>

#include <zephyr/net/coap.h>
#include <zephyr/net/coap_link_format.h>

#include "coap_internal.h"

static inline bool append(struct coap_packet *cpkt, const uint8_t *data, uint16_t len)
{
	if (!cpkt || !data) {
		return false;
	}

	if (cpkt->max_len - cpkt->offset < len) {
		return false;
	}

	memcpy(cpkt->data + cpkt->offset, data, len);
	cpkt->offset += len;

	return true;
}

static bool match_path_uri(const char * const *path,
			   const char *uri, uint16_t len)
{
	const char * const *p;
	bool wildcard = false;
	uint16_t prefix_len;
	uint16_t pos;

	if (path == NULL || *path == NULL || uri == NULL || len == 0U) {
		return false;
	}

	if (uri[len - 1U] == '*') {
		wildcard = true;
		prefix_len = len - 1U;
	} else {
		prefix_len = len;
	}

	if (prefix_len == 0U) {
		return true;
	}

	if (uri[0] != '/') {
		return false;
	}

	pos = 1U;
	if (pos == prefix_len) {
		return wildcard || ((*path)[0] == '\0' && *(path + 1) == NULL);
	}

	for (p = path; *p != NULL; p++) {
		size_t seg_len = strlen(*p);
		uint16_t rem = prefix_len - pos;

		if (p != path) {
			if (rem == 0U) {
				return wildcard;
			}

			if (uri[pos] != '/') {
				return false;
			}

			pos++;
			rem--;
			if (rem == 0U) {
				return wildcard;
			}
		}

		if (rem < seg_len) {
			return wildcard && (memcmp(&uri[pos], *p, rem) == 0);
		}

		if (memcmp(&uri[pos], *p, seg_len) != 0) {
			return false;
		}

		pos += (uint16_t)seg_len;
	}

	return pos == prefix_len;
}

static bool match_attributes(const char * const *attributes,
			     const struct coap_option *query)
{
	const char * const *attr;

	/* FIXME: deal with the case when there are multiple options in a
	 * query, for example: 'rt=lux temperature', if I want to list
	 * resources with resource type lux or temperature.
	 */
	for (attr = attributes; attr && *attr; attr++) {
		uint16_t attr_len = strlen(*attr);

		if (query->len != attr_len) {
			continue;
		}

		if (!strncmp((char *) query->value, *attr, attr_len)) {
			return true;
		}
	}

	return false;
}

static bool match_queries_resource(const struct coap_resource *resource,
				   const struct coap_option *query,
				   int num_queries)
{
	static const char href_prefix[] = "href=";
	const size_t href_prefix_len = sizeof(href_prefix) - 1U;
	struct coap_core_metadata *meta = resource->metadata;
	const char * const *attributes = NULL;

	if (num_queries == 0) {
		return true;
	}

	if (query->len > href_prefix_len &&
	    memcmp(query->value, href_prefix, href_prefix_len) == 0) {
		const char *uri = (const char *)query->value + href_prefix_len;
		uint16_t uri_len = query->len - (uint16_t)href_prefix_len;

		return match_path_uri(resource->path, uri, uri_len);
	}

	if (meta != NULL && meta->attributes != NULL) {
		attributes = meta->attributes;
	}

	if (attributes == NULL) {
		return false;
	}

	return match_attributes(attributes, query);
}

#if defined(CONFIG_COAP_WELL_KNOWN_BLOCK_WISE)

static uint32_t hash_combine_str(uint32_t seed, const char *str)
{
	/* Boost-style hash combine, the constant is the 32-bit golden ratio */
	return seed ^ (sys_hash32(str, strlen(str)) + 0x9e3779b9U + (seed << 6) + (seed >> 2));
}

/* The link-format document is regenerated for every block, so derive an ETag
 * from the inputs that determine its content. It lets clients verify that the
 * blocks they reassemble belong to the same representation
 * (RFC 7959, section 2.4).
 */
static uint32_t well_known_core_etag(const struct coap_resource *resources, size_t resources_len,
				     const struct coap_option *query, unsigned int num_queries)
{
	uint32_t etag = 0U;

	for (size_t i = 0; i < resources_len; i++) {
		const struct coap_core_metadata *meta = resources[i].metadata;

		if (!match_queries_resource(&resources[i], query, num_queries)) {
			continue;
		}

		for (const char * const *p = resources[i].path; p != NULL && *p != NULL; p++) {
			etag = hash_combine_str(etag, *p);
		}

		/* Separate the path segments from the attributes, like ">" does
		 * in the document, so that a segment moving between the two
		 * lists, or to the next resource, changes the ETag.
		 */
		etag = hash_combine_str(etag, ">");

		if (meta != NULL && meta->attributes != NULL) {
			for (const char * const *attr = meta->attributes; *attr != NULL; attr++) {
				etag = hash_combine_str(etag, *attr);
			}
		}
	}

	return etag;
}

enum coap_block_size default_block_size(void)
{
	switch (CONFIG_COAP_WELL_KNOWN_BLOCK_WISE_SIZE) {
	case 16:
		return COAP_BLOCK_16;
	case 32:
		return COAP_BLOCK_32;
	case 64:
		return COAP_BLOCK_64;
	case 128:
		return COAP_BLOCK_128;
	case 256:
		return COAP_BLOCK_256;
	case 512:
		return COAP_BLOCK_512;
	case 1024:
		return COAP_BLOCK_1024;
	}

	return COAP_BLOCK_64;
}

static int well_known_core_total_size(const struct coap_resource *resources,
				      size_t resources_len,
				      const struct coap_option *query,
				      unsigned int num_queries,
				      size_t *total_size)
{
	size_t total = 0U;
	bool first = true;

	for (size_t i = 0; i < resources_len; i++) {
		const struct coap_core_metadata *meta = resources[i].metadata;
		const char * const *p;
		size_t res_len;

		if (!match_queries_resource(&resources[i], query, num_queries)) {
			continue;
		}

		if (resources[i].path == NULL) {
			return -EINVAL;
		}

		/* "," separator (after first resource) + "</" + ">" */
		res_len = first ? 3U : 4U;
		first = false;

		for (p = resources[i].path; *p != NULL; p++) {
			size_t seg_len = strlen(*p);

			if (size_add_overflow(res_len, seg_len, &res_len)) {
				return -EOVERFLOW;
			}

			if (*(p + 1) != NULL && size_add_overflow(res_len, 1U, &res_len)) {
				return -EOVERFLOW;
			}
		}

		if (meta != NULL && meta->attributes != NULL) {
			for (const char * const *attr = meta->attributes; *attr != NULL; attr++) {
				size_t attr_len = strlen(*attr);

				if (size_add_overflow(res_len, 1U, &res_len) ||
				    size_add_overflow(res_len, attr_len, &res_len)) {
					return -EOVERFLOW;
				}
			}
		}

		if (size_add_overflow(total, res_len, &total)) {
			return -EOVERFLOW;
		}
	}

	*total_size = total;
	return 0;
}

static bool append_to_coap_pkt(struct coap_packet *response,
			       const char *str, size_t len,
			       uint16_t *remaining, size_t *offset,
			       size_t current)
{
	size_t pos = 0U;
	bool res;

	if (!*remaining || len == 0U) {
		return true;
	}

	if (*offset < current) {
		pos = current - *offset;

		if (len > pos) {
			len -= pos;
			*offset += pos;
		} else {
			*offset += len;
			return true;
		}
	}

	if (len > *remaining) {
		len = *remaining;
	}

	res = append(response, (const uint8_t *)(str + pos), (uint16_t)len);

	*remaining -= (uint16_t)len;
	*offset += len;

	return res;
}

static int format_uri(const char * const *path,
		      struct coap_packet *response,
		      uint16_t *remaining, size_t *offset,
		      size_t current)
{
	static const char prefix[] = "</";
	const char * const *p;
	bool res;

	if (!path) {
		return -EINVAL;
	}

	res = append_to_coap_pkt(response, &prefix[0], sizeof(prefix) - 1U,
				 remaining, offset, current);
	if (!res) {
		return -ENOMEM;
	}

	if (!*remaining) {
		return 0;
	}

	for (p = path; *p; ) {
		size_t path_len = strlen(*p);

		res = append_to_coap_pkt(response, *p, path_len, remaining,
					 offset, current);
		if (!res) {
			return -ENOMEM;
		}

		if (!*remaining) {
			return 0;
		}

		p++;
		if (!*p) {
			continue;
		}

		res = append_to_coap_pkt(response, "/", 1U, remaining, offset,
					 current);
		if (!res) {
			return -ENOMEM;
		}

		if (!*remaining) {
			return 0;
		}
	}

	res = append_to_coap_pkt(response, ">", 1U, remaining, offset, current);
	if (!res) {
		return -ENOMEM;
	}

	return 0;
}

static int format_attributes(const char * const *attributes,
			     struct coap_packet *response,
			     uint16_t *remaining, size_t *offset,
			     size_t current)
{
	const char * const *attr;
	bool res;

	if (!attributes) {
		return 0;
	}

	for (attr = attributes; *attr; attr++) {
		size_t attr_len;

		res = append_to_coap_pkt(response, ";", 1U,
					 remaining, offset, current);
		if (!res) {
			return -ENOMEM;
		}

		if (!*remaining) {
			return 0;
		}

		attr_len = strlen(*attr);

		res = append_to_coap_pkt(response, *attr, attr_len,
					 remaining, offset, current);
		if (!res) {
			return -ENOMEM;
		}

		if (!*remaining) {
			return 0;
		}
	}

	return 0;
}

static int format_resource(const struct coap_resource *resource,
			   struct coap_packet *response,
			   uint16_t *remaining, size_t *offset,
			   size_t current)
{
	struct coap_core_metadata *meta = resource->metadata;
	const char * const *attributes = NULL;
	int r;

	r = format_uri(resource->path, response, remaining, offset, current);
	if (r < 0) {
		return r;
	}

	if (!*remaining) {
		return 0;
	}

	if (meta != NULL && meta->attributes != NULL) {
		attributes = meta->attributes;
	}

	return format_attributes(attributes, response, remaining, offset,
				 current);
}

int coap_well_known_core_get_len(struct coap_resource *resources,
				 size_t resources_len,
				 const struct coap_packet *request,
				 struct coap_packet *response,
				 uint8_t *data, uint16_t len)
{
	struct coap_block_context ctx;
	struct coap_option query;
	struct coap_option block2_opts[2];
	unsigned int num_queries;
	size_t total_size;
	size_t offset;
	uint16_t remaining;
	int r;
	bool first = true;

	if (!resources || !request || !response || !data || !len) {
		return -EINVAL;
	}

	/* Per RFC 6690, Section 4.1, only one (or none) query parameter may be
	 * provided, use the first if multiple.
	 */
	r = coap_find_options(request, COAP_OPTION_URI_QUERY, &query, 1);
	if (r < 0) {
		return r;
	}

	num_queries = r;

	r = well_known_core_total_size(resources, resources_len, &query,
				       num_queries, &total_size);
	if (r < 0) {
		return r;
	}

	/* The response is regenerated for every block and the block context is
	 * derived from the request alone, so no state is shared between calls
	 * and concurrent transfers from different clients stay independent.
	 * Initializing the context with the exact link-format document length
	 * lets coap_append_block2_option() encode the M bit directly without a
	 * fixed size ceiling or post-hoc option rewriting.
	 */
	coap_block_transfer_init(&ctx, default_block_size(), total_size);

	/* Validate the Block2 option (RFC 7959 Table 1: non-repeatable, 0-3
	 * bytes) and honor a smaller block size negotiated by the client on an
	 * earlier block, which coap_update_from_block() would otherwise reject
	 * as a size mismatch.
	 */
	r = coap_find_options(request, COAP_OPTION_BLOCK2, block2_opts, ARRAY_SIZE(block2_opts));
	if (r < 0) {
		return r;
	}

	if (r > 1 || (r == 1 && block2_opts[0].len > 3U)) {
		return -EINVAL;
	}

	if (r == 1) {
		unsigned int block2 = coap_option_value_to_int(&block2_opts[0]);

		ctx.block_size = MIN((enum coap_block_size)GET_BLOCK_SIZE(block2),
				     ctx.block_size);
	}

	r = coap_update_from_block_for_option(request, &ctx, COAP_OPTION_BLOCK2);
	if (r < 0) {
		return r;
	}

	if (ctx.current > 0U && ctx.current >= total_size) {
		return -EINVAL;
	}

	r = coap_init_response_packet(response, request, COAP_RESPONSE_CODE_CONTENT, data, len);
	if (r < 0) {
		return -ENOMEM;
	}

	if (IS_ENABLED(CONFIG_SYS_HASH_FUNC32)) {
		uint32_t etag = well_known_core_etag(resources, resources_len,
						     &query, num_queries);

		r = coap_packet_append_option(response, COAP_OPTION_ETAG,
					      (const uint8_t *)&etag, sizeof(etag));
		if (r < 0) {
			return -ENOMEM;
		}
	}

	r = coap_append_option_int(response, COAP_OPTION_CONTENT_FORMAT,
				   COAP_CONTENT_FORMAT_APP_LINK_FORMAT);
	if (r < 0) {
		return -ENOMEM;
	}

	r = coap_append_block2_option(response, &ctx);
	if (r < 0) {
		return -ENOMEM;
	}

	if (total_size == 0U) {
		return 0;
	}

	r = coap_packet_append_payload_marker(response);
	if (r < 0) {
		return -ENOMEM;
	}

	offset = 0U;
	remaining = coap_block_size_to_bytes(ctx.block_size);

	for (size_t i = 0; i < resources_len && remaining > 0U; ++i) {
		if (!match_queries_resource(&resources[i], &query, num_queries)) {
			continue;
		}

		if (first) {
			first = false;
		} else {
			if (!append_to_coap_pkt(response, ",", 1U, &remaining,
						&offset, ctx.current)) {
				return -ENOMEM;
			}

			if (!remaining) {
				break;
			}
		}

		r = format_resource(&resources[i], response, &remaining, &offset,
				    ctx.current);
		if (r < 0) {
			return r;
		}
	}

	return 0;
}

#else

static inline bool append_u8(struct coap_packet *cpkt, uint8_t data)
{
	if (!cpkt) {
		return false;
	}

	if (cpkt->max_len - cpkt->offset < 1) {
		return false;
	}

	cpkt->data[cpkt->offset++] = data;

	return true;
}

static int format_uri(const char * const *path, struct coap_packet *response)
{
	const char * const *p;
	char *prefix = "</";
	bool res;

	if (!path) {
		return -EINVAL;
	}

	res = append(response, (uint8_t *) prefix, strlen(prefix));
	if (!res) {
		return -ENOMEM;
	}

	for (p = path; *p; ) {
		res = append(response, (uint8_t *) *p, strlen(*p));
		if (!res) {
			return -ENOMEM;
		}

		p++;
		if (*p) {
			res = append_u8(response, (uint8_t) '/');
			if (!res) {
				return -ENOMEM;
			}
		}
	}

	res = append_u8(response, (uint8_t) '>');
	if (!res) {
		return -ENOMEM;
	}

	return 0;
}

static int format_attributes(const char * const *attributes,
			     struct coap_packet *response)
{
	const char * const *attr;
	bool res;

	if (!attributes) {
		return 0;
	}

	for (attr = attributes; *attr; attr++) {
		res = append_u8(response, (uint8_t) ';');
		if (!res) {
			return -ENOMEM;
		}

		res = append(response, (uint8_t *) *attr, strlen(*attr));
		if (!res) {
			return -ENOMEM;
		}
	}

	return 0;
}

static int format_resource(const struct coap_resource *resource,
			   struct coap_packet *response)
{
	struct coap_core_metadata *meta = resource->metadata;
	const char * const *attributes = NULL;
	int r;

	r = format_uri(resource->path, response);
	if (r < 0) {
		return r;
	}

	if (meta != NULL && meta->attributes != NULL) {
		attributes = meta->attributes;
	}

	return format_attributes(attributes, response);
}

int coap_well_known_core_get_len(struct coap_resource *resources,
				 size_t resources_len,
				 const struct coap_packet *request,
				 struct coap_packet *response,
				 uint8_t *data, uint16_t data_len)
{
	struct coap_option query;
	uint8_t num_queries;
	int r;
	bool first = true;

	if (!resources || !request || !response || !data || !data_len) {
		return -EINVAL;
	}

	/* Per RFC 6690, Section 4.1, only one (or none) query parameter may be
	 * provided, use the first if multiple.
	 */
	r = coap_find_options(request, COAP_OPTION_URI_QUERY, &query, 1);
	if (r < 0) {
		return r;
	}

	num_queries = r;

	r = coap_init_response_packet(response, request, COAP_RESPONSE_CODE_CONTENT, data,
				      data_len);
	if (r < 0) {
		return -ENOMEM;
	}

	r = coap_append_option_int(response, COAP_OPTION_CONTENT_FORMAT,
				   COAP_CONTENT_FORMAT_APP_LINK_FORMAT);
	if (r < 0) {
		return -ENOMEM;
	}

	for (size_t i = 0; i < resources_len; ++i) {
		if (!match_queries_resource(&resources[i], &query, num_queries)) {
			continue;
		}

		if (first) {
			r = coap_packet_append_payload_marker(response);
			if (r < 0) {
				return -ENOMEM;
			}
			first = false;
		} else {
			r = append_u8(response, (uint8_t) ',');
			if (!r) {
				return -ENOMEM;
			}
		}

		r = format_resource(&resources[i], response);
		if (r < 0) {
			return r;
		}
	}

	return 0;
}
#endif

int coap_well_known_core_get(struct coap_resource *resource,
			     const struct coap_packet *request,
			     struct coap_packet *response,
			     uint8_t *data, uint16_t data_len)
{
	struct coap_resource *resources;
	size_t resources_len = 0;

	if (resource == NULL) {
		return -EINVAL;
	}

	resources = resource + 1;

	while (resources[resources_len].path) {
		resources_len++;
	}

	return coap_well_known_core_get_len(resources, resources_len, request, response, data,
					    data_len);
}

/* Exposing some of the APIs to CoAP unit tests in tests/net/lib/coap */
#if defined(CONFIG_COAP_TEST_API_ENABLE)
bool _coap_match_path_uri(const char * const *path,
			  const char *uri, uint16_t len)
{
	return match_path_uri(path, uri, len);
}
#endif

/*
 * SPDX-FileCopyrightText: Copyright (c) 2026 Basalte bv
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>

#include <zephyr/net/coap.h>
#include <zephyr/net/coap_link_format.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <zephyr/ztest.h>

static const char * const path_a[] = { "large", NULL };
static const char * const attrs_a[] = { "rt=\"observe\"", "if=\"core.s\"", NULL };
static struct coap_core_metadata meta_a = { .attributes = attrs_a };

static const char * const path_b[] = { "location", "1", NULL };

static const char * const path_c[] = { "switch", NULL };
static const char * const attrs_c[] = { "rt=\"actuator\"", NULL };
static struct coap_core_metadata meta_c = { .attributes = attrs_c };

static struct coap_resource test_resources[] = {
	{ .path = path_a, .metadata = &meta_a },
	{ .path = path_b },
	{ .path = path_c, .metadata = &meta_c },
};

static const char expected_doc[] =
	"</large>;rt=\"observe\";if=\"core.s\",</location/1>,</switch>;rt=\"actuator\"";

static int send_wk_req(struct coap_resource *resources, size_t num_resources, bool use_get_wrapper,
		       const char *query, int block_num, enum coap_block_size req_szx,
		       struct coap_packet *response, uint8_t *buf, uint16_t buf_len)
{
	struct coap_packet request;
	uint8_t req_buf[96];
	int r;

	r = coap_packet_init(&request, req_buf, sizeof(req_buf), COAP_VERSION_1, COAP_TYPE_CON,
			     COAP_TOKEN_MAX_LEN, coap_next_token(), COAP_METHOD_GET,
			     coap_next_id());
	zassert_equal(r, 0, "Failed to init request");

	if (query != NULL) {
		r = coap_packet_append_option(&request, COAP_OPTION_URI_QUERY,
					      (const uint8_t *)query, strlen(query));
		zassert_equal(r, 0, "Failed to append uri-query option");
	}

	if (block_num >= 0) {
		r = coap_append_option_int(&request, COAP_OPTION_BLOCK2,
					   ((unsigned int)block_num << 4) | req_szx);
		zassert_equal(r, 0, "Failed to append block2 option");
	}

	if (use_get_wrapper) {
		return coap_well_known_core_get(resources, &request, response, buf, buf_len);
	}

	return coap_well_known_core_get_len(resources, num_resources, &request, response, buf,
					    buf_len);
}

static void verify_well_known_non_framing(void)
{
	static const uint8_t token[] = { 0x4A, 0x5B, 0x6C, 0x7D };
	struct coap_packet request;
	struct coap_packet response;
	struct coap_packet parsed;
	uint8_t req_buf[96];
	uint8_t rsp_buf[256];
	uint8_t rsp_token[COAP_TOKEN_MAX_LEN];
	const uint8_t *payload;
	uint16_t payload_len = 0U;
	uint16_t req_id = (uint16_t)(coap_next_id() + 0x1234U);
	uint16_t mid1;
	uint16_t mid2;
	int r;

	/* NON request -> NON 2.05 Content with matching token and server-generated MID. */
	r = coap_packet_init(&request, req_buf, sizeof(req_buf), COAP_VERSION_1, COAP_TYPE_NON_CON,
			     sizeof(token), token, COAP_METHOD_GET, req_id);
	zassert_equal(r, 0);

	r = coap_well_known_core_get_len(test_resources, ARRAY_SIZE(test_resources), &request,
					 &response, rsp_buf, sizeof(rsp_buf));
	zassert_equal(r, 0, "NON /.well-known/core failed (%d)", r);
	zassert_equal(coap_packet_parse(&parsed, rsp_buf, response.offset, NULL, 0U), 0);
	zassert_equal(coap_header_get_type(&parsed), COAP_TYPE_NON_CON,
		      "NON request must receive a NON response");
	zassert_equal(coap_header_get_code(&parsed), COAP_RESPONSE_CODE_CONTENT);
	zassert_equal(coap_header_get_token(&parsed, rsp_token), sizeof(token));
	zassert_mem_equal(rsp_token, token, sizeof(token));
	mid1 = coap_header_get_id(&parsed);
	zassert_not_equal(mid1, req_id, "NON response must not echo request MID");
	payload = coap_packet_get_payload(&parsed, &payload_len);
	zassert_not_null(payload);
	zassert_true(payload_len > 0U);

	/* Second NON request with the same req_id gets a fresh server MID. */
	r = coap_well_known_core_get_len(test_resources, ARRAY_SIZE(test_resources), &request,
					 &response, rsp_buf, sizeof(rsp_buf));
	zassert_equal(r, 0);
	zassert_equal(coap_packet_parse(&parsed, rsp_buf, response.offset, NULL, 0U), 0);
	zassert_equal(coap_header_get_type(&parsed), COAP_TYPE_NON_CON);
	mid2 = coap_header_get_id(&parsed);
	zassert_not_equal(mid2, req_id, "Second NON response must not echo request MID");
	zassert_not_equal(mid2, mid1, "NON responses must advance server MID");

	/* CON request still receives a piggybacked ACK echoing request MID. */
	r = coap_packet_init(&request, req_buf, sizeof(req_buf), COAP_VERSION_1, COAP_TYPE_CON,
			     sizeof(token), token, COAP_METHOD_GET, req_id);
	zassert_equal(r, 0);
	r = coap_well_known_core_get_len(test_resources, ARRAY_SIZE(test_resources), &request,
					 &response, rsp_buf, sizeof(rsp_buf));
	zassert_equal(r, 0);
	zassert_equal(coap_packet_parse(&parsed, rsp_buf, response.offset, NULL, 0U), 0);
	zassert_equal(coap_header_get_type(&parsed), COAP_TYPE_ACK);
	zassert_equal(coap_header_get_id(&parsed), req_id);
	zassert_equal(coap_header_get_token(&parsed, rsp_token), sizeof(token));
	zassert_mem_equal(rsp_token, token, sizeof(token));
}

static void verify_well_known_href_queries(void)
{
	/* Keep Uri-Query strings <= 12 bytes so the default coap_option->value
	 * buffer fits without CONFIG_COAP_EXTENDED_OPTIONS_LEN.
	 */
	static const char * const path_s_h[] = { "s", "h", NULL };
	static const char * const attrs_s_h[] = { "rt=\"h\"", NULL };
	static struct coap_core_metadata meta_s_h = { .attributes = attrs_s_h };
	static const char * const path_s_i[] = { "s", "i", NULL };
	static const char * const path_h[] = { "h", NULL };
	static const char * const attrs_h[] = { "rt=\"r\"", NULL };
	static struct coap_core_metadata meta_h = { .attributes = attrs_h };
	static const char * const path_dev[] = { "dev", NULL };
	static const char * const path_a_slash[] = { "a", "", NULL };
	static const char * const path_root[] = { "", NULL };
	struct coap_resource href_resources[] = {
		{ .path = path_s_h, .metadata = &meta_s_h },
		{ .path = path_s_i },
		{ .path = path_h, .metadata = &meta_h },
		{ .path = path_dev },
		{ .path = path_a_slash },
		{ .path = path_root },
	};
	static const struct {
		const char *query;
		const char *expected;
	} cases[] = {
		{ "href=/s/h", "</s/h>;rt=\"h\"" },
		{ "href=/h", "</h>;rt=\"r\"" },
		{ "href=/s/*", "</s/h>;rt=\"h\",</s/i>" },
		{ "href=/s/i", "</s/i>" },
		{ "href=/dev", "</dev>" },
		{ "href=/a/", "</a/>" },
		{ "href=/a/*", "</a/>" },
		{ "href=/", "</>" },
	};

	for (size_t i = 0U; i < ARRAY_SIZE(cases); i++) {
		char assembled[128] = { 0 };
		size_t pos = 0U;
		struct coap_packet response;
		struct coap_packet parsed;
		uint8_t rsp_buf[256];
		const uint8_t *payload;
		uint16_t payload_len = 0U;
		bool more = true;

		for (int blk = 0; more; blk++) {
			int r = send_wk_req(href_resources, ARRAY_SIZE(href_resources), false,
					    cases[i].query,
					    IS_ENABLED(CONFIG_COAP_WELL_KNOWN_BLOCK_WISE) ? blk
											  : -1,
					    COAP_BLOCK_32, &response, rsp_buf, sizeof(rsp_buf));

			zassert_equal(r, 0, "Query %s failed (%d)", cases[i].query, r);
			zassert_equal(coap_packet_parse(&parsed, rsp_buf, response.offset, NULL,
							0U),
				      0);
			payload = coap_packet_get_payload(&parsed, &payload_len);
			zassert_not_null(payload, "Query %s missing payload", cases[i].query);
			zassert_true(pos + payload_len < sizeof(assembled));
			memcpy(&assembled[pos], payload, payload_len);
			pos += payload_len;

			if (IS_ENABLED(CONFIG_COAP_WELL_KNOWN_BLOCK_WISE)) {
				int b2 = coap_get_option_int(&parsed, COAP_OPTION_BLOCK2);

				zassert_true(b2 >= 0);
				more = GET_MORE(b2);
			} else {
				more = false;
			}
		}

		zassert_equal(pos, strlen(cases[i].expected), "Query %s len mismatch (got %.*s)",
			      cases[i].query, (int)pos, assembled);
		zassert_mem_equal(assembled, cases[i].expected, pos,
				  "Query %s payload mismatch", cases[i].query);
	}
}

static void verify_well_known_short_workspace(void)
{
	static const uint8_t token[] = { 0x10, 0x20, 0x30, 0x40 };
	struct coap_packet request;
	struct coap_packet response;
	uint8_t req_buf[64];
	uint8_t rsp_buf[256];
	uint16_t full_len;
	int r;

	r = coap_packet_init(&request, req_buf, sizeof(req_buf), COAP_VERSION_1, COAP_TYPE_CON,
			     sizeof(token), token, COAP_METHOD_GET, coap_next_id());
	zassert_equal(r, 0);

	r = coap_well_known_core_get_len(test_resources, ARRAY_SIZE(test_resources), &request,
					 &response, rsp_buf, sizeof(rsp_buf));
	zassert_equal(r, 0);
	full_len = response.offset;
	zassert_true(full_len > 8U);

	/* Every truncated non-zero workspace buffer must fail with -ENOMEM, not -EINVAL. */
	for (uint16_t short_len = 1U; short_len < full_len; short_len++) {
		r = coap_well_known_core_get_len(test_resources, ARRAY_SIZE(test_resources),
						 &request, &response, rsp_buf, short_len);
		zassert_equal(r, -ENOMEM,
			      "Short workspace len %u (full %u) returned %d instead of -ENOMEM",
			      short_len, full_len, r);
	}

	/* Zero-length buffer remains an invalid caller argument (-EINVAL). */
	r = coap_well_known_core_get_len(test_resources, ARRAY_SIZE(test_resources), &request,
					 &response, rsp_buf, 0U);
	zassert_equal(r, -EINVAL);
}

#if defined(CONFIG_COAP_WELL_KNOWN_BLOCK_WISE)

#define WK_BUF_SIZE           256U
#define WK_LARGE_PKT_BUF_SIZE 1280U
#define WK_LARGE_DOC_MAX      4096U
#define WK_LARGE_RES_MAX      72U

#define WK_DEFAULT_BLOCK_SIZE CONFIG_COAP_WELL_KNOWN_BLOCK_WISE_SIZE
#define WK_DEFAULT_SZX        coap_bytes_to_block_size(WK_DEFAULT_BLOCK_SIZE)

/* The existing block-wise tests request blocks of up to 32 bytes. */
BUILD_ASSERT(WK_DEFAULT_BLOCK_SIZE >= 32, "Tests request blocks of up to 32 bytes");

struct wk_transfer {
	uint8_t doc[WK_BUF_SIZE];
	size_t len;
	uint8_t etag[4];
};

struct wk_client {
	struct coap_resource *resources;
	size_t num_resources;
	bool use_get_wrapper;
	const char *query;
	enum coap_block_size req_szx;
	bool omit_block2_on_first;
	size_t received;
	uint8_t etag[4];
};

static uint8_t wk_pkt_buf[WK_LARGE_PKT_BUF_SIZE];
static char wk_expected_doc[WK_LARGE_DOC_MAX];
static char wk_expected_doc_b[WK_LARGE_DOC_MAX];

static struct coap_resource wk_dyn_resources[WK_LARGE_RES_MAX + 2U];
static char wk_dyn_names[WK_LARGE_RES_MAX][12];
static const char *wk_dyn_paths[WK_LARGE_RES_MAX][4];
static char wk_pad_seg[64];
static const char *wk_pad_path[2] = { wk_pad_seg, NULL };

static const char * const wk_core_path[] = { ".well-known", "core", NULL };

static const char * const attrs_bulk[] = {
	"rt=\"bulk\"", "if=\"core.p\"", "ct=60", "obs", NULL
};
static struct coap_core_metadata meta_bulk = { .attributes = attrs_bulk };

static const char * const attrs_small[] = { "rt=\"small\"", "ct=50", NULL };
static struct coap_core_metadata meta_small = { .attributes = attrs_small };

static const char * const attrs_end[] = { "rt=\"end\"", NULL };
static struct coap_core_metadata meta_end = { .attributes = attrs_end };

static const char * const attrs_other[] = { "rt=\"other\"", NULL };
static struct coap_core_metadata meta_other = { .attributes = attrs_other };

static uint16_t expected_block2_len(unsigned int val)
{
	return (val == 0U) ? 0U : (val <= 0xFFU) ? 1U : (val <= 0xFFFFU) ? 2U : 3U;
}

static size_t attrs_encoded_len(const struct coap_core_metadata *meta)
{
	size_t len = 0U;

	if (meta == NULL || meta->attributes == NULL) {
		return 0U;
	}

	for (const char * const *a = meta->attributes; *a != NULL; a++) {
		len += 1U + strlen(*a);
	}

	return len;
}

static bool resource_matches_attr(const struct coap_resource *res, const char *query_attr)
{
	const struct coap_core_metadata *meta = res->metadata;

	if (query_attr == NULL) {
		return true;
	}

	if (meta == NULL || meta->attributes == NULL) {
		return false;
	}

	for (const char * const *a = meta->attributes; *a != NULL; a++) {
		if (strcmp(*a, query_attr) == 0) {
			return true;
		}
	}

	return false;
}

static size_t build_expected_doc(const struct coap_resource *resources, size_t num_resources,
				 const char *query_attr, char *out, size_t out_cap)
{
	size_t pos = 0U;
	bool first = true;

	for (size_t i = 0U; i < num_resources; i++) {
		const struct coap_core_metadata *meta = resources[i].metadata;

		if (!resource_matches_attr(&resources[i], query_attr)) {
			continue;
		}

		if (!first) {
			zassert_true(pos + 1U < out_cap, "Expected doc buffer overflow");
			out[pos++] = ',';
		}
		first = false;

		zassert_true(pos + 2U < out_cap, "Expected doc buffer overflow");
		out[pos++] = '<';
		out[pos++] = '/';

		for (const char * const *p = resources[i].path; *p != NULL; p++) {
			size_t slen = strlen(*p);

			zassert_true(pos + slen + 2U < out_cap, "Expected doc buffer overflow");
			memcpy(&out[pos], *p, slen);
			pos += slen;

			if (*(p + 1) != NULL) {
				out[pos++] = '/';
			}
		}

		out[pos++] = '>';

		if (meta != NULL && meta->attributes != NULL) {
			for (const char * const *a = meta->attributes; *a != NULL; a++) {
				size_t alen = strlen(*a);

				zassert_true(pos + 1U + alen < out_cap,
					     "Expected doc buffer overflow");
				out[pos++] = ';';
				memcpy(&out[pos], *a, alen);
				pos += alen;
			}
		}
	}

	out[pos] = '\0';
	return pos;
}

struct wk_req_spec {
	struct coap_resource *resources;
	size_t num_resources;
	bool use_get_wrapper;
	const char *query;
	int req_num;
	enum coap_block_size req_szx;
	int exp_num;
	enum coap_block_size exp_szx;
	int exp_more;
	int exp_len;
	uint8_t *etag;
};

/* Fetch one block, verify CoAP/Block2/ETag/payload invariants, return M bit.
 * Pass exp_more < 0 to skip asserting M and exp_len < 0 for any non-empty payload.
 */
static bool wk_fetch_block(const struct wk_req_spec *s, const uint8_t **payload_out,
			   uint16_t *payload_len_out)
{
	struct coap_packet response;
	struct coap_packet parsed;
	struct coap_option block2_opt = { 0 };
	struct coap_option etag_opt = { 0 };
	const uint8_t *payload;
	uint16_t payload_len = 0U;
	unsigned int exp_block2_val;
	bool more;
	int block2;
	int r;

	r = send_wk_req(s->resources, s->num_resources, s->use_get_wrapper, s->query, s->req_num,
			s->req_szx, &response, wk_pkt_buf, sizeof(wk_pkt_buf));
	zassert_equal(r, 0, "Request failed at block %d (%d)", s->exp_num, r);

	r = coap_packet_parse(&parsed, wk_pkt_buf, response.offset, NULL, 0U);
	zassert_equal(r, 0, "Failed to parse response at block %d (%d)", s->exp_num, r);
	zassert_equal(coap_header_get_code(&parsed), COAP_RESPONSE_CODE_CONTENT,
		      "Unexpected response code");
	zassert_equal(coap_get_option_int(&parsed, COAP_OPTION_CONTENT_FORMAT),
		      COAP_CONTENT_FORMAT_APP_LINK_FORMAT,
		      "Unexpected content-format at block %d", s->exp_num);

	if (IS_ENABLED(CONFIG_SYS_HASH_FUNC32)) {
		r = coap_find_options(&parsed, COAP_OPTION_ETAG, &etag_opt, 1U);
		zassert_equal(r, 1, "No ETag at block %d", s->exp_num);
		zassert_equal(etag_opt.len, 4U, "Unexpected ETag length");

		if (s->etag != NULL) {
			if (s->exp_num == 0) {
				memcpy(s->etag, etag_opt.value, 4U);
			} else {
				zassert_mem_equal(etag_opt.value, s->etag, 4U,
						  "ETag changed at block %d", s->exp_num);
			}
		}
	}

	r = coap_find_options(&parsed, COAP_OPTION_BLOCK2, &block2_opt, 1U);
	zassert_equal(r, 1, "Missing Block2 option at block %d", s->exp_num);

	block2 = coap_get_option_int(&parsed, COAP_OPTION_BLOCK2);
	zassert_true(block2 >= 0, "Missing Block2 option");
	zassert_equal(GET_BLOCK_NUM(block2), s->exp_num, "Unexpected block number");
	zassert_equal(GET_BLOCK_SIZE(block2), s->exp_szx, "Unexpected block size");

	more = GET_MORE(block2);
	if (s->exp_more >= 0) {
		zassert_equal(more, s->exp_more != 0, "Unexpected M bit at block %d", s->exp_num);
	}

	exp_block2_val = ((unsigned int)s->exp_num << 4) | (more ? 0x08U : 0x00U) |
			 (unsigned int)s->exp_szx;
	zassert_equal(block2_opt.len, expected_block2_len(exp_block2_val),
		      "Non-canonical Block2 option length at block %d", s->exp_num);

	payload = coap_packet_get_payload(&parsed, &payload_len);
	if (s->exp_len == 0) {
		zassert_is_null(payload, "Unexpected payload for empty document");
		zassert_equal(payload_len, 0U, "Expected zero payload length");
		zassert_not_equal(wk_pkt_buf[response.offset - 1U], 0xFFU,
				  "Malformed trailing payload marker");
	} else {
		zassert_not_null(payload, "Missing payload at block %d", s->exp_num);
		if (s->exp_len > 0) {
			zassert_equal(payload_len, (uint16_t)s->exp_len,
				      "Unexpected payload len at block %d", s->exp_num);
		}
	}

	if (payload_out != NULL) {
		*payload_out = payload;
	}
	if (payload_len_out != NULL) {
		*payload_len_out = payload_len;
	}

	return more;
}

static bool wk_step_client(struct wk_client *c, int block_num, const char *expected,
			   size_t expected_len)
{
	enum coap_block_size exp_szx =
		c->omit_block2_on_first ? WK_DEFAULT_SZX : MIN(c->req_szx, WK_DEFAULT_SZX);
	uint16_t block_bytes = coap_block_size_to_bytes(exp_szx);
	size_t byte_off = (size_t)block_num * block_bytes;
	bool exp_more = (byte_off + block_bytes) < expected_len;
	size_t exp_chunk = (expected_len == 0U)
		? 0U
		: (exp_more ? block_bytes : (expected_len - byte_off));
	const struct wk_req_spec spec = {
		.resources = c->resources,
		.num_resources = c->num_resources,
		.use_get_wrapper = c->use_get_wrapper,
		.query = c->query,
		.req_num = (c->omit_block2_on_first && block_num == 0) ? -1 : block_num,
		.req_szx = exp_szx,
		.exp_num = block_num,
		.exp_szx = exp_szx,
		.exp_more = exp_more ? 1 : 0,
		.exp_len = (int)exp_chunk,
		.etag = c->etag,
	};
	const uint8_t *payload;
	uint16_t payload_len;
	bool more;

	more = wk_fetch_block(&spec, &payload, &payload_len);

	if (expected_len > 0U) {
		zassert_equal(c->received, byte_off, "Unexpected reassembly offset");
		zassert_mem_equal(payload, &expected[byte_off], payload_len,
				  "Content mismatch at block %d", block_num);
		c->received += payload_len;
	}

	return more;
}

static void verify_full_transfer(struct coap_resource *resources, size_t num_resources,
				 bool use_get_wrapper, const char *query,
				 enum coap_block_size req_szx, bool omit_block2_on_first,
				 const char *expected, size_t expected_len, uint8_t etag_out[4])
{
	struct wk_client client = {
		.resources = resources,
		.num_resources = num_resources,
		.use_get_wrapper = use_get_wrapper,
		.query = query,
		.req_szx = req_szx,
		.omit_block2_on_first = omit_block2_on_first,
	};
	enum coap_block_size exp_szx =
		omit_block2_on_first ? WK_DEFAULT_SZX : MIN(req_szx, WK_DEFAULT_SZX);
	struct coap_packet response;
	bool more = true;
	int block_num;
	int r;

	for (block_num = 0; more; block_num++) {
		more = wk_step_client(&client, block_num, expected, expected_len);
	}

	zassert_equal(client.received, expected_len, "Reassembled length mismatch");
	if (etag_out != NULL) {
		memcpy(etag_out, client.etag, sizeof(client.etag));
	}

	/* Requesting the block immediately past the final block must fail. */
	r = send_wk_req(resources, num_resources, use_get_wrapper, query, block_num, exp_szx,
			&response, wk_pkt_buf, sizeof(wk_pkt_buf));
	zassert_equal(r, -EINVAL, "Out-of-range block %d was accepted (%d)", block_num, r);
}

static void set_dyn_res(size_t slot, size_t idx, const char *seg0, const char *name_fmt,
			size_t num, const char *seg2, struct coap_core_metadata *meta)
{
	snprintk(wk_dyn_names[idx], sizeof(wk_dyn_names[idx]), name_fmt, num);
	wk_dyn_paths[idx][0] = seg0;
	wk_dyn_paths[idx][1] = wk_dyn_names[idx];
	wk_dyn_paths[idx][2] = seg2;
	wk_dyn_paths[idx][3] = NULL;

	wk_dyn_resources[slot].path = wk_dyn_paths[idx];
	wk_dyn_resources[slot].metadata = meta;
}

/* Populate wk_dyn_resources with 64 attributed resources (> 3,200 bytes total). */
static size_t populate_large_unfiltered_resources(bool with_wk_sentinel)
{
	const size_t count = 64U;
	size_t base = with_wk_sentinel ? 1U : 0U;

	memset(wk_dyn_resources, 0, sizeof(wk_dyn_resources));
	if (with_wk_sentinel) {
		wk_dyn_resources[0].path = wk_core_path;
	}

	for (size_t i = 0U; i < count; i++) {
		set_dyn_res(base + i, i, "servo", "ch%02zu", i, "status", &meta_bulk);
	}

	return count;
}

/* Build an array of resources whose RFC 6690 listing has the exact target_len. */
static size_t populate_exact_size_resources(size_t target_len, bool end_with_attr,
					    bool filtered_bulk, size_t trailing_non_matching)
{
	struct coap_core_metadata *final_meta =
		filtered_bulk ? &meta_bulk : (end_with_attr ? &meta_end : NULL);
	size_t base_entry_len = strlen("</svc/r00/item>") + attrs_encoded_len(&meta_bulk);
	size_t final_attr_len = attrs_encoded_len(final_meta);
	/* Minimum padded final resource: ',' + "</" + 1 char + ">" + attrs */
	size_t min_final = 5U + final_attr_len;
	size_t count = 0U;
	size_t cur_len = 0U;
	size_t overhead;
	size_t pad_len;

	memset(wk_dyn_resources, 0, sizeof(wk_dyn_resources));

	while (count < WK_LARGE_RES_MAX - trailing_non_matching - 1U) {
		size_t entry_len = (count > 0U ? 1U : 0U) + base_entry_len;

		if (cur_len + entry_len + min_final > target_len) {
			break;
		}

		set_dyn_res(count, count, "svc", "r%02zu", count, "item", &meta_bulk);
		cur_len += entry_len;
		count++;
	}

	overhead = (count > 0U ? 1U : 0U) + 3U + final_attr_len;
	zassert_true(target_len > cur_len + overhead,
		     "Cannot pad to target_len %zu (cur=%zu, overhead=%zu)",
		     target_len, cur_len, overhead);

	pad_len = target_len - cur_len - overhead;
	zassert_true(pad_len > 0U && pad_len < sizeof(wk_pad_seg),
		     "Unexpected pad_len %zu", pad_len);
	memset(wk_pad_seg, 'p', pad_len);
	wk_pad_seg[pad_len] = '\0';

	wk_dyn_resources[count].path = wk_pad_path;
	wk_dyn_resources[count].metadata = final_meta;
	count++;

	for (size_t i = 0U; i < trailing_non_matching; i++) {
		set_dyn_res(count, count, "skip", "x%02zu", i, NULL, &meta_other);
		count++;
	}

	return count;
}

static bool fetch_block(struct wk_transfer *tr, int block_num, enum coap_block_size size)
{
	const struct wk_req_spec spec = {
		.resources = test_resources,
		.num_resources = ARRAY_SIZE(test_resources),
		.req_num = block_num,
		.req_szx = size,
		.exp_num = block_num,
		.exp_szx = size,
		.exp_more = -1,
		.exp_len = -1,
		.etag = tr->etag,
	};
	const uint8_t *payload;
	uint16_t payload_len;
	bool more;

	more = wk_fetch_block(&spec, &payload, &payload_len);

	zassert_true(tr->len + payload_len <= sizeof(tr->doc), "Transfer too large");
	zassert_equal(tr->len, (size_t)block_num * coap_block_size_to_bytes(size),
		      "Unexpected payload offset");

	memcpy(&tr->doc[tr->len], payload, payload_len);
	tr->len += payload_len;

	return more;
}

ZTEST(coap_wellknown, test_blockwise_reassembly)
{
	struct wk_transfer tr = { 0 };
	bool more = true;

	for (int i = 0; more; i++) {
		more = fetch_block(&tr, i, COAP_BLOCK_16);
	}

	zassert_equal(tr.len, strlen(expected_doc), "Unexpected document length");
	zassert_mem_equal(tr.doc, expected_doc, tr.len, "Unexpected document content");
}

ZTEST(coap_wellknown, test_blockwise_interleaved)
{
	struct wk_transfer tr_a = { 0 };
	struct wk_transfer tr_b = { 0 };
	bool more_a = true;
	bool more_b = true;

	/* Two interleaved transfers with different block sizes must not
	 * interfere with each other.
	 */
	for (int i = 0; more_a || more_b; i++) {
		if (more_a) {
			more_a = fetch_block(&tr_a, i, COAP_BLOCK_16);
		}
		if (more_b) {
			more_b = fetch_block(&tr_b, i, COAP_BLOCK_32);
		}
	}

	zassert_equal(tr_a.len, strlen(expected_doc), "Unexpected document length");
	zassert_mem_equal(tr_a.doc, expected_doc, tr_a.len, "Unexpected document content");
	zassert_equal(tr_b.len, strlen(expected_doc), "Unexpected document length");
	zassert_mem_equal(tr_b.doc, expected_doc, tr_b.len, "Unexpected document content");
}

ZTEST(coap_wellknown, test_blockwise_fresh_after_abandoned)
{
	struct wk_transfer tr = { 0 };
	const uint8_t *payload;
	uint16_t payload_len;
	size_t doc_len = strlen(expected_doc);
	bool expected_more = doc_len > WK_DEFAULT_BLOCK_SIZE;
	size_t expected_len = MIN(doc_len, (size_t)WK_DEFAULT_BLOCK_SIZE);

	/* Abandon a transfer after two blocks */
	(void)fetch_block(&tr, 0, COAP_BLOCK_16);
	(void)fetch_block(&tr, 1, COAP_BLOCK_16);

	/* A fresh request without a block2 option starts over at block 0 */
	{
		const struct wk_req_spec spec = {
			.resources = test_resources,
			.num_resources = ARRAY_SIZE(test_resources),
			.req_num = -1,
			.req_szx = COAP_BLOCK_16,
			.exp_szx = WK_DEFAULT_SZX,
			.exp_more = expected_more ? 1 : 0,
			.exp_len = (int)expected_len,
		};

		(void)wk_fetch_block(&spec, &payload, &payload_len);
	}
	zassert_mem_equal(payload, expected_doc, payload_len, "Unexpected document content");
}

ZTEST(coap_wellknown, test_blockwise_etag_representation)
{
	struct wk_transfer tr_a = { 0 };
	struct wk_transfer tr_b = { 0 };
	uint8_t etag_other[4] = { 0 };
	const struct wk_req_spec other_spec = {
		.resources = test_resources,
		.num_resources = ARRAY_SIZE(test_resources) - 1U,
		.req_szx = COAP_BLOCK_16,
		.exp_szx = COAP_BLOCK_16,
		.exp_more = -1,
		.exp_len = -1,
		.etag = etag_other,
	};

	if (!IS_ENABLED(CONFIG_SYS_HASH_FUNC32)) {
		ztest_test_skip();
	}

	/* The same resource set produces the same ETag across transfers */
	(void)fetch_block(&tr_a, 0, COAP_BLOCK_16);
	(void)fetch_block(&tr_b, 0, COAP_BLOCK_16);
	zassert_mem_equal(tr_a.etag, tr_b.etag, sizeof(tr_a.etag),
			  "ETag differs for the same resource set");

	/* A different resource set produces a different ETag */
	(void)wk_fetch_block(&other_spec, NULL, NULL);
	zassert_true(memcmp(etag_other, tr_a.etag, sizeof(tr_a.etag)) != 0,
		     "ETag did not change with the resource set");
}

ZTEST(coap_wellknown, test_blockwise_over_2048_unfiltered)
{
	static const enum coap_block_size all_sizes[] = {
		COAP_BLOCK_16, COAP_BLOCK_32, COAP_BLOCK_64, COAP_BLOCK_128,
		COAP_BLOCK_256, COAP_BLOCK_512, COAP_BLOCK_1024,
	};
	size_t count = populate_large_unfiltered_resources(false);
	size_t expected_len = build_expected_doc(wk_dyn_resources, count, NULL,
						 wk_expected_doc, sizeof(wk_expected_doc));

	zassert_true(expected_len > 3200U,
		     "Listing must exceed 3,200 bytes (got %zu)", expected_len);

	/* Discovery starting without a Block2 option uses default block size. */
	verify_full_transfer(wk_dyn_resources, count, false, NULL, WK_DEFAULT_SZX, true,
			     wk_expected_doc, expected_len, NULL);

	/* Verify every valid CoAP block size: full transfer for sizes <= default,
	 * and early-negotiation downgrade for larger sizes on block 0.
	 */
	for (size_t i = 0U; i < ARRAY_SIZE(all_sizes); i++) {
		enum coap_block_size sz = all_sizes[i];

		if (coap_block_size_to_bytes(sz) <= WK_DEFAULT_BLOCK_SIZE) {
			verify_full_transfer(wk_dyn_resources, count, false, NULL, sz, false,
					     wk_expected_doc, expected_len, NULL);
		} else {
			const struct wk_req_spec spec = {
				.resources = wk_dyn_resources,
				.num_resources = count,
				.req_szx = sz,
				.exp_szx = WK_DEFAULT_SZX,
				.exp_more = 1,
				.exp_len = WK_DEFAULT_BLOCK_SIZE,
			};
			struct coap_packet response;
			const uint8_t *payload;
			uint16_t payload_len;
			int r;

			(void)wk_fetch_block(&spec, &payload, &payload_len);
			zassert_mem_equal(payload, wk_expected_doc, payload_len,
					  "Unexpected payload bytes");

			/* Non-zero block with SZX above server max must be rejected. */
			r = send_wk_req(wk_dyn_resources, count, false, NULL, 1, sz,
					&response, wk_pkt_buf, sizeof(wk_pkt_buf));
			zassert_equal(r, -EINVAL, "Oversized SZX on block 1 was accepted");
		}
	}

	/* Verify NULL-terminated coap_well_known_core_get() wrapper past 2,048 bytes. */
	count = populate_large_unfiltered_resources(true);
	expected_len = build_expected_doc(&wk_dyn_resources[1], count, NULL,
					  wk_expected_doc, sizeof(wk_expected_doc));
	zassert_true(expected_len > 3200U, "Wrapper listing must exceed 3,200 bytes");

	verify_full_transfer(wk_dyn_resources, 0U, true, NULL, WK_DEFAULT_SZX, true,
			     wk_expected_doc, expected_len, NULL);
}

ZTEST(coap_wellknown, test_blockwise_filtered_queries)
{
	const size_t bulk_count = 46U;
	const size_t small_count = 6U;
	const size_t total_count = bulk_count + small_count + 10U;
	uint8_t etag_all[4] = { 0 };
	uint8_t etag_bulk[4] = { 0 };
	uint8_t etag_small[4] = { 0 };
	uint8_t etag_none[4] = { 0 };
	size_t expected_len;

	memset(wk_dyn_resources, 0, sizeof(wk_dyn_resources));
	for (size_t i = 0U; i < total_count; i++) {
		struct coap_core_metadata *meta =
			(i < bulk_count) ? &meta_bulk :
			(i < bulk_count + small_count) ? &meta_small :
			((i % 2U == 0U) ? &meta_other : NULL);

		set_dyn_res(i, i, "dev", "q%02zu", i, "val", meta);
	}

	expected_len = build_expected_doc(wk_dyn_resources, total_count, NULL,
					  wk_expected_doc, sizeof(wk_expected_doc));
	zassert_true(expected_len > 2048U, "Expected unfiltered > 2048 bytes");
	verify_full_transfer(wk_dyn_resources, total_count, false, NULL, WK_DEFAULT_SZX,
			     false, wk_expected_doc, expected_len, etag_all);

	expected_len = build_expected_doc(wk_dyn_resources, total_count, "rt=\"bulk\"",
					  wk_expected_doc, sizeof(wk_expected_doc));
	zassert_true(expected_len > 2048U,
		     "Expected ?rt=\"bulk\" > 2048 bytes (got %zu)", expected_len);
	verify_full_transfer(wk_dyn_resources, total_count, false, "rt=\"bulk\"",
			     COAP_BLOCK_32, false, wk_expected_doc, expected_len, etag_bulk);
	verify_full_transfer(wk_dyn_resources, total_count, false, "rt=\"bulk\"",
			     WK_DEFAULT_SZX, true, wk_expected_doc, expected_len, NULL);

	expected_len = build_expected_doc(wk_dyn_resources, total_count, "ct=60",
					  wk_expected_doc, sizeof(wk_expected_doc));
	zassert_true(expected_len > 2048U, "Expected ?ct=60 > 2048 bytes");
	verify_full_transfer(wk_dyn_resources, total_count, false, "ct=60",
			     COAP_BLOCK_32, false, wk_expected_doc, expected_len, NULL);

	expected_len = build_expected_doc(wk_dyn_resources, total_count, "rt=\"small\"",
					  wk_expected_doc, sizeof(wk_expected_doc));
	zassert_true(expected_len > 0U && expected_len < 2048U,
		     "Expected ?rt=\"small\" in (0, 2048) (got %zu)", expected_len);
	verify_full_transfer(wk_dyn_resources, total_count, false, "rt=\"small\"",
			     COAP_BLOCK_16, false, wk_expected_doc, expected_len, etag_small);
	verify_full_transfer(wk_dyn_resources, total_count, false, "rt=\"small\"",
			     COAP_BLOCK_32, false, wk_expected_doc, expected_len, NULL);

	/* 0 matching resources -> valid empty response omitting the 0xFF marker. */
	verify_full_transfer(wk_dyn_resources, total_count, false, "rt=\"none\"",
			     COAP_BLOCK_16, false, "", 0U, etag_none);
	verify_full_transfer(wk_dyn_resources, total_count, false, "rt=\"none\"",
			     WK_DEFAULT_SZX, true, "", 0U, NULL);

	if (IS_ENABLED(CONFIG_SYS_HASH_FUNC32)) {
		zassert_true(memcmp(etag_all, etag_bulk, 4U) != 0,
			     "Unfiltered and ?rt=\"bulk\" ETags collided");
		zassert_true(memcmp(etag_bulk, etag_small, 4U) != 0,
			     "?rt=\"bulk\" and ?rt=\"small\" ETags collided");
		zassert_true(memcmp(etag_small, etag_none, 4U) != 0,
			     "?rt=\"small\" and ?rt=\"none\" ETags collided");
	}
}

ZTEST(coap_wellknown, test_blockwise_boundary_sizes)
{
	static const size_t boundary_sizes[] = { 2047U, 2048U, 2049U, 2080U };

	for (size_t i = 0U; i < ARRAY_SIZE(boundary_sizes); i++) {
		size_t target = boundary_sizes[i];

		/* Test both ending on '>' (no attributes) and ending on an attribute. */
		for (int end_attr = 0; end_attr <= 1; end_attr++) {
			size_t count = populate_exact_size_resources(target, end_attr != 0,
								     false, 0U);
			size_t len = build_expected_doc(wk_dyn_resources, count, NULL,
							wk_expected_doc, sizeof(wk_expected_doc));

			zassert_equal(len, target,
				      "Boundary builder produced %zu instead of %zu", len, target);

			verify_full_transfer(wk_dyn_resources, count, false, NULL,
					     COAP_BLOCK_32, false, wk_expected_doc, len, NULL);

			if (target == 2048U || target == 2049U) {
				verify_full_transfer(wk_dyn_resources, count, false, NULL,
						     COAP_BLOCK_16, false, wk_expected_doc,
						     len, NULL);
			}

			if (WK_DEFAULT_BLOCK_SIZE >= 1024) {
				verify_full_transfer(wk_dyn_resources, count, false, NULL,
						     COAP_BLOCK_512, false, wk_expected_doc,
						     len, NULL);
				verify_full_transfer(wk_dyn_resources, count, false, NULL,
						     COAP_BLOCK_1024, false, wk_expected_doc,
						     len, NULL);
			}
		}
	}

	/* Exact 2,048-byte filtered match followed by non-matching resources must
	 * emit M=0 on block 63 (not a spurious M=1 followed by an empty block 64).
	 */
	{
		size_t count = populate_exact_size_resources(2048U, true, true, 5U);
		size_t len = build_expected_doc(wk_dyn_resources, count, "rt=\"bulk\"",
						wk_expected_doc, sizeof(wk_expected_doc));

		zassert_equal(len, 2048U, "Filtered boundary builder produced %zu", len);
		verify_full_transfer(wk_dyn_resources, count, false, "rt=\"bulk\"",
				     COAP_BLOCK_32, false, wk_expected_doc, len, NULL);
	}
}

ZTEST(coap_wellknown, test_blockwise_out_of_range_and_invalid_block2)
{
	struct coap_packet response;
	size_t count;
	size_t expected_len;
	int last_block;

	/* Empty resource table: block 0 valid (no 0xFF), block 1 rejected. */
	verify_full_transfer(test_resources, 0U, false, NULL, COAP_BLOCK_16, false, "", 0U, NULL);

	count = populate_large_unfiltered_resources(false);
	expected_len = build_expected_doc(wk_dyn_resources, count, NULL,
					  wk_expected_doc, sizeof(wk_expected_doc));
	last_block = (int)((expected_len - 1U) / 32U);

	const struct {
		int num;
		enum coap_block_size szx;
	} bad_reqs[] = {
		{ last_block + 1, COAP_BLOCK_32 },
		{ last_block + 100, COAP_BLOCK_32 },
		{ 0xFFFFF, COAP_BLOCK_32 },
		{ 0x100000, COAP_BLOCK_32 },
		{ 0, COAP_BLOCK_BERT },
	};

	for (size_t i = 0U; i < ARRAY_SIZE(bad_reqs); i++) {
		int r = send_wk_req(wk_dyn_resources, count, false, NULL, bad_reqs[i].num,
				    bad_reqs[i].szx, &response, wk_pkt_buf, sizeof(wk_pkt_buf));

		zassert_equal(r, -EINVAL, "Invalid Block2 case %zu (num=%d szx=%d) returned %d",
			      i, bad_reqs[i].num, bad_reqs[i].szx, r);
	}

	/* Block2 options longer than 3 bytes (RFC 7959 Table 1) or repeated
	 * Block2 options must be rejected with -EINVAL.
	 */
	{
		static const uint8_t block2_4b[] = { 0x00, 0x00, 0x00, 0x01 };
		static const uint8_t block2_5b[] = { 0x01, 0x02, 0x03, 0x04, 0x05 };
		struct coap_packet req;
		uint8_t req_buf[96];
		int r;

		r = coap_packet_init(&req, req_buf, sizeof(req_buf), COAP_VERSION_1,
				     COAP_TYPE_CON, COAP_TOKEN_MAX_LEN, coap_next_token(),
				     COAP_METHOD_GET, coap_next_id());
		zassert_ok(r);
		zassert_ok(coap_packet_append_option(&req, COAP_OPTION_BLOCK2, block2_4b,
						     sizeof(block2_4b)));
		r = coap_well_known_core_get_len(wk_dyn_resources, count, &req, &response,
						 wk_pkt_buf, sizeof(wk_pkt_buf));
		zassert_equal(r, -EINVAL, "4-byte Block2 option was accepted (%d)", r);

		r = coap_packet_init(&req, req_buf, sizeof(req_buf), COAP_VERSION_1,
				     COAP_TYPE_CON, COAP_TOKEN_MAX_LEN, coap_next_token(),
				     COAP_METHOD_GET, coap_next_id());
		zassert_ok(r);
		zassert_ok(coap_packet_append_option(&req, COAP_OPTION_BLOCK2, block2_5b,
						     sizeof(block2_5b)));
		r = coap_well_known_core_get_len(wk_dyn_resources, count, &req, &response,
						 wk_pkt_buf, sizeof(wk_pkt_buf));
		zassert_equal(r, -EINVAL, "5-byte Block2 option was accepted (%d)", r);

		r = coap_packet_init(&req, req_buf, sizeof(req_buf), COAP_VERSION_1,
				     COAP_TYPE_CON, COAP_TOKEN_MAX_LEN, coap_next_token(),
				     COAP_METHOD_GET, coap_next_id());
		zassert_ok(r);
		zassert_ok(coap_append_option_int(&req, COAP_OPTION_BLOCK2, COAP_BLOCK_32));
		zassert_ok(coap_append_option_int(&req, COAP_OPTION_BLOCK2, COAP_BLOCK_32));
		r = coap_well_known_core_get_len(wk_dyn_resources, count, &req, &response,
						 wk_pkt_buf, sizeof(wk_pkt_buf));
		zassert_equal(r, -EINVAL, "Duplicate Block2 option was accepted (%d)", r);
	}

	/* Subsequent normal transfer must still succeed cleanly. */
	verify_full_transfer(wk_dyn_resources, count, false, NULL, COAP_BLOCK_32, false,
			     wk_expected_doc, expected_len, NULL);
}

ZTEST(coap_wellknown, test_blockwise_option_encodings_and_over_64k)
{
	/* NUM=0, M=0, SZX=0 must encode as a 0-byte canonical Block2 option. */
	{
		static const char * const short_path[] = { "a", "1", NULL };
		struct coap_resource short_res[] = { { .path = short_path } };
		const struct wk_req_spec spec = {
			.resources = short_res,
			.num_resources = ARRAY_SIZE(short_res),
			.req_szx = COAP_BLOCK_16,
			.exp_szx = COAP_BLOCK_16,
			.exp_len = 6,
		};
		const uint8_t *payload;
		uint16_t payload_len;

		(void)wk_fetch_block(&spec, &payload, &payload_len);
		zassert_mem_equal(payload, "</a/1>", 6U, "Unexpected payload content");
	}

	/* 60 * 1,107 + 59 commas = 66,479 bytes (> 65,536): exercises Block2 wire
	 * lengths 1 -> 2 -> 3 and non-truncating size_t offset skipping past 64 KiB.
	 */
	{
		static const char seg64[] =
			"0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef";
		static const char attr64[] =
			"rt=\"0123456789abcdef0123456789abcdef0123456789abcdef0123456789a\"";
		static const char * const huge_path[] = { seg64, NULL };
		static const char * const huge_attrs[] = {
			attr64, attr64, attr64, attr64, attr64, attr64, attr64, attr64,
			attr64, attr64, attr64, attr64, attr64, attr64, attr64, attr64,
			NULL
		};
		static struct coap_core_metadata huge_meta = { .attributes = huge_attrs };
		const size_t huge_count = 60U;
		const size_t per_res = 2U + 64U + 1U + 16U * (1U + 64U);
		const size_t total_len = huge_count * per_res + (huge_count - 1U);
		const int test_blocks[] = { 0, 15, 16, 4095, 4096, (int)(total_len / 16U) };
		struct coap_packet response;
		size_t single_len;
		int past_end = (int)(total_len / 16U) + 1;
		int r;

		zassert_equal(strlen(seg64), 64U);
		zassert_equal(strlen(attr64), 64U);
		zassert_true(total_len > 65536U, "Huge listing must exceed 65,536 bytes");

		memset(wk_dyn_resources, 0, sizeof(wk_dyn_resources));
		for (size_t i = 0U; i < huge_count; i++) {
			wk_dyn_resources[i].path = huge_path;
			wk_dyn_resources[i].metadata = &huge_meta;
		}

		single_len = build_expected_doc(wk_dyn_resources, 1U, NULL,
						wk_expected_doc, sizeof(wk_expected_doc));
		zassert_equal(single_len, per_res, "Unexpected single huge entry length");

		for (size_t t = 0U; t < ARRAY_SIZE(test_blocks); t++) {
			const uint8_t *payload;
			uint16_t payload_len;
			int bnum = test_blocks[t];
			size_t byte_off = (size_t)bnum * 16U;
			bool exp_more = (byte_off + 16U) < total_len;
			size_t exp_chunk = exp_more ? 16U : (total_len - byte_off);
			const struct wk_req_spec spec = {
				.resources = wk_dyn_resources,
				.num_resources = huge_count,
				.req_num = bnum,
				.req_szx = COAP_BLOCK_16,
				.exp_num = bnum,
				.exp_szx = COAP_BLOCK_16,
				.exp_more = exp_more ? 1 : 0,
				.exp_len = (int)exp_chunk,
			};

			(void)wk_fetch_block(&spec, &payload, &payload_len);

			for (uint16_t k = 0U; k < payload_len; k++) {
				size_t doc_pos = byte_off + k;
				size_t mod = doc_pos % (per_res + 1U);
				char expected_ch = (mod == per_res) ? ',' : wk_expected_doc[mod];

				zassert_equal((char)payload[k], expected_ch,
					      "Byte mismatch at doc offset %zu (block %d)",
					      doc_pos, bnum);
			}
		}

		r = send_wk_req(wk_dyn_resources, huge_count, false, NULL, past_end,
				COAP_BLOCK_16, &response, wk_pkt_buf, sizeof(wk_pkt_buf));
		zassert_equal(r, -EINVAL, "Block %d past 64KiB listing was accepted", past_end);
	}
}

ZTEST(coap_wellknown, test_blockwise_concurrent_over_2048)
{
	const size_t bulk_count = 52U;
	const size_t total_count = 70U;
	struct wk_client a = {
		.resources = wk_dyn_resources,
		.num_resources = total_count,
		.req_szx = COAP_BLOCK_16,
	};
	struct wk_client b = {
		.resources = wk_dyn_resources,
		.num_resources = total_count,
		.query = "rt=\"bulk\"",
		.req_szx = (WK_DEFAULT_BLOCK_SIZE >= 256) ? COAP_BLOCK_256 : COAP_BLOCK_32,
	};
	size_t exp_len_a;
	size_t exp_len_b;
	bool more_a = true;
	bool more_b = true;

	memset(wk_dyn_resources, 0, sizeof(wk_dyn_resources));
	for (size_t i = 0U; i < total_count; i++) {
		set_dyn_res(i, i, "servo", "c%02zu", i, "channel",
			    (i < bulk_count) ? &meta_bulk : &meta_small);
	}

	exp_len_a = build_expected_doc(wk_dyn_resources, total_count, NULL,
				       wk_expected_doc, sizeof(wk_expected_doc));
	exp_len_b = build_expected_doc(wk_dyn_resources, total_count, b.query,
				       wk_expected_doc_b, sizeof(wk_expected_doc_b));
	zassert_true(exp_len_a > 3200U, "Concurrent unfiltered transfer must exceed 3,200B");
	zassert_true(exp_len_b > 2048U, "Concurrent filtered transfer must exceed 2,048B");

	for (int blk = 0; more_a || more_b; blk++) {
		if (more_a) {
			more_a = wk_step_client(&a, blk, wk_expected_doc, exp_len_a);
		}
		if (more_b) {
			more_b = wk_step_client(&b, blk, wk_expected_doc_b, exp_len_b);
		}
	}

	zassert_equal(a.received, exp_len_a, "Client A length mismatch");
	zassert_equal(b.received, exp_len_b, "Client B length mismatch");
	if (IS_ENABLED(CONFIG_SYS_HASH_FUNC32)) {
		zassert_true(memcmp(a.etag, b.etag, sizeof(a.etag)) != 0,
			     "Unfiltered and filtered ETags must differ");
	}
}

ZTEST(coap_wellknown, test_blockwise_non_request_framing)
{
	verify_well_known_non_framing();
}

ZTEST(coap_wellknown, test_blockwise_href_queries)
{
	verify_well_known_href_queries();
}

ZTEST(coap_wellknown, test_blockwise_short_workspace_returns_enomem)
{
	verify_well_known_short_workspace();
}

ZTEST(coap_wellknown, test_blockwise_ignores_request_block1)
{
	struct coap_packet req;
	struct coap_packet response;
	struct coap_packet parsed;
	uint8_t req_buf[96];
	const uint8_t *payload;
	uint16_t payload_len = 0U;
	int block2;
	int r;

	/* Block1 on a GET /.well-known/core with Block2 must not overwrite Block2's offset/SZX. */
	zassert_ok(coap_packet_init(&req, req_buf, sizeof(req_buf), COAP_VERSION_1, COAP_TYPE_CON,
				    COAP_TOKEN_MAX_LEN, coap_next_token(), COAP_METHOD_GET,
				    coap_next_id()));
	zassert_ok(coap_append_option_int(&req, COAP_OPTION_BLOCK2, (1U << 4) | COAP_BLOCK_32));
	zassert_ok(coap_append_option_int(&req, COAP_OPTION_BLOCK1, COAP_BLOCK_16));

	r = coap_well_known_core_get_len(test_resources, ARRAY_SIZE(test_resources), &req,
					 &response, wk_pkt_buf, sizeof(wk_pkt_buf));
	zassert_equal(r, 0, "Block2+Block1 discovery request failed (%d)", r);
	zassert_ok(coap_packet_parse(&parsed, wk_pkt_buf, response.offset, NULL, 0U));
	block2 = coap_get_option_int(&parsed, COAP_OPTION_BLOCK2);
	zassert_true(block2 >= 0);
	zassert_equal(GET_BLOCK_NUM(block2), 1U, "Block1 clobbered Block2 number");
	zassert_equal(GET_BLOCK_SIZE(block2), COAP_BLOCK_32, "Block1 clobbered Block2 size");
	payload = coap_packet_get_payload(&parsed, &payload_len);
	zassert_not_null(payload);
	zassert_equal(payload_len, 32U);
	zassert_mem_equal(payload, &expected_doc[32], 32U, "Block1 clobbered Block2 offset");

	/* Out-of-range Block2 paired with Block1 NUM=0 must still be rejected with -EINVAL. */
	zassert_ok(coap_packet_init(&req, req_buf, sizeof(req_buf), COAP_VERSION_1, COAP_TYPE_CON,
				    COAP_TOKEN_MAX_LEN, coap_next_token(), COAP_METHOD_GET,
				    coap_next_id()));
	zassert_ok(coap_append_option_int(&req, COAP_OPTION_BLOCK2, (10U << 4) | COAP_BLOCK_32));
	zassert_ok(coap_append_option_int(&req, COAP_OPTION_BLOCK1, COAP_BLOCK_32));
	r = coap_well_known_core_get_len(test_resources, ARRAY_SIZE(test_resources), &req,
					 &response, wk_pkt_buf, sizeof(wk_pkt_buf));
	zassert_equal(r, -EINVAL, "Block1 bypassed out-of-range Block2 rejection (%d)", r);

	/* Block1 alone (without Block2) must not advance the Block2 response offset. */
	zassert_ok(coap_packet_init(&req, req_buf, sizeof(req_buf), COAP_VERSION_1, COAP_TYPE_CON,
				    COAP_TOKEN_MAX_LEN, coap_next_token(), COAP_METHOD_GET,
				    coap_next_id()));
	zassert_ok(coap_append_option_int(&req, COAP_OPTION_BLOCK1, (1U << 4) | COAP_BLOCK_32));
	r = coap_well_known_core_get_len(test_resources, ARRAY_SIZE(test_resources), &req,
					 &response, wk_pkt_buf, sizeof(wk_pkt_buf));
	zassert_equal(r, 0);
	zassert_ok(coap_packet_parse(&parsed, wk_pkt_buf, response.offset, NULL, 0U));
	block2 = coap_get_option_int(&parsed, COAP_OPTION_BLOCK2);
	zassert_true(block2 >= 0);
	zassert_equal(GET_BLOCK_NUM(block2), 0U, "Block1 alone advanced Block2 number");
	payload = coap_packet_get_payload(&parsed, &payload_len);
	zassert_not_null(payload);
	zassert_mem_equal(payload, expected_doc, payload_len);
}

ZTEST_SUITE(coap_wellknown, NULL, NULL, NULL, NULL, NULL);

#else /* !CONFIG_COAP_WELL_KNOWN_BLOCK_WISE */

ZTEST(coap_wellknown_simple, test_non_blockwise_empty_and_filtered)
{
	struct coap_packet response;
	struct coap_packet parsed;
	uint8_t rsp_buf[256];
	const uint8_t *payload;
	uint16_t payload_len = 0U;
	int r;

	r = send_wk_req(test_resources, ARRAY_SIZE(test_resources), false, NULL, -1,
			COAP_BLOCK_16, &response, rsp_buf, sizeof(rsp_buf));
	zassert_equal(r, 0);
	zassert_equal(coap_packet_parse(&parsed, rsp_buf, response.offset, NULL, 0U), 0);
	payload = coap_packet_get_payload(&parsed, &payload_len);
	zassert_not_null(payload);
	zassert_equal(payload_len, strlen(expected_doc));
	zassert_mem_equal(payload, expected_doc, payload_len);

	/* Non-matching query must omit the 0xFF payload marker so the response is valid. */
	r = send_wk_req(test_resources, ARRAY_SIZE(test_resources), false, "rt=\"none\"", -1,
			COAP_BLOCK_16, &response, rsp_buf, sizeof(rsp_buf));
	zassert_equal(r, 0);
	zassert_equal(coap_packet_parse(&parsed, rsp_buf, response.offset, NULL, 0U), 0,
		      "Empty non-blockwise response failed to parse");
	zassert_not_equal(rsp_buf[response.offset - 1U], 0xFFU,
			  "Malformed trailing payload marker");
	payload = coap_packet_get_payload(&parsed, &payload_len);
	zassert_is_null(payload);
	zassert_equal(payload_len, 0U);
}

ZTEST(coap_wellknown_simple, test_non_blockwise_non_request_framing)
{
	verify_well_known_non_framing();
}

ZTEST(coap_wellknown_simple, test_non_blockwise_href_queries)
{
	verify_well_known_href_queries();
}

ZTEST(coap_wellknown_simple, test_non_blockwise_short_workspace_returns_enomem)
{
	verify_well_known_short_workspace();
}

ZTEST_SUITE(coap_wellknown_simple, NULL, NULL, NULL, NULL, NULL);

#endif /* CONFIG_COAP_WELL_KNOWN_BLOCK_WISE */

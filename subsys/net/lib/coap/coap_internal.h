/*
 * Copyright (c) 2026 Xiang Pan <panezion@outlook.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_SUBSYS_NET_LIB_COAP_INTERNAL_H_
#define ZEPHYR_SUBSYS_NET_LIB_COAP_INTERNAL_H_

#include <stdint.h>
#include <zephyr/net/coap.h>

/* RFC 7252 Section 5.2.3: answer CON requests with a piggybacked ACK echoing
 * the request MID, and NON requests with a NON reply carrying a fresh server
 * MID, preserving the request token in both cases.
 */
static inline int coap_init_response_packet(struct coap_packet *response,
					    const struct coap_packet *request,
					    uint8_t code, uint8_t *buf, uint16_t buf_len)
{
	uint8_t token[COAP_TOKEN_MAX_LEN];
	uint8_t tkl = coap_header_get_token(request, token);
	uint8_t type = (coap_header_get_type(request) == COAP_TYPE_CON) ? COAP_TYPE_ACK
									: COAP_TYPE_NON_CON;
	uint16_t id = (type == COAP_TYPE_ACK) ? coap_header_get_id(request) : coap_next_id();

	return coap_packet_init(response, buf, buf_len, COAP_VERSION_1, type, tkl, token, code, id);
}

#endif /* ZEPHYR_SUBSYS_NET_LIB_COAP_INTERNAL_H_ */

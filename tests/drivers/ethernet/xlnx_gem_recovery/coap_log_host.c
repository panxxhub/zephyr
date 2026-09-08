/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 */
#include <assert.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdatomic.h>
#include <string.h>
#include <sys/types.h>

#define CONFIG_COAP_EXTENDED_OPTIONS_LEN       1
#define CONFIG_COAP_EXTENDED_OPTIONS_LEN_VALUE 68
#define CONFIG_COAP_SERVER_TRUNCATE_MSGS       0
#define IS_ENABLED(value)                      (value)
#define MIN(a, b)                              ((a) < (b) ? (a) : (b))
#define COAP_SERVER_WIRE_MESSAGE_SIZE          1280
#define MAX_OPTIONS                            16
#define ZSOCK_MSG_DONTWAIT                     1
#define ZSOCK_MSG_TRUNC                        2
#define net_sad(addr)                          (addr)
#define CONFIG_LOG_RATELIMIT                   1
#define CONFIG_LOG_RATELIMIT_FALLBACK_DROP     1
#define LOG_LEVEL_ERR                          1
#define TEST_DEBUG_LOGGING                     0
#define unlikely(value)                        (value)
#define LOG_ERR(...)                           slow_uart(__VA_ARGS__)
#define NET_ERR(...)                           slow_uart(__VA_ARGS__)
#define LOG_DBG(...)                                                                               \
	do {                                                                                       \
		if (TEST_DEBUG_LOGGING) {                                                          \
			slow_uart(__VA_ARGS__);                                                    \
		}                                                                                  \
	} while (0)
#define NET_DBG(...)      LOG_DBG(__VA_ARGS__)
#define Z_LOG(level, ...) slow_uart(__VA_ARGS__)
typedef atomic_int atomic_t;
#define atomic_get(value)   atomic_load(value)
#define atomic_clear(value) atomic_exchange(value, 0)
#define atomic_inc(value)   atomic_fetch_add(value, 1)
static bool atomic_cas(atomic_t *value, int old, int next)
{
	return atomic_compare_exchange_strong(value, &old, next);
}
/* NATIVE RATELIMIT MACROS */
static uint64_t now_us;
static uint32_t uart_lines, server_lines, option_lines, skipped_lines;
static uint8_t datagram[1200];
static size_t datagram_len;
struct net_sockaddr_storage {
	uint32_t unused;
};
typedef size_t net_socklen_t;
/* DECLARATIONS */
static uint32_t k_uptime_get_32(void)
{
	return now_us / 1000;
}
static void slow_uart(const char *format, ...)
{
	if (strstr(format, "Failed To parse")) {
		server_lines++;
	} else if (strstr(format, "sizeof(coap_option")) {
		option_lines++;
	} else if (strstr(format, "Skipped")) {
		skipped_lines++;
	}
	uart_lines++;
	now_us += 6000;
}
static bool u16_add_overflow(uint16_t a, uint16_t b, uint16_t *result)
{
	return __builtin_add_overflow(a, b, result);
}
static ssize_t zsock_recvfrom(int fd, void *buf, size_t size, int flags,
			      struct net_sockaddr_storage *addr, net_socklen_t *addr_len)
{
	assert(datagram_len <= size);
	memcpy(buf, datagram, datagram_len);
	return datagram_len;
}
/* PARSER FUNCTIONS */
/* SERVER REJECTION PATH */
int main(int argc, char **argv)
{
	/* Alternate reserved token length and an oversized 69-byte option.
	 * Both are actual 1200-byte datagrams, not mocked parser return codes.
	 */
	for (uint32_t kind = 0; kind < 2; kind++) {
		uint64_t start = 1000000 + (uint64_t)kind * 61000000;

		now_us = start;
		uart_lines = server_lines = option_lines = skipped_lines = 0;
		uint32_t heartbeats = 0;
		uint64_t last_heartbeat = start;

		memset(datagram, 0, sizeof(datagram));
		datagram_len = sizeof(datagram);
		datagram[0] = kind == 0 ? 0x49 : 0x40;
		datagram[1] = 1;
		datagram[4] = 0x0d; /* option length = next byte + 13 */
		datagram[5] = 56;
		for (uint32_t n = 0; n < 108000; n++) {
			uint64_t arrival = start + (uint64_t)n * 1000000 / 1800;

			if (now_us < arrival) {
				now_us = arrival;
			}
			now_us += 50; /* Injected non-logging CPU cost per iteration. */
			int ret = coap_server_process(0);

			assert(ret == (kind == 0 ? -EBADMSG : -EILSEQ));
			/* Each site may print its error plus a skipped-count line.
			 * Two sites cost at most 24 ms at an interval boundary.
			 * Require bounded heartbeat gaps AND no accumulating backlog:
			 * plain per-datagram errors exceed the arrival-lag budget.
			 */
			assert(now_us - last_heartbeat < 25000);
			assert(now_us - arrival < 25000);
			last_heartbeat = now_us;
			heartbeats++;
		}
		assert(heartbeats == 108000 && now_us - start <= 60000000);
		assert(server_lines >= 59 && server_lines <= 61);
		assert(kind == 0 ? option_lines == 0 : (option_lines >= 59 && option_lines <= 61));
		assert(skipped_lines > 0 && uart_lines <= (kind + 1) * 122);
		/* A well-formed request still parses after the storm. */
		struct coap_packet packet;
		struct coap_option options[MAX_OPTIONS];

		datagram[0] = 0x40;
		assert(coap_packet_parse(&packet, datagram, 4, options, MAX_OPTIONS) == 0);
	}
	return 0;
}

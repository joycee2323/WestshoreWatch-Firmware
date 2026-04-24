#include "dns_server.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/inet.h"

#include <string.h>
#include <errno.h>
#include <stdint.h>

static const char *TAG = "DNS_SRV";

static TaskHandle_t s_dns_task    = NULL;
static volatile bool s_running    = false;
static uint32_t      s_redirect_ip = 0;  /* network byte order */

/* ─────────────────────────────────────────────────────────────────────────────
 * DNS wire format
 * ───────────────────────────────────────────────────────────────────────────── */
#pragma pack(push, 1)
typedef struct {
    uint16_t id;
    uint16_t flags;
    uint16_t qd_count;
    uint16_t an_count;
    uint16_t ns_count;
    uint16_t ar_count;
} dns_header_t;
#pragma pack(pop)

/* Flags: QR=1, OPCODE=0, AA=1, TC=0, RD=1, RA=1, Z=0, RCODE=0 → 0x8580 */
#define DNS_FLAGS_RESPONSE   0x8580
#define DNS_TYPE_A           1
#define DNS_CLASS_IN         1

/* Walk a DNS qname starting at `offset`. Returns the offset of the byte
 * following the terminating 0x00, or -1 if the name is malformed or runs
 * past `buf_len`. Compression pointers are accepted (length-2 terminator). */
static int dns_skip_qname(const uint8_t *buf, int buf_len, int offset)
{
    while (offset < buf_len) {
        uint8_t len = buf[offset];
        if (len == 0) {
            return offset + 1;
        }
        if ((len & 0xC0) == 0xC0) {
            /* Compression pointer is 2 bytes and terminates the name. */
            if (offset + 1 >= buf_len) return -1;
            return offset + 2;
        }
        if (len > 63) return -1;
        offset += len + 1;
    }
    return -1;
}

/* ─────────────────────────────────────────────────────────────────────────────
 * DNS task
 * ───────────────────────────────────────────────────────────────────────────── */
static void dns_task(void *arg)
{
    (void)arg;

    while (s_running) {
        int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (sock < 0) {
            ESP_LOGE(TAG, "socket() failed: errno %d", errno);
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        struct sockaddr_in addr = {
            .sin_family      = AF_INET,
            .sin_addr.s_addr = htonl(INADDR_ANY),
            .sin_port        = htons(53),
        };

        if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
            ESP_LOGE(TAG, "bind(53) failed: errno %d", errno);
            close(sock);
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        /* 2-second recv timeout so we can notice s_running flips to false. */
        struct timeval tv = { .tv_sec = 2, .tv_usec = 0 };
        setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

        ESP_LOGI(TAG, "Captive DNS listening on UDP/53 → %08lx",
                 (unsigned long)s_redirect_ip);

        uint8_t rx[512];
        uint8_t tx[600];

        while (s_running) {
            struct sockaddr_in src;
            socklen_t src_len = sizeof(src);
            int n = recvfrom(sock, rx, sizeof(rx), 0,
                             (struct sockaddr *)&src, &src_len);
            if (n < 0) {
                if (errno == EAGAIN || errno == EWOULDBLOCK) continue;
                ESP_LOGW(TAG, "recvfrom errno %d — rebinding", errno);
                break;
            }
            if (n < (int)sizeof(dns_header_t) + 5) continue;

            dns_header_t *qh = (dns_header_t *)rx;
            uint16_t qd = ntohs(qh->qd_count);
            if (qd < 1) continue;

            /* Skip QR=1 responses (shouldn't happen, but defend). */
            if (ntohs(qh->flags) & 0x8000) continue;

            /* We only answer the first question. That's all captive-portal
             * probes ever send and it keeps the parser bounded. */
            int q_name_start = sizeof(dns_header_t);
            int q_name_end   = dns_skip_qname(rx, n, q_name_start);
            if (q_name_end < 0 || q_name_end + 4 > n) continue;

            uint16_t qtype = ((uint16_t)rx[q_name_end] << 8) |
                              (uint16_t)rx[q_name_end + 1];
            int question_len = (q_name_end + 4) - q_name_start;

            /* Copy header + first question into response buffer. */
            int total = sizeof(dns_header_t) + question_len;
            if (total > (int)sizeof(tx)) continue;
            memcpy(tx, rx, total);

            dns_header_t *rh = (dns_header_t *)tx;
            rh->flags    = htons(DNS_FLAGS_RESPONSE);
            rh->qd_count = htons(1);
            rh->an_count = htons(qtype == DNS_TYPE_A ? 1 : 0);
            rh->ns_count = 0;
            rh->ar_count = 0;

            if (qtype == DNS_TYPE_A) {
                /* Name = compression pointer to offset 0x000C (start of
                 * question name), type A, class IN, TTL 60, RDLENGTH 4. */
                uint8_t ans[16];
                uint16_t name_ptr = htons(0xC000 | (uint16_t)q_name_start);
                uint16_t type_a   = htons(DNS_TYPE_A);
                uint16_t class_in = htons(DNS_CLASS_IN);
                uint32_t ttl      = htonl(60);
                uint16_t rdlen    = htons(4);

                memcpy(&ans[0],  &name_ptr, 2);
                memcpy(&ans[2],  &type_a,   2);
                memcpy(&ans[4],  &class_in, 2);
                memcpy(&ans[6],  &ttl,      4);
                memcpy(&ans[10], &rdlen,    2);
                memcpy(&ans[12], &s_redirect_ip, 4);

                if (total + (int)sizeof(ans) > (int)sizeof(tx)) continue;
                memcpy(tx + total, ans, sizeof(ans));
                total += sizeof(ans);
            }

            int sent = sendto(sock, tx, total, 0,
                              (struct sockaddr *)&src, src_len);
            if (sent < 0) {
                ESP_LOGD(TAG, "sendto errno %d", errno);
            }
        }

        close(sock);
    }

    ESP_LOGI(TAG, "DNS task exiting");
    s_dns_task = NULL;
    vTaskDelete(NULL);
}

/* ─────────────────────────────────────────────────────────────────────────────
 * Public API
 * ───────────────────────────────────────────────────────────────────────────── */
esp_err_t dns_server_start(const char *redirect_ip_str)
{
    if (s_running) return ESP_OK;
    if (!redirect_ip_str) return ESP_ERR_INVALID_ARG;

    ip4_addr_t ip;
    if (!ip4addr_aton(redirect_ip_str, &ip)) {
        return ESP_ERR_INVALID_ARG;
    }
    s_redirect_ip = ip.addr;  /* lwIP stores in network byte order */

    s_running = true;
    BaseType_t ok = xTaskCreate(dns_task, "dns_srv", 4096,
                                NULL, 5, &s_dns_task);
    if (ok != pdPASS) {
        s_running = false;
        return ESP_FAIL;
    }
    return ESP_OK;
}

void dns_server_stop(void)
{
    s_running = false;
}

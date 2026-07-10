#include "detection_queue.h"
#include "esp_log.h"
#include "esp_spiffs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <string.h>
#include <stdio.h>

static const char *TAG = "DET_QUEUE";

/* ── On-disk layout ───────────────────────────────────────────────────────── */
#define RING_PATH      "/storage/det_ring.bin"
#define RING_MAGIC     0x44455451   /* "DETQ" */
/* v2: odid_detection_t grew band/channel fields, so SLOT_SIZE changed. The
 * resume check keys on version (not slot size), so bump this to discard any
 * pre-existing v1 buffer whose slot layout no longer matches. */
#define RING_VERSION   2
#define SLOT_SIZE      sizeof(odid_detection_t)

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint32_t version;
    uint32_t head;          /* index of oldest item (next to pop)  */
    uint32_t tail;          /* index of next write slot             */
    uint32_t count;         /* number of live items                 */
    uint32_t capacity;
} ring_header_t;

#define HEADER_SIZE  sizeof(ring_header_t)

static SemaphoreHandle_t s_mutex;
static ring_header_t     s_hdr;
static bool              s_mounted;

/* ── File I/O helpers ─────────────────────────────────────────────────────── */
static bool write_header(void)
{
    FILE *f = fopen(RING_PATH, "r+b");
    if (!f) return false;
    bool ok = fwrite(&s_hdr, HEADER_SIZE, 1, f) == 1;
    fclose(f);
    return ok;
}

static size_t slot_offset(uint32_t idx)
{
    return HEADER_SIZE + (size_t)idx * SLOT_SIZE;
}

static bool write_slot(uint32_t idx, const odid_detection_t *det)
{
    FILE *f = fopen(RING_PATH, "r+b");
    if (!f) return false;
    fseek(f, (long)slot_offset(idx), SEEK_SET);
    bool ok = fwrite(det, SLOT_SIZE, 1, f) == 1;
    fclose(f);
    return ok;
}

static bool read_slot(uint32_t idx, odid_detection_t *det)
{
    FILE *f = fopen(RING_PATH, "rb");
    if (!f) return false;
    fseek(f, (long)slot_offset(idx), SEEK_SET);
    bool ok = fread(det, SLOT_SIZE, 1, f) == 1;
    fclose(f);
    return ok;
}

/* ── Public API ───────────────────────────────────────────────────────────── */
esp_err_t detection_queue_init(void)
{
    esp_vfs_spiffs_conf_t spiffs_cfg = {
        .base_path       = "/storage",
        .partition_label = "storage",
        .max_files       = 2,
        .format_if_mount_failed = true,
    };
    esp_err_t err = esp_vfs_spiffs_register(&spiffs_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SPIFFS mount failed: %s", esp_err_to_name(err));
        return err;
    }
    s_mounted = true;

    s_mutex = xSemaphoreCreateMutex();
    if (!s_mutex) return ESP_ERR_NO_MEM;

    /* Try to read existing ring buffer */
    FILE *f = fopen(RING_PATH, "rb");
    if (f) {
        if (fread(&s_hdr, HEADER_SIZE, 1, f) == 1 &&
            s_hdr.magic == RING_MAGIC && s_hdr.version == RING_VERSION &&
            s_hdr.capacity == DETECTION_QUEUE_CAPACITY) {
            fclose(f);
            ESP_LOGI(TAG, "resumed ring buffer: %lu items buffered",
                     (unsigned long)s_hdr.count);
            return ESP_OK;
        }
        fclose(f);
    }

    /* Create fresh ring buffer file */
    s_hdr = (ring_header_t){
        .magic    = RING_MAGIC,
        .version  = RING_VERSION,
        .head     = 0,
        .tail     = 0,
        .count    = 0,
        .capacity = DETECTION_QUEUE_CAPACITY,
    };

    f = fopen(RING_PATH, "wb");
    if (!f) {
        ESP_LOGE(TAG, "failed to create ring file");
        return ESP_FAIL;
    }
    fwrite(&s_hdr, HEADER_SIZE, 1, f);

    /* Pre-allocate slots with zeros */
    odid_detection_t empty;
    memset(&empty, 0, sizeof(empty));
    for (uint32_t i = 0; i < DETECTION_QUEUE_CAPACITY; i++) {
        fwrite(&empty, SLOT_SIZE, 1, f);
    }
    fclose(f);

    ESP_LOGI(TAG, "created ring buffer: %d slots, %zu bytes/slot",
             DETECTION_QUEUE_CAPACITY, SLOT_SIZE);
    return ESP_OK;
}

esp_err_t detection_queue_push(const odid_detection_t *det)
{
    if (!det) return ESP_ERR_INVALID_ARG;
    xSemaphoreTake(s_mutex, portMAX_DELAY);

    if (s_hdr.count >= s_hdr.capacity) {
        /* Drop oldest */
        s_hdr.head = (s_hdr.head + 1) % s_hdr.capacity;
        s_hdr.count--;
        ESP_LOGW(TAG, "ring full — dropped oldest detection");
    }

    if (!write_slot(s_hdr.tail, det)) {
        xSemaphoreGive(s_mutex);
        return ESP_FAIL;
    }

    s_hdr.tail = (s_hdr.tail + 1) % s_hdr.capacity;
    s_hdr.count++;
    write_header();

    xSemaphoreGive(s_mutex);
    return ESP_OK;
}

esp_err_t detection_queue_pop(odid_detection_t *det)
{
    if (!det) return ESP_ERR_INVALID_ARG;
    xSemaphoreTake(s_mutex, portMAX_DELAY);

    if (s_hdr.count == 0) {
        xSemaphoreGive(s_mutex);
        return ESP_ERR_NOT_FOUND;
    }

    if (!read_slot(s_hdr.head, det)) {
        xSemaphoreGive(s_mutex);
        return ESP_FAIL;
    }

    s_hdr.head = (s_hdr.head + 1) % s_hdr.capacity;
    s_hdr.count--;
    write_header();

    xSemaphoreGive(s_mutex);
    return ESP_OK;
}

int detection_queue_count(void)
{
    return (int)s_hdr.count;
}

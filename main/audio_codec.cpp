// Audio backend for the M5Stack Tab5.
//
// Same extern "C" interface as the S3 forks' audio_i2s.cpp (audio_init /
// audio_write_s32 / the three counters), and the same shape: np2kai on one
// core converts and enqueues, a task on the other drains into the codec with a
// blocking write so the emulator never stalls on audio.
//
// What differs from the S3 boards: the Tab5 does not have a bare I2S amplifier
// on three GPIOs, it has a codec behind I2C with the speaker enable on an I/O
// expander. All of that is the BSP's job — bsp_audio_init() brings up I2S and
// bsp_audio_codec_speaker_init() returns a handle that takes PCM. So there is
// no pin table here at all.

#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/stream_buffer.h"
#include "freertos/idf_additions.h"  // xStreamBufferCreateWithCaps
#include "esp_heap_caps.h"

#include "bsp/esp-bsp.h"
#include "esp_codec_dev.h"

#include "board_pins.h"
#include "pie_simd.h"  // np2kai shim: falls back to plain C off Xtensa/S3

extern "C" int ets_printf(const char *fmt, ...);

static bool s_ok = false;
static int s_bufframes = 0;
static int16_t *s_buf = nullptr;                // conversion scratch (stereo int16)
static StreamBufferHandle_t s_ring = nullptr;   // core1 -> core0 PCM stream (bytes)
static esp_codec_dev_handle_t s_spk = nullptr;  // BSP speaker codec
static int s_volume = 70;                       // ES8388 output level, 0-100

extern "C" volatile int g_audio_peak = 0;   // max |sample| (>32767 = clipping)
extern "C" volatile int g_audio_drops = 0;  // stereo frames the ring refused
extern "C" volatile int g_audio_under = 0;  // ring-empty events (crackle)

// ---- core 0: drain the ring into the codec --------------------------------
static void audio_task(void *arg) {
    (void)arg;
    // 4KB = 1024 stereo frames = ~46ms per codec write. On the S3 this was 1KB
    // because the ring behind it was small; here it just means fewer wakeups.
    static uint8_t rx[4096];
    size_t carry = 0;  // bytes of a split stereo frame held back from last time
    for (;;) {
        size_t n = xStreamBufferReceive(s_ring, rx + carry, sizeof(rx) - carry, pdMS_TO_TICKS(100));
        if (n == 0) {
            g_audio_under++;
            continue;
        }
        n += carry;
        // A stereo s16 frame is 4 bytes and the ring is byte-oriented, so what
        // comes back is not always a whole number of frames. Handing the codec
        // a partial frame shifts left and right for the rest of the session —
        // that is exactly the "clicking" bug the split build hit. Carry the
        // remainder into the next read instead.
        const size_t play = n & ~(size_t)3;
        carry = n - play;
        if (play) {
            esp_codec_dev_write(s_spk, rx, play);
        }
        if (carry) {
            memmove(rx, rx + play, carry);
        }
    }
}

// ---- init -----------------------------------------------------------------
// rate = sample rate, maxframes = the biggest block audio_write_s32 will get.
extern "C" bool audio_init(int rate, int maxframes) {
    if (maxframes <= 0) {
        return false;
    }
    if (bsp_audio_init(nullptr) != ESP_OK) {
        ets_printf("audio: bsp_audio_init FAIL\n");
        return false;
    }
    s_spk = bsp_audio_codec_speaker_init();
    if (!s_spk) {
        ets_printf("audio: codec speaker init FAIL\n");
        return false;
    }

    esp_codec_dev_sample_info_t fs = {};
    fs.bits_per_sample = 16;
    fs.channel = 2;
    fs.sample_rate = (uint32_t)rate;
    if (esp_codec_dev_open(s_spk, &fs) != ESP_OK) {
        ets_printf("audio: codec open FAIL\n");
        return false;
    }
    esp_codec_dev_set_out_vol(s_spk, s_volume);

    s_bufframes = maxframes;
    // 16-byte align the scratch: the PIE fast path requires it on the S3, and
    // keeping the alignment costs nothing here.
    const size_t scratch = (size_t)maxframes * 2 * sizeof(int16_t) + 16;
    void *raw = heap_caps_malloc(scratch, MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL);
    if (!raw) {
        ets_printf("audio: scratch alloc FAIL\n");
        return false;
    }
    s_buf = (int16_t *)(((uintptr_t)raw + 15) & ~(uintptr_t)15);

    // 16 blocks of 20ms = ~320ms. The S3 forks landed on 10 blocks (~200ms)
    // because PSRAM there was 8MB and shared with the frame buffers; this board
    // has 32MB with ~11MB spare after the 13MB of emulated extended memory, so
    // the ring is sized for how long a stall it should ride out rather than for
    // what fits. A shorter ring refills more often,
    // so the producer is throttled more frequently and the stutter at a BGM
    // change (where the game loads from the card and the emulator stalls) is
    // worse, not better. In PSRAM because only tasks touch it, never an ISR.
    const size_t ringbytes = (size_t)maxframes * 2 * sizeof(int16_t) * 16;
    s_ring = xStreamBufferCreateWithCaps(ringbytes, 1, MALLOC_CAP_SPIRAM);
    if (!s_ring) {
        ets_printf("audio: ring alloc FAIL (%u bytes psram)\n", (unsigned)ringbytes);
        return false;
    }

    s_ok = true;
    xTaskCreatePinnedToCore(audio_task, "audio", 4096, nullptr, 6, nullptr, 0);
    ets_printf("audio: codec ready rate=%d\n", rate);
    return true;
}

// ---- core 1: convert np2kai's SINT32 mix and enqueue ----------------------
extern "C" void audio_write_s32(const int32_t *pcm, int frames) {
    if (!s_ok || !s_ring || !s_buf || !pcm || frames <= 0) {
        return;
    }
    if (frames > s_bufframes) {
        frames = s_bufframes;
    }
    const int n = frames * 2;
    g_audio_peak = np2simd_s32_to_s16(pcm, s_buf, n, g_audio_peak);

    const size_t bytes = (size_t)n * sizeof(int16_t);
    const size_t sent = xStreamBufferSend(s_ring, s_buf, bytes, 0);
    if (sent < bytes) {
        g_audio_drops += (int)((bytes - sent) / 4);
    }
}

// ---- volume ---------------------------------------------------------------
// The ES8388's own output level, not a gain applied to the samples: the mix is
// already scaled to fit int16 (np2cfg.vol_master), and attenuating it again in
// software would only cost bits.
extern "C" void audio_set_volume(int percent) {
    if (percent < 0) {
        percent = 0;
    } else if (percent > 100) {
        percent = 100;
    }
    s_volume = percent;
    if (s_spk) {
        esp_codec_dev_set_out_vol(s_spk, s_volume);
    }
}

extern "C" int audio_get_volume(void) {
    return s_volume;
}

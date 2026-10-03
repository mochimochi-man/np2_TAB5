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
#include <math.h>

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

// ---- sound effects ----------------------------------------------------------
// The FM chip's output reaches the speaker as hard as it left the chip: at a
// 22kHz mix its upper harmonics fold back as a gritty edge, and the SSG's
// square waves keep their corners. These are optional colourings, applied
// here on the way to the codec - on this core, so the emulator pays nothing:
//
//   0  original   untouched
//   1  warm       the edge taken off the top (a gentle low-pass) and some
//                 weight added at the bottom (a low shelf)
//   2  hall       warm, in a room: a small stereo reverb, its two sides tuned
//                 apart so that a mono chip comes out wide
static volatile int s_fx = 0;
#define PRIME_MS 80
#define HIGH_MS  150
static size_t s_prime_bytes = 22050 * 4 * PRIME_MS / 1000;
static size_t s_high_bytes = 22050 * 4 * HIGH_MS / 1000;
static int s_fx_rate = 22050;

struct biquad {
    float b0, b1, b2, a1, a2;
    float z1[2], z2[2];
};

static void bq_set(biquad &f, float b0, float b1, float b2, float a0, float a1, float a2) {
    f.b0 = b0 / a0; f.b1 = b1 / a0; f.b2 = b2 / a0; f.a1 = a1 / a0; f.a2 = a2 / a0;
    f.z1[0] = f.z1[1] = f.z2[0] = f.z2[1] = 0.0f;
}

// RBJ cookbook.
static void bq_lowpass(biquad &f, float fs, float fc, float q) {
    const float w = 2.0f * (float)M_PI * fc / fs, c = cosf(w), al = sinf(w) / (2.0f * q);
    bq_set(f, (1 - c) / 2, 1 - c, (1 - c) / 2, 1 + al, -2 * c, 1 - al);
}

static void bq_lowshelf(biquad &f, float fs, float fc, float db) {
    const float A = powf(10.0f, db / 40.0f), w = 2.0f * (float)M_PI * fc / fs;
    const float c = cosf(w), sn = sinf(w), al = sn / 2.0f * sqrtf(2.0f);
    const float sa = 2.0f * sqrtf(A) * al;
    bq_set(f, A * ((A + 1) - (A - 1) * c + sa), 2 * A * ((A - 1) - (A + 1) * c),
           A * ((A + 1) - (A - 1) * c - sa), (A + 1) + (A - 1) * c + sa,
           -2 * ((A - 1) + (A + 1) * c), (A + 1) + (A - 1) * c - sa);
}

static inline float bq_run(biquad &f, int ch, float x) {
    const float y = f.b0 * x + f.z1[ch];
    f.z1[ch] = f.b1 * x - f.a1 * y + f.z2[ch];
    f.z2[ch] = f.b2 * x - f.a2 * y;
    return y;
}

// Freeverb's structure (parallel damped combs, then series allpasses), four
// combs a side, delays halved for 22kHz from its 44.1kHz tuning.
#define FX_COMBS 4
#define FX_ALLPS 2
#define FX_SPREAD 23                    // samples between the two sides' tunings
#define FX_WIDE_LEN 331                 // 15ms at 22kHz: the pseudo-stereo delay
static const int k_comb_len[FX_COMBS] = {558, 594, 639, 678};
static const int k_allp_len[FX_ALLPS] = {278, 220};

struct fx_line {
    float *buf;
    int    len, pos;
    float  store;
};

static fx_line s_comb[2][FX_COMBS];
static fx_line s_allp[2][FX_ALLPS];
static biquad  s_lp, s_shelf;
static bool    s_fx_ready = false;
static float  *s_wide;                  // FX_WIDE_LEN samples of the mono signal
static int     s_wide_pos;

static inline float comb_run(fx_line &c, float x, float fb, float damp) {
    const float y = c.buf[c.pos];
    c.store = y * (1.0f - damp) + c.store * damp;
    c.buf[c.pos] = x + c.store * fb;
    if (++c.pos >= c.len) {
        c.pos = 0;
    }
    return y;
}

static inline float allp_run(fx_line &a, float x) {
    const float b = a.buf[a.pos];
    a.buf[a.pos] = x + b * 0.5f;
    if (++a.pos >= a.len) {
        a.pos = 0;
    }
    return b - x;
}

static void fx_reset(void) {
    const float fs = (float)s_fx_rate;
    bq_lowpass(s_lp, fs, 6500.0f, 0.6f);
    bq_lowshelf(s_shelf, fs, 180.0f, 5.0f);
    for (int ch = 0; ch < 2; ch++) {
        for (int k = 0; k < FX_COMBS; k++) {
            memset(s_comb[ch][k].buf, 0, sizeof(float) * s_comb[ch][k].len);
            s_comb[ch][k].pos = 0;
            s_comb[ch][k].store = 0.0f;
        }
        for (int k = 0; k < FX_ALLPS; k++) {
            memset(s_allp[ch][k].buf, 0, sizeof(float) * s_allp[ch][k].len);
            s_allp[ch][k].pos = 0;
        }
    }
    memset(s_wide, 0, sizeof(float) * FX_WIDE_LEN);
    s_wide_pos = 0;
}

static bool fx_init(int rate) {
    s_fx_rate = rate;
    for (int ch = 0; ch < 2; ch++) {
        const int sp = ch ? FX_SPREAD : 0;
        for (int k = 0; k < FX_COMBS; k++) {
            s_comb[ch][k].len = k_comb_len[k] + sp;
            s_comb[ch][k].buf = (float *)heap_caps_malloc(sizeof(float) * s_comb[ch][k].len,
                                                          MALLOC_CAP_SPIRAM);
            if (!s_comb[ch][k].buf) {
                return false;
            }
        }
        for (int k = 0; k < FX_ALLPS; k++) {
            s_allp[ch][k].len = k_allp_len[k] + sp;
            s_allp[ch][k].buf = (float *)heap_caps_malloc(sizeof(float) * s_allp[ch][k].len,
                                                          MALLOC_CAP_SPIRAM);
            if (!s_allp[ch][k].buf) {
                return false;
            }
        }
    }
    s_wide = (float *)heap_caps_malloc(sizeof(float) * FX_WIDE_LEN, MALLOC_CAP_SPIRAM);
    if (!s_wide) {
        return false;
    }
    fx_reset();
    s_fx_ready = true;
    return true;
}

// A hard clip is a click every time a peak touches the top; the boost at the
// bottom and the reverb's tail make that common. Above 70% of full scale the
// curve bends over instead, smoothly, and never quite reaches it.
static inline int16_t fx_clip(float v) {
    float n = v * (1.0f / 32768.0f);
    const float a = fabsf(n);
    if (a > 0.7f) {
        const float e = a - 0.7f;
        n = copysignf(0.7f + 0.3f * e / (e + 0.3f), n);
    }
    return (int16_t)(n * 32767.0f);
}

// In place, on stereo s16 frames.
static void fx_process(int16_t *p, int frames, int mode) {
    // Headroom for the shelf's boost and the reverb's tail; the codec's own
    // volume is where loudness belongs.
    const float pre = 0.6f;
    for (int i = 0; i < frames; i++) {
        float l = bq_run(s_shelf, 0, bq_run(s_lp, 0, p[2 * i] * pre));
        float r = bq_run(s_shelf, 1, bq_run(s_lp, 1, p[2 * i + 1] * pre));
        if (mode == 2) {
            const float in = (l + r) * 0.015f;
            float wl = 0.0f, wr = 0.0f;
            for (int k = 0; k < FX_COMBS; k++) {
                wl += comb_run(s_comb[0][k], in, 0.83f, 0.35f);
                wr += comb_run(s_comb[1][k], in, 0.83f, 0.35f);
            }
            for (int k = 0; k < FX_ALLPS; k++) {
                wl = allp_run(s_allp[0][k], wl);
                wr = allp_run(s_allp[1][k], wr);
            }
            // Pseudo-stereo (Lauridsen): the signal 15ms late, added to one
            // side and taken from the other. Each frequency leans left or
            // right by a different amount, so a mono chip opens out - and the
            // two sides still add back up to the original.
            const float m = (l + r) * 0.5f;
            const float d = s_wide[s_wide_pos];
            s_wide[s_wide_pos] = m;
            if (++s_wide_pos >= FX_WIDE_LEN) {
                s_wide_pos = 0;
            }
            // The reverb wider than freeverb's 1.0 as well: each side gets a
            // little of the other's tail, inverted.
            const float w1 = 1.25f, w2 = -0.25f;
            l = l * 0.8f + d * 0.35f + (wl * w1 + wr * w2) * 1.3f;
            r = r * 0.8f - d * 0.35f + (wr * w1 + wl * w2) * 1.3f;
        }
        p[2 * i] = fx_clip(l);
        p[2 * i + 1] = fx_clip(r);
    }
}

extern "C" void audio_set_fx(int mode) {
    s_fx = (mode >= 0 && mode <= 2) ? mode : 0;
}

extern "C" int audio_get_fx(void) {
    return s_fx;
}

// ---- core 0: drain the ring into the codec --------------------------------
static void audio_task(void *arg) {
    (void)arg;
    // 4KB = 1024 stereo frames = ~46ms per codec write. On the S3 this was 1KB
    // because the ring behind it was small; here it just means fewer wakeups.
    static uint8_t rx[4096] __attribute__((aligned(4)));
    size_t carry = 0;  // bytes of a split stereo frame held back from last time
    int fx_was = 0;
    bool primed = false;
    for (;;) {
        // Keep a cushion. The emulator hands over a frame's worth of sound at
        // a time, and a frame does not always take the same time; passed on
        // the moment it arrived, the codec was fed just in time, and every
        // frame that ran a little long was a gap - a click. So wait until
        // there is PRIME_MS of sound in hand before starting, and again after
        // running dry, and the jitter is absorbed by what is waiting.
        if (!primed) {
            if (xStreamBufferBytesAvailable(s_ring) < s_prime_bytes) {
                vTaskDelay(pdMS_TO_TICKS(5));
                continue;
            }
            primed = true;
        }
        size_t n = xStreamBufferReceive(s_ring, rx + carry, sizeof(rx) - carry, pdMS_TO_TICKS(100));
        if (n > 0 && n + carry < sizeof(rx)) {
            primed = false;          // the ring is empty now: build the cushion again
        }
        if (n == 0) {
            g_audio_under++;
            primed = false;
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
            const int fx = s_fx;
            if (fx != fx_was) {
                if (s_fx_ready) {
                    fx_reset();          // no tail of the old setting into the new
                }
                fx_was = fx;
            }
            // The chips render a hair faster than the codec plays (about
            // 22,100 against 22,050 a second), so the cushion grows until the
            // ring is full and a whole block has to be refused - a click.
            // Past HIGH_MS, take two frames out of the block instead, each
            // folded into its neighbour: a 0.1% tightening nobody can hear.
            size_t out = play;
            if (play >= 64 && xStreamBufferBytesAvailable(s_ring) > s_high_bytes) {
                int16_t *f = (int16_t *)rx;
                int frames = (int)(play / 4);
                for (int cut = 0; cut < 2; cut++) {
                    const int j = frames * (cut + 1) / 3;
                    f[2 * j] = (int16_t)((f[2 * j] + f[2 * j + 2]) / 2);
                    f[2 * j + 1] = (int16_t)((f[2 * j + 1] + f[2 * j + 3]) / 2);
                    memmove(&f[2 * (j + 1)], &f[2 * (j + 2)], (size_t)(frames - j - 2) * 4);
                    frames--;
                }
                out = (size_t)frames * 4;
            }
            if (fx && s_fx_ready) {
                fx_process((int16_t *)rx, (int)(out / 4), fx);
            }
            esp_codec_dev_write(s_spk, rx, out);
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

    s_prime_bytes = (size_t)rate * 4 * PRIME_MS / 1000;
    s_high_bytes = (size_t)rate * 4 * HIGH_MS / 1000;
    if (!fx_init(rate)) {
        ets_printf("audio: no memory for the effects - they stay off\n");
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
    // All of the block or none of it. A send cut short at whatever byte the
    // ring had room for left the stream out of step with the 4-byte frames,
    // and everything after it played as noise - left and right, high and low
    // bytes all crossed - until another cut happened to put it back.
    if (xStreamBufferSpacesAvailable(s_ring) < bytes) {
        g_audio_drops += frames;
        return;
    }
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

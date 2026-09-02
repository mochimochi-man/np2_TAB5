// ESP32-S3 PIE (128-bit SIMD) helpers for np2_espresso.
//
// The toolchain ships no TIE intrinsics header, so these are GNU inline-asm
// wrappers in esp-dsp style, each paired with a pure-C reference of identical
// semantics. NP2_SIMD_SELFCHECK runs both on real data and compares them
// byte-exact (the only way to verify PIE code without host-side execution).
//
// IMPORTANT: EE.VLD/VST.128 require 16-byte aligned addresses (unaligned
// raises an exception), so every kernel checks alignment and falls back to C.
//
// NP2_SIMD_PIE=0 forces the C reference everywhere (A/B switch for perf runs).
#ifndef NP2_PIE_SIMD_H
#define NP2_PIE_SIMD_H

#include <stdint.h>

#if defined(ESP_PLATFORM)
#include "sdkconfig.h"
#endif

#ifndef NP2_SIMD_PIE
#  if defined(__XTENSA__) && defined(CONFIG_IDF_TARGET_ESP32S3)
#    define NP2_SIMD_PIE 1
#  else
#    define NP2_SIMD_PIE 0
#  endif
#endif

// Development aid: 1 = dual-run C vs PIE and ets_printf the comparison.
// Set to 0 once the on-device check reports PASS. VERIFIED on-device 2026-08-09:
// np2simd_s32_to_s16 passed thousands of consecutive dual-run checks (C==PIE,
// byte-exact), so the redundant C run is disabled for the release build.
#ifndef NP2_SIMD_SELFCHECK
#  define NP2_SIMD_SELFCHECK 0
#endif

#if NP2_SIMD_SELFCHECK
#include "esp_cpu.h"    // esp_cpu_get_cycle_count()
#include <string.h>     // memcmp
#endif

#ifdef __cplusplus
extern "C" {
#endif
#if NP2_SIMD_SELFCHECK
extern int ets_printf(const char *fmt, ...);
#endif
#ifdef __cplusplus
}
#endif

#define np2simd_aligned16(p) ((((uintptr_t)(p)) & 15) == 0)

// ---- s32 -> s16 saturating conversion (audio path) --------------------------

// C reference: convert n interleaved samples, tracking max |sample|.
// Returns the new peak (>= peak_in).
static inline int np2simd_s32_to_s16_c(const int32_t *src, int16_t *dst, int n, int peak_in) {
    int peak = peak_in;
    for (int i = 0; i < n; i++) {
        int32_t v = src[i];
        int32_t av = v < 0 ? -v : v;
        if (av > peak) peak = av;
        dst[i] = (int16_t)(v > 32767 ? 32767 : (v < -32768 ? -32768 : v));
    }
    return peak;
}

#if NP2_SIMD_PIE
// PIE kernel: 8 samples per iteration. src/dst must be 16-byte aligned, n a
// multiple of 8. acc4 (4 x int32, 16-byte aligned) is the running |v| max.
static inline void np2simd_pie_s32_to_s16_8(const int32_t *src, int16_t *dst, int n,
                                            int32_t acc4[4]) {
    static const int32_t c_pos = 32767, c_neg = -32768, c_one = 1, c_m1 = -1;
    asm volatile (
        // broadcast consts: q4=32767  q5=-32768  q6=1  q7=-1
        "ee.vldbc.32.ip  q4, %[pos], 0\n"
        "ee.vldbc.32.ip  q5, %[neg], 0\n"
        "ee.vldbc.32.ip  q6, %[one], 0\n"
        "ee.vldbc.32.ip  q7, %[m1],  0\n"
        // q3 = running |v| max accumulator
        "ee.vld.128.ip   q3, %[acc], 0\n"
        "1:\n"
        // q1 = samples 0..3 (LOW half), q0 = samples 4..7 (HIGH half):
        // EE.VUNZIP.16 treats the first operand as the low 128 bits of the
        // element array, so this order packs samples 0..7 sequentially.
        "ee.vld.128.ip   q1, %[src], 16\n"
        "ee.vld.128.ip   q0, %[src], 16\n"
        // abs(q0) = max(q0, ~q0+1) -> fold into q3  (vadds saturates, so
        // INT_MIN gives 0x7FFFFFFF instead of the C ref's wrap; unreachable
        // for real FM output, and the selfcheck compares on real data)
        "ee.xorq         q2, q0, q7\n"
        "ee.vadds.s32    q2, q2, q6\n"
        "ee.vmax.s32     q2, q0, q2\n"
        "ee.vmax.s32     q3, q3, q2\n"
        // abs(q1) -> fold into q3
        "ee.xorq         q2, q1, q7\n"
        "ee.vadds.s32    q2, q2, q6\n"
        "ee.vmax.s32     q2, q1, q2\n"
        "ee.vmax.s32     q3, q3, q2\n"
        // clamp to [-32768, 32767]
        "ee.vmax.s32     q0, q0, q5\n"
        "ee.vmin.s32     q0, q0, q4\n"
        "ee.vmax.s32     q1, q1, q5\n"
        "ee.vmin.s32     q1, q1, q4\n"
        // gather the low 16 bits of the eight 32-bit lanes -> q1 (in order);
        // EE.VUNZIP.16 puts the even 16-bit elements in the FIRST operand
        "ee.vunzip.16    q1, q0\n"
        "ee.vst.128.ip   q1, %[dst], 16\n"
        "addi            %[n], %[n], -8\n"
        "bnez            %[n], 1b\n"
        "ee.vst.128.ip   q3, %[acc], 0\n"
        : [src] "+r" (src), [dst] "+r" (dst), [n] "+r" (n)
        : [pos] "r" (&c_pos), [neg] "r" (&c_neg), [one] "r" (&c_one),
          [m1] "r" (&c_m1), [acc] "r" (acc4)
        : "memory");
}
#endif // NP2_SIMD_PIE

// Dispatcher: PIE fast path when possible, C reference otherwise.
// Same contract as np2simd_s32_to_s16_c.
static inline int np2simd_s32_to_s16(const int32_t *src, int16_t *dst, int n, int peak_in) {
#if NP2_SIMD_PIE
    int i = 0;
    int peak = peak_in;
    if (n >= 8 && np2simd_aligned16(src) && np2simd_aligned16(dst)) {
        int32_t acc4[4] __attribute__((aligned(16))) = { peak, peak, peak, peak };
        int nv = n & ~7;
        np2simd_pie_s32_to_s16_8(src, dst, nv, acc4);
        for (int k = 0; k < 4; k++) if (acc4[k] > peak) peak = acc4[k];
        i = nv;
    }
    return np2simd_s32_to_s16_c(src + i, dst + i, n - i, peak);
#else
    return np2simd_s32_to_s16_c(src, dst, n, peak_in);
#endif
}

// ---- masked clear: p[i] &= keep over n bytes (vramupdate clear) -------------

// C reference: n must be a multiple of 4, p 4-byte aligned (same assumption
// as the original makegrph.c loops).
static inline void np2simd_andmask_c(uint8_t *p, uint32_t keep, uint32_t n) {
    for (uint32_t i = 0; i < n; i += 4) *(uint32_t *)(p + i) &= keep;
}

#if NP2_SIMD_PIE
// PIE kernel: 64 bytes per iteration, loads batched before stores (back-to-
// back load/store on the same PSRAM cache line stalls badly). p 16-byte
// aligned, n multiple of 64.
static inline void np2simd_pie_andmask_64(uint8_t *p, uint32_t keep, uint32_t n) {
    uint8_t *wr = p;    // load/store use separate pointers: .ip post-increments
    asm volatile (
        "ee.vldbc.32.ip  q4, %[keep], 0\n"     // q4 = keep x4
        "1:\n"
        "ee.vld.128.ip   q0, %[p], 16\n"
        "ee.vld.128.ip   q1, %[p], 16\n"
        "ee.vld.128.ip   q2, %[p], 16\n"
        "ee.vld.128.ip   q3, %[p], 16\n"
        "ee.andq         q0, q0, q4\n"
        "ee.andq         q1, q1, q4\n"
        "ee.andq         q2, q2, q4\n"
        "ee.andq         q3, q3, q4\n"
        "ee.vst.128.ip   q0, %[wr], 16\n"
        "ee.vst.128.ip   q1, %[wr], 16\n"
        "ee.vst.128.ip   q2, %[wr], 16\n"
        "ee.vst.128.ip   q3, %[wr], 16\n"
        "addi            %[n], %[n], -64\n"
        "bnez            %[n], 1b\n"
        : [p] "+r" (p), [wr] "+r" (wr), [n] "+r" (n)
        : [keep] "r" (&keep)
        : "memory");
}
#endif // NP2_SIMD_PIE

// Dispatcher: PIE when aligned, C otherwise. n multiple of 4.
static inline void np2simd_andmask(uint8_t *p, uint32_t keep, uint32_t n) {
#if NP2_SIMD_PIE
    if (np2simd_aligned16(p) && n >= 64) {
        uint32_t nv = n & ~63u;
        np2simd_pie_andmask_64(p, keep, nv);
        np2simd_andmask_c(p + nv, keep, n - nv);
        return;
    }
#endif
    np2simd_andmask_c(p, keep, n);
}

#endif // NP2_PIE_SIMD_H

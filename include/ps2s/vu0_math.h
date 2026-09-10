/* VU0 macro-mode math experiment. C/C++ compatible; no legacy VU classes.
 *
 * Main render thread owns VU0 during each complete asm block. Do not call from
 * an interrupt handler, another VU0-using thread, or while a VU0 microprogram
 * runs. VU1 is independent. No register value survives as part of this API.
 * VF1-24 and VU MAC/status flags are scratch; ACC/Q/I/R/VI registers are unused.
 * Current GCC does not allocate VF registers: fixed $vfN names plus pointer
 * inputs and a memory clobber follow PS2SDK math3d/libvux's modern-GCC pattern.
 * The memory clobber describes CPU-visible reads/writes, NOT a VU ownership lock.
 *
 * Each block loads every input before storing results, so overlapping input
 * and output ranges are supported. LQC2/SQC2 buffers must be 16-byte aligned;
 * the native-row Vector3 output uses scalar stores and needs only 4 bytes.
 * Macro-mode VF RAW/WAW and result-transfer interlocks handle dependencies.
 * No DMA, cache flush, interrupt-mask change, VU1 program, or microcode call.
 *
 * Separate VMUL/VADD retains expression ordering but may differ numerically
 * from EE COP1. Bruno authorized this visible experiment on 2026-09-10.
 */
#ifndef PS2S_VU0_MATH_H
#define PS2S_VU0_MATH_H

/* Interleave independent points inside the existing batch, preserving every
 * point's operand/order tree. 0 selects the preceding serial batch schedule. */
#ifndef PS2S_VU0_BATCH_INTERLEAVE
#define PS2S_VU0_BATCH_INTERLEAVE 1
#endif
#if PS2S_VU0_BATCH_INTERLEAVE != 0 && PS2S_VU0_BATCH_INTERLEAVE != 1
#error "PS2S_VU0_BATCH_INTERLEAVE must be 0 or 1"
#endif

typedef float ps2s_vu0_mat4[16] __attribute__((aligned(16)));
typedef float ps2s_vu0_vec4[4] __attribute__((aligned(16)));

#if defined(_EE)

/* OUT = LHS * RHS, column-major storage. Raymath's native Matrix field order
 * can use this same storage operation: its existing MatrixMultiply expression
 * maps to these exact slots without a transpose. VF1-16 and MAC/status scratch.
 */
#define PS2S_VU0_MAT4_MUL_ALIGNED(OUT, LHS, RHS) \
    do { \
        __asm__ __volatile__( \
            "lqc2 $vf1, 0x00(%[lhs])\n\t" \
            "lqc2 $vf2, 0x10(%[lhs])\n\t" \
            "lqc2 $vf3, 0x20(%[lhs])\n\t" \
            "lqc2 $vf4, 0x30(%[lhs])\n\t" \
            "lqc2 $vf5, 0x00(%[rhs])\n\t" \
            "lqc2 $vf6, 0x10(%[rhs])\n\t" \
            "lqc2 $vf7, 0x20(%[rhs])\n\t" \
            "lqc2 $vf8, 0x30(%[rhs])\n\t" \
            "vmulx.xyzw $vf9, $vf1, $vf5\n\t" \
            "vmulx.xyzw $vf10, $vf1, $vf6\n\t" \
            "vmulx.xyzw $vf11, $vf1, $vf7\n\t" \
            "vmulx.xyzw $vf12, $vf1, $vf8\n\t" \
            "vmuly.xyzw $vf13, $vf2, $vf5\n\t" \
            "vmuly.xyzw $vf14, $vf2, $vf6\n\t" \
            "vmuly.xyzw $vf15, $vf2, $vf7\n\t" \
            "vmuly.xyzw $vf16, $vf2, $vf8\n\t" \
            "vadd.xyzw $vf9, $vf9, $vf13\n\t" \
            "vadd.xyzw $vf10, $vf10, $vf14\n\t" \
            "vadd.xyzw $vf11, $vf11, $vf15\n\t" \
            "vadd.xyzw $vf12, $vf12, $vf16\n\t" \
            "vmulz.xyzw $vf13, $vf3, $vf5\n\t" \
            "vmulz.xyzw $vf14, $vf3, $vf6\n\t" \
            "vmulz.xyzw $vf15, $vf3, $vf7\n\t" \
            "vmulz.xyzw $vf16, $vf3, $vf8\n\t" \
            "vadd.xyzw $vf9, $vf9, $vf13\n\t" \
            "vadd.xyzw $vf10, $vf10, $vf14\n\t" \
            "vadd.xyzw $vf11, $vf11, $vf15\n\t" \
            "vadd.xyzw $vf12, $vf12, $vf16\n\t" \
            "vmulw.xyzw $vf13, $vf4, $vf5\n\t" \
            "vmulw.xyzw $vf14, $vf4, $vf6\n\t" \
            "vmulw.xyzw $vf15, $vf4, $vf7\n\t" \
            "vmulw.xyzw $vf16, $vf4, $vf8\n\t" \
            "vadd.xyzw $vf9, $vf9, $vf13\n\t" \
            "vadd.xyzw $vf10, $vf10, $vf14\n\t" \
            "vadd.xyzw $vf11, $vf11, $vf15\n\t" \
            "vadd.xyzw $vf12, $vf12, $vf16\n\t" \
            "sqc2 $vf9, 0x00(%[out])\n\t" \
            "sqc2 $vf10, 0x10(%[out])\n\t" \
            "sqc2 $vf11, 0x20(%[out])\n\t" \
            "sqc2 $vf12, 0x30(%[out])\n\t" \
            : : [out] "r"(OUT), [lhs] "r"(LHS), [rhs] "r"(RHS) : "memory"); \
    } while (0)

/* Four-lane column-major matrix * vector; VF1-9 and MAC/status scratch. */
#define PS2S_VU0_MAT4_VEC4_ALIGNED(OUT, MAT, VEC) \
    do { \
        __asm__ __volatile__( \
            "lqc2 $vf1, 0x00(%[mat])\n\t" \
            "lqc2 $vf2, 0x10(%[mat])\n\t" \
            "lqc2 $vf3, 0x20(%[mat])\n\t" \
            "lqc2 $vf4, 0x30(%[mat])\n\t" \
            "lqc2 $vf5, 0x00(%[vec])\n\t" \
            "vmulx.xyzw $vf9, $vf1, $vf5\n\t" \
            "vmuly.xyzw $vf6, $vf2, $vf5\n\t" \
            "vmulz.xyzw $vf7, $vf3, $vf5\n\t" \
            "vmulw.xyzw $vf8, $vf4, $vf5\n\t" \
            "vadd.xyzw $vf9, $vf9, $vf6\n\t" \
            "vadd.xyzw $vf9, $vf9, $vf7\n\t" \
            "vadd.xyzw $vf9, $vf9, $vf8\n\t" \
            "sqc2 $vf9, 0x00(%[out])\n\t" \
            : : [out] "r"(OUT), [mat] "r"(MAT), [vec] "r"(VEC) : "memory"); \
    } while (0)

/* Column-major affine transform: three products, then add column 3 directly.
 * OUT has four floats; callers needing xyz copy only the first three. VEC.w
 * is loaded but never used. VF1-7/VF9 and MAC/status are scratch.
 */
#define PS2S_VU0_MAT4_VEC3_ALIGNED(OUT, MAT, VEC) \
    do { \
        __asm__ __volatile__( \
            "lqc2 $vf1, 0x00(%[mat])\n\t" \
            "lqc2 $vf2, 0x10(%[mat])\n\t" \
            "lqc2 $vf3, 0x20(%[mat])\n\t" \
            "lqc2 $vf4, 0x30(%[mat])\n\t" \
            "lqc2 $vf5, 0x00(%[vec])\n\t" \
            "vmulx.xyzw $vf9, $vf1, $vf5\n\t" \
            "vmuly.xyzw $vf6, $vf2, $vf5\n\t" \
            "vmulz.xyzw $vf7, $vf3, $vf5\n\t" \
            "vadd.xyzw $vf9, $vf9, $vf6\n\t" \
            "vadd.xyzw $vf9, $vf9, $vf7\n\t" \
            "vadd.xyzw $vf9, $vf9, $vf4\n\t" \
            "sqc2 $vf9, 0x00(%[out])\n\t" \
            : : [out] "r"(OUT), [mat] "r"(MAT), [vec] "r"(VEC) : "memory"); \
    } while (0)

#if PS2S_VU0_BATCH_INTERLEAVE
/* Four independent points at a time separate dependent sums with useful work.
 * Same twelve input loads, 24 VMUL, 24 VADD and eight output stores as below.
 * VF1-12 retain all inputs before any store; VF13-24 are private scratch.
 * Each point still sums ((col0*x + col1*y) + col2*z) + col3. No ACC/Q use.
 */
#define PS2S_VU0_MAT4_VEC3_BATCH8_ALIGNED(OUT, MAT, VEC) \
    do { \
        __asm__ __volatile__( \
            "lqc2 $vf1, 0x00(%[mat])\n\t" \
            "lqc2 $vf2, 0x10(%[mat])\n\t" \
            "lqc2 $vf3, 0x20(%[mat])\n\t" \
            "lqc2 $vf4, 0x30(%[mat])\n\t" \
            "lqc2 $vf5, 0x00(%[vec])\n\t" \
            "lqc2 $vf6, 0x10(%[vec])\n\t" \
            "lqc2 $vf7, 0x20(%[vec])\n\t" \
            "lqc2 $vf8, 0x30(%[vec])\n\t" \
            "lqc2 $vf9, 0x40(%[vec])\n\t" \
            "lqc2 $vf10, 0x50(%[vec])\n\t" \
            "lqc2 $vf11, 0x60(%[vec])\n\t" \
            "lqc2 $vf12, 0x70(%[vec])\n\t" \
            "vmulx.xyzw $vf13, $vf1, $vf5\n\t" \
            "vmuly.xyzw $vf14, $vf2, $vf5\n\t" \
            "vmulz.xyzw $vf15, $vf3, $vf5\n\t" \
            "vmulx.xyzw $vf16, $vf1, $vf6\n\t" \
            "vmuly.xyzw $vf17, $vf2, $vf6\n\t" \
            "vmulz.xyzw $vf18, $vf3, $vf6\n\t" \
            "vmulx.xyzw $vf19, $vf1, $vf7\n\t" \
            "vmuly.xyzw $vf20, $vf2, $vf7\n\t" \
            "vmulz.xyzw $vf21, $vf3, $vf7\n\t" \
            "vmulx.xyzw $vf22, $vf1, $vf8\n\t" \
            "vmuly.xyzw $vf23, $vf2, $vf8\n\t" \
            "vmulz.xyzw $vf24, $vf3, $vf8\n\t" \
            "vadd.xyzw $vf13, $vf13, $vf14\n\t" \
            "vadd.xyzw $vf16, $vf16, $vf17\n\t" \
            "vadd.xyzw $vf19, $vf19, $vf20\n\t" \
            "vadd.xyzw $vf22, $vf22, $vf23\n\t" \
            "vadd.xyzw $vf13, $vf13, $vf15\n\t" \
            "vadd.xyzw $vf16, $vf16, $vf18\n\t" \
            "vadd.xyzw $vf19, $vf19, $vf21\n\t" \
            "vadd.xyzw $vf22, $vf22, $vf24\n\t" \
            "vadd.xyzw $vf13, $vf13, $vf4\n\t" \
            "vadd.xyzw $vf16, $vf16, $vf4\n\t" \
            "vadd.xyzw $vf19, $vf19, $vf4\n\t" \
            "vadd.xyzw $vf22, $vf22, $vf4\n\t" \
            "sqc2 $vf13, 0x00(%[out])\n\t" \
            "sqc2 $vf16, 0x10(%[out])\n\t" \
            "sqc2 $vf19, 0x20(%[out])\n\t" \
            "sqc2 $vf22, 0x30(%[out])\n\t" \
            "vmulx.xyzw $vf13, $vf1, $vf9\n\t" \
            "vmuly.xyzw $vf14, $vf2, $vf9\n\t" \
            "vmulz.xyzw $vf15, $vf3, $vf9\n\t" \
            "vmulx.xyzw $vf16, $vf1, $vf10\n\t" \
            "vmuly.xyzw $vf17, $vf2, $vf10\n\t" \
            "vmulz.xyzw $vf18, $vf3, $vf10\n\t" \
            "vmulx.xyzw $vf19, $vf1, $vf11\n\t" \
            "vmuly.xyzw $vf20, $vf2, $vf11\n\t" \
            "vmulz.xyzw $vf21, $vf3, $vf11\n\t" \
            "vmulx.xyzw $vf22, $vf1, $vf12\n\t" \
            "vmuly.xyzw $vf23, $vf2, $vf12\n\t" \
            "vmulz.xyzw $vf24, $vf3, $vf12\n\t" \
            "vadd.xyzw $vf13, $vf13, $vf14\n\t" \
            "vadd.xyzw $vf16, $vf16, $vf17\n\t" \
            "vadd.xyzw $vf19, $vf19, $vf20\n\t" \
            "vadd.xyzw $vf22, $vf22, $vf23\n\t" \
            "vadd.xyzw $vf13, $vf13, $vf15\n\t" \
            "vadd.xyzw $vf16, $vf16, $vf18\n\t" \
            "vadd.xyzw $vf19, $vf19, $vf21\n\t" \
            "vadd.xyzw $vf22, $vf22, $vf24\n\t" \
            "vadd.xyzw $vf13, $vf13, $vf4\n\t" \
            "vadd.xyzw $vf16, $vf16, $vf4\n\t" \
            "vadd.xyzw $vf19, $vf19, $vf4\n\t" \
            "vadd.xyzw $vf22, $vf22, $vf4\n\t" \
            "sqc2 $vf13, 0x40(%[out])\n\t" \
            "sqc2 $vf16, 0x50(%[out])\n\t" \
            "sqc2 $vf19, 0x60(%[out])\n\t" \
            "sqc2 $vf22, 0x70(%[out])\n\t" \
            : : [out] "r"(OUT), [mat] "r"(MAT), [vec] "r"(VEC) : "memory"); \
    } while (0)
#else
/* Eight affine points share four matrix loads. MAT is 16 floats; VEC and OUT
 * are 32 floats (eight xyzw records). Input w is unused; all output lanes are
 * defined. All twelve input qwords load before the first store, including for
 * overlapping ranges. VF1-12 retain inputs; VF13-15 and MAC/status are scratch.
 */
#define PS2S_VU0_MAT4_VEC3_BATCH8_ALIGNED(OUT, MAT, VEC) \
    do { \
        __asm__ __volatile__( \
            "lqc2 $vf1, 0x00(%[mat])\n\t" \
            "lqc2 $vf2, 0x10(%[mat])\n\t" \
            "lqc2 $vf3, 0x20(%[mat])\n\t" \
            "lqc2 $vf4, 0x30(%[mat])\n\t" \
            "lqc2 $vf5, 0x00(%[vec])\n\t" \
            "lqc2 $vf6, 0x10(%[vec])\n\t" \
            "lqc2 $vf7, 0x20(%[vec])\n\t" \
            "lqc2 $vf8, 0x30(%[vec])\n\t" \
            "lqc2 $vf9, 0x40(%[vec])\n\t" \
            "lqc2 $vf10, 0x50(%[vec])\n\t" \
            "lqc2 $vf11, 0x60(%[vec])\n\t" \
            "lqc2 $vf12, 0x70(%[vec])\n\t" \
            "vmulx.xyzw $vf13, $vf1, $vf5\n\t" \
            "vmuly.xyzw $vf14, $vf2, $vf5\n\t" \
            "vmulz.xyzw $vf15, $vf3, $vf5\n\t" \
            "vadd.xyzw $vf13, $vf13, $vf14\n\t" \
            "vadd.xyzw $vf13, $vf13, $vf15\n\t" \
            "vadd.xyzw $vf13, $vf13, $vf4\n\t" \
            "sqc2 $vf13, 0x00(%[out])\n\t" \
            "vmulx.xyzw $vf13, $vf1, $vf6\n\t" \
            "vmuly.xyzw $vf14, $vf2, $vf6\n\t" \
            "vmulz.xyzw $vf15, $vf3, $vf6\n\t" \
            "vadd.xyzw $vf13, $vf13, $vf14\n\t" \
            "vadd.xyzw $vf13, $vf13, $vf15\n\t" \
            "vadd.xyzw $vf13, $vf13, $vf4\n\t" \
            "sqc2 $vf13, 0x10(%[out])\n\t" \
            "vmulx.xyzw $vf13, $vf1, $vf7\n\t" \
            "vmuly.xyzw $vf14, $vf2, $vf7\n\t" \
            "vmulz.xyzw $vf15, $vf3, $vf7\n\t" \
            "vadd.xyzw $vf13, $vf13, $vf14\n\t" \
            "vadd.xyzw $vf13, $vf13, $vf15\n\t" \
            "vadd.xyzw $vf13, $vf13, $vf4\n\t" \
            "sqc2 $vf13, 0x20(%[out])\n\t" \
            "vmulx.xyzw $vf13, $vf1, $vf8\n\t" \
            "vmuly.xyzw $vf14, $vf2, $vf8\n\t" \
            "vmulz.xyzw $vf15, $vf3, $vf8\n\t" \
            "vadd.xyzw $vf13, $vf13, $vf14\n\t" \
            "vadd.xyzw $vf13, $vf13, $vf15\n\t" \
            "vadd.xyzw $vf13, $vf13, $vf4\n\t" \
            "sqc2 $vf13, 0x30(%[out])\n\t" \
            "vmulx.xyzw $vf13, $vf1, $vf9\n\t" \
            "vmuly.xyzw $vf14, $vf2, $vf9\n\t" \
            "vmulz.xyzw $vf15, $vf3, $vf9\n\t" \
            "vadd.xyzw $vf13, $vf13, $vf14\n\t" \
            "vadd.xyzw $vf13, $vf13, $vf15\n\t" \
            "vadd.xyzw $vf13, $vf13, $vf4\n\t" \
            "sqc2 $vf13, 0x40(%[out])\n\t" \
            "vmulx.xyzw $vf13, $vf1, $vf10\n\t" \
            "vmuly.xyzw $vf14, $vf2, $vf10\n\t" \
            "vmulz.xyzw $vf15, $vf3, $vf10\n\t" \
            "vadd.xyzw $vf13, $vf13, $vf14\n\t" \
            "vadd.xyzw $vf13, $vf13, $vf15\n\t" \
            "vadd.xyzw $vf13, $vf13, $vf4\n\t" \
            "sqc2 $vf13, 0x50(%[out])\n\t" \
            "vmulx.xyzw $vf13, $vf1, $vf11\n\t" \
            "vmuly.xyzw $vf14, $vf2, $vf11\n\t" \
            "vmulz.xyzw $vf15, $vf3, $vf11\n\t" \
            "vadd.xyzw $vf13, $vf13, $vf14\n\t" \
            "vadd.xyzw $vf13, $vf13, $vf15\n\t" \
            "vadd.xyzw $vf13, $vf13, $vf4\n\t" \
            "sqc2 $vf13, 0x60(%[out])\n\t" \
            "vmulx.xyzw $vf13, $vf1, $vf12\n\t" \
            "vmuly.xyzw $vf14, $vf2, $vf12\n\t" \
            "vmulz.xyzw $vf15, $vf3, $vf12\n\t" \
            "vadd.xyzw $vf13, $vf13, $vf14\n\t" \
            "vadd.xyzw $vf13, $vf13, $vf15\n\t" \
            "vadd.xyzw $vf13, $vf13, $vf4\n\t" \
            "sqc2 $vf13, 0x70(%[out])\n\t" \
            : : [out] "r"(OUT), [mat] "r"(MAT), [vec] "r"(VEC) : "memory"); \
    } while (0)

#endif /* PS2S_VU0_BATCH_INTERLEAVE */

/* Four independent dot3 points, component-major storage: VEC holds four X,
 * four Y, then four Z values; OUT has the same shape. MAT holds three padded
 * basis rows. Preserve point * basis operand direction and (X + Y) + Z.
 * VF1-15 and MAC/status scratch; all six input qwords precede every store.
 * Three independent output axes hide dependencies without changing any sum.
 */
#define PS2S_VU0_DOT3_POINTS4_ALIGNED(OUT, MAT, VEC) \
    do { \
        __asm__ __volatile__( \
            "lqc2 $vf1, 0x00(%[mat])\n\t" \
            "lqc2 $vf2, 0x10(%[mat])\n\t" \
            "lqc2 $vf3, 0x20(%[mat])\n\t" \
            "lqc2 $vf4, 0x00(%[vec])\n\t" \
            "lqc2 $vf5, 0x10(%[vec])\n\t" \
            "lqc2 $vf6, 0x20(%[vec])\n\t" \
            "vmulx.xyzw $vf7, $vf4, $vf1\n\t" \
            "vmuly.xyzw $vf8, $vf5, $vf1\n\t" \
            "vmulz.xyzw $vf9, $vf6, $vf1\n\t" \
            "vmulx.xyzw $vf10, $vf4, $vf2\n\t" \
            "vmuly.xyzw $vf11, $vf5, $vf2\n\t" \
            "vmulz.xyzw $vf12, $vf6, $vf2\n\t" \
            "vmulx.xyzw $vf13, $vf4, $vf3\n\t" \
            "vmuly.xyzw $vf14, $vf5, $vf3\n\t" \
            "vmulz.xyzw $vf15, $vf6, $vf3\n\t" \
            "vadd.xyzw $vf7, $vf7, $vf8\n\t" \
            "vadd.xyzw $vf10, $vf10, $vf11\n\t" \
            "vadd.xyzw $vf13, $vf13, $vf14\n\t" \
            "vadd.xyzw $vf7, $vf7, $vf9\n\t" \
            "vadd.xyzw $vf10, $vf10, $vf12\n\t" \
            "vadd.xyzw $vf13, $vf13, $vf15\n\t" \
            "sqc2 $vf7, 0x00(%[out])\n\t" \
            "sqc2 $vf10, 0x10(%[out])\n\t" \
            "sqc2 $vf13, 0x20(%[out])\n\t" \
            : : [out] "r"(OUT), [mat] "r"(MAT), [vec] "r"(VEC) : "memory"); \
    } while (0)

/* Native row-major affine xyz: row.xyz * vec.xyz, then add row.w.
 * Only the first three rows are read. VEC's fourth float must exist, but its
 * value is unused. Return exactly 12 bytes; never SQC2 through a Vector3.
 * QMFC2 copies 128 bits to a compiler-owned GPR; SW uses its low x word only.
 * All GPR outputs are early-clobber, separate from the three pointer inputs.
 * VF1-7 and MAC/status scratch. Undefined scratch y/z/w never reach memory.
 */
#define PS2S_VU0_MAT4_VEC3_ROWS(OUT_XYZ, ROWS_ALIGNED, VEC4_ALIGNED) \
    do { \
        unsigned int ps2s_x_, ps2s_y_, ps2s_z_; \
        __asm__ __volatile__( \
            "lqc2 $vf1, 0x00(%[mat])\n\t" \
            "lqc2 $vf2, 0x10(%[mat])\n\t" \
            "lqc2 $vf3, 0x20(%[mat])\n\t" \
            "lqc2 $vf4, 0x00(%[vec])\n\t" \
            "vmul.xyz $vf5, $vf1, $vf4\n\t" \
            "vmul.xyz $vf6, $vf2, $vf4\n\t" \
            "vmul.xyz $vf7, $vf3, $vf4\n\t" \
            "vaddy.x $vf5, $vf5, $vf5\n\t" \
            "vaddy.x $vf6, $vf6, $vf6\n\t" \
            "vaddy.x $vf7, $vf7, $vf7\n\t" \
            "vaddz.x $vf5, $vf5, $vf5\n\t" \
            "vaddz.x $vf6, $vf6, $vf6\n\t" \
            "vaddz.x $vf7, $vf7, $vf7\n\t" \
            "vaddw.x $vf5, $vf5, $vf1\n\t" \
            "vaddw.x $vf6, $vf6, $vf2\n\t" \
            "vaddw.x $vf7, $vf7, $vf3\n\t" \
            "qmfc2 %[x], $vf5\n\t" \
            "qmfc2 %[y], $vf6\n\t" \
            "qmfc2 %[z], $vf7\n\t" \
            "sw %[x], 0x00(%[out])\n\t" \
            "sw %[y], 0x04(%[out])\n\t" \
            "sw %[z], 0x08(%[out])\n\t" \
            : [x] "=&r"(ps2s_x_), [y] "=&r"(ps2s_y_), [z] "=&r"(ps2s_z_) \
            : [out] "r"(OUT_XYZ), [mat] "r"(ROWS_ALIGNED), [vec] "r"(VEC4_ALIGNED) \
            : "memory"); \
    } while (0)

#else

/* Portable storage/ordering reference. Temporary outputs preserve overlap. */
#define PS2S_VU0_MAT4_MUL_ALIGNED(OUT, LHS, RHS) \
    do { \
        float *ps2s_out_ = (OUT); const float *ps2s_a_ = (LHS), *ps2s_b_ = (RHS); \
        float ps2s_r_[16]; \
        for (int ps2s_c_ = 0; ps2s_c_ < 4; ++ps2s_c_) \
            for (int ps2s_i_ = 0; ps2s_i_ < 4; ++ps2s_i_) \
                ps2s_r_[4*ps2s_c_ + ps2s_i_] = \
                    ((ps2s_a_[ps2s_i_]*ps2s_b_[4*ps2s_c_] + ps2s_a_[4+ps2s_i_]*ps2s_b_[4*ps2s_c_+1]) \
                    + ps2s_a_[8+ps2s_i_]*ps2s_b_[4*ps2s_c_+2]) + ps2s_a_[12+ps2s_i_]*ps2s_b_[4*ps2s_c_+3]; \
        for (int ps2s_i_ = 0; ps2s_i_ < 16; ++ps2s_i_) ps2s_out_[ps2s_i_] = ps2s_r_[ps2s_i_]; \
    } while (0)

#define PS2S_VU0_MAT4_VEC4_ALIGNED(OUT, MAT, VEC) \
    do { \
        float *ps2s_out_ = (OUT); const float *ps2s_a_ = (MAT), *ps2s_b_ = (VEC); \
        float ps2s_r_[4]; \
        for (int ps2s_i_ = 0; ps2s_i_ < 4; ++ps2s_i_) \
            ps2s_r_[ps2s_i_] = ((ps2s_a_[ps2s_i_]*ps2s_b_[0] + ps2s_a_[4+ps2s_i_]*ps2s_b_[1]) \
                + ps2s_a_[8+ps2s_i_]*ps2s_b_[2]) + ps2s_a_[12+ps2s_i_]*ps2s_b_[3]; \
        for (int ps2s_i_ = 0; ps2s_i_ < 4; ++ps2s_i_) ps2s_out_[ps2s_i_] = ps2s_r_[ps2s_i_]; \
    } while (0)

#define PS2S_VU0_MAT4_VEC3_ROWS(OUT_XYZ, ROWS_ALIGNED, VEC4_ALIGNED) \
    do { \
        float *ps2s_out_ = (OUT_XYZ); const float *ps2s_a_ = (ROWS_ALIGNED), *ps2s_b_ = (VEC4_ALIGNED); \
        float ps2s_r_[3]; \
        for (int ps2s_i_ = 0; ps2s_i_ < 3; ++ps2s_i_) \
            ps2s_r_[ps2s_i_] = ((ps2s_a_[4*ps2s_i_]*ps2s_b_[0] + ps2s_a_[4*ps2s_i_+1]*ps2s_b_[1]) \
                + ps2s_a_[4*ps2s_i_+2]*ps2s_b_[2]) + ps2s_a_[4*ps2s_i_+3]; \
        for (int ps2s_i_ = 0; ps2s_i_ < 3; ++ps2s_i_) ps2s_out_[ps2s_i_] = ps2s_r_[ps2s_i_]; \
    } while (0)

#define PS2S_VU0_MAT4_VEC3_ALIGNED(OUT, MAT, VEC) \
    do { \
        float *ps2s_out_ = (OUT); const float *ps2s_a_ = (MAT), *ps2s_b_ = (VEC); \
        float ps2s_r_[4]; \
        for (int ps2s_i_ = 0; ps2s_i_ < 4; ++ps2s_i_) \
            ps2s_r_[ps2s_i_] = ((ps2s_a_[ps2s_i_]*ps2s_b_[0] + ps2s_a_[4+ps2s_i_]*ps2s_b_[1]) \
                + ps2s_a_[8+ps2s_i_]*ps2s_b_[2]) + ps2s_a_[12+ps2s_i_]; \
        for (int ps2s_i_ = 0; ps2s_i_ < 4; ++ps2s_i_) ps2s_out_[ps2s_i_] = ps2s_r_[ps2s_i_]; \
    } while (0)

#define PS2S_VU0_MAT4_VEC3_BATCH8_ALIGNED(OUT, MAT, VEC) \
    do { \
        float *ps2s_out_ = (OUT); const float *ps2s_a_ = (MAT), *ps2s_b_ = (VEC); \
        float ps2s_r_[32]; \
        for (int ps2s_v_ = 0; ps2s_v_ < 8; ++ps2s_v_) \
            for (int ps2s_i_ = 0; ps2s_i_ < 4; ++ps2s_i_) \
                ps2s_r_[4*ps2s_v_ + ps2s_i_] = \
                    ((ps2s_a_[ps2s_i_]*ps2s_b_[4*ps2s_v_] + ps2s_a_[4+ps2s_i_]*ps2s_b_[4*ps2s_v_+1]) \
                    + ps2s_a_[8+ps2s_i_]*ps2s_b_[4*ps2s_v_+2]) + ps2s_a_[12+ps2s_i_]; \
        for (int ps2s_i_ = 0; ps2s_i_ < 32; ++ps2s_i_) ps2s_out_[ps2s_i_] = ps2s_r_[ps2s_i_]; \
    } while (0)

#define PS2S_VU0_DOT3_POINTS4_ALIGNED(OUT, MAT, VEC) \
    do { \
        float *ps2s_out_ = (OUT); const float *ps2s_a_ = (MAT), *ps2s_b_ = (VEC); \
        float ps2s_r_[12]; \
        for (int ps2s_i_ = 0; ps2s_i_ < 3; ++ps2s_i_) \
            for (int ps2s_v_ = 0; ps2s_v_ < 4; ++ps2s_v_) \
                ps2s_r_[4*ps2s_i_ + ps2s_v_] = \
                    (ps2s_b_[ps2s_v_]*ps2s_a_[4*ps2s_i_] + ps2s_b_[4+ps2s_v_]*ps2s_a_[4*ps2s_i_+1]) \
                    + ps2s_b_[8+ps2s_v_]*ps2s_a_[4*ps2s_i_+2]; \
        for (int ps2s_i_ = 0; ps2s_i_ < 12; ++ps2s_i_) ps2s_out_[ps2s_i_] = ps2s_r_[ps2s_i_]; \
    } while (0)

#endif

static inline void ps2s_vu0_dot3_points4_aligned(float out[12], const float mat[12], const float vec[12])
{
    PS2S_VU0_DOT3_POINTS4_ALIGNED(out, mat, vec);
}

static inline void ps2s_vu0_mat4_mul_aligned(float out[16], const float lhs[16], const float rhs[16])
{
    PS2S_VU0_MAT4_MUL_ALIGNED(out, lhs, rhs);
}

static inline void ps2s_vu0_mat4_vec4_aligned(float out[4], const float mat[16], const float vec[4])
{
    PS2S_VU0_MAT4_VEC4_ALIGNED(out, mat, vec);
}

static inline void ps2s_vu0_mat4_vec3_rows(float out[3], const float rows[16], const float vec[4])
{
    PS2S_VU0_MAT4_VEC3_ROWS(out, rows, vec);
}

static inline void ps2s_vu0_mat4_vec3_aligned(float out[4], const float mat[16], const float vec[4])
{
    PS2S_VU0_MAT4_VEC3_ALIGNED(out, mat, vec);
}

static inline void ps2s_vu0_mat4_vec3_batch8_aligned(float out[32], const float mat[16], const float vec[32])
{
    PS2S_VU0_MAT4_VEC3_BATCH8_ALIGNED(out, mat, vec);
}

#endif // PS2S_VU0_MATH_H

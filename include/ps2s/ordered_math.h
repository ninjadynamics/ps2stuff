/* Ordered scalar EE math shared by C and C++ callers.
 *
 * These helpers do not enable the legacy NO_ASM / NO_VU0_VECTORS alternatives.
 * Callers select them with independent experiment gates. All products retain
 * their operand order and all sums retain their left association. No ACC,
 * Q, I, SA, VU register, fixed FPR or memory access is hidden in the asm.
 *
 * Macros support raymath's external-inline C functions without referring to
 * static functions. OUT arguments must be float lvalues; inputs are evaluated
 * once. Paired outputs must be distinct. Ordinary scalar float alignment is
 * sufficient. No fast-math/reassociation flags should be used for this code.
 */
#ifndef PS2S_ORDERED_MATH_H
#define PS2S_ORDERED_MATH_H

#if defined(_EE)

/* Four independent multiplies before the dependent left-associated adds.
 * Early clobbers keep every still-live input distinct from the four outputs.
 */
#define PS2S_ORDERED_DOT4_COP1(OUT, A0, B0, A1, B1, A2, B2, A3, B3) \
    do { \
        float ps2s_p1_, ps2s_p2_, ps2s_p3_; \
        __asm__( \
            "mul.s %[sum], %[a0], %[b0]\n\t" \
            "mul.s %[p1], %[a1], %[b1]\n\t" \
            "mul.s %[p2], %[a2], %[b2]\n\t" \
            "mul.s %[p3], %[a3], %[b3]\n\t" \
            "add.s %[sum], %[sum], %[p1]\n\t" \
            "add.s %[sum], %[sum], %[p2]\n\t" \
            "add.s %[sum], %[sum], %[p3]\n\t" \
            : [sum] "=&f"(OUT), [p1] "=&f"(ps2s_p1_), \
              [p2] "=&f"(ps2s_p2_), [p3] "=&f"(ps2s_p3_) \
            : [a0] "f"(A0), [b0] "f"(B0), [a1] "f"(A1), [b1] "f"(B1), \
              [a2] "f"(A2), [b2] "f"(B2), [a3] "f"(A3), [b3] "f"(B3)); \
    } while (0)

/* The translation is an addition, not an extra multiplication by 1. */
#define PS2S_ORDERED_DOT3_ADD_COP1(OUT, A0, B0, A1, B1, A2, B2, ADDEND) \
    do { \
        float ps2s_p1_, ps2s_p2_; \
        __asm__( \
            "mul.s %[sum], %[a0], %[b0]\n\t" \
            "mul.s %[p1], %[a1], %[b1]\n\t" \
            "mul.s %[p2], %[a2], %[b2]\n\t" \
            "add.s %[sum], %[sum], %[p1]\n\t" \
            "add.s %[sum], %[sum], %[p2]\n\t" \
            "add.s %[sum], %[sum], %[addend]\n\t" \
            : [sum] "=&f"(OUT), [p1] "=&f"(ps2s_p1_), [p2] "=&f"(ps2s_p2_) \
            : [a0] "f"(A0), [b0] "f"(B0), [a1] "f"(A1), [b1] "f"(B1), \
              [a2] "f"(A2), [b2] "f"(B2), [addend] "f"(ADDEND)); \
    } while (0)

/* Two matrix rows sharing one RHS column. Interleave independent lanes while
 * preserving each row's products and sum order. 12 input + 8 output FPRs;
 * four rows in one block would exceed the EE's 32 scalar FPRs.
 */
#define PS2S_ORDERED_DOT4_PAIR_COP1(OUT0, OUT1, A0, A1, A2, A3, C0, C1, C2, C3, B0, B1, B2, B3) \
    do { \
        float ps2s_ap1_, ps2s_ap2_, ps2s_ap3_, ps2s_cp1_, ps2s_cp2_, ps2s_cp3_; \
        __asm__( \
            "mul.s %[s0], %[a0], %[b0]\n\t" \
            "mul.s %[s1], %[c0], %[b0]\n\t" \
            "mul.s %[ap1], %[a1], %[b1]\n\t" \
            "mul.s %[cp1], %[c1], %[b1]\n\t" \
            "mul.s %[ap2], %[a2], %[b2]\n\t" \
            "mul.s %[cp2], %[c2], %[b2]\n\t" \
            "mul.s %[ap3], %[a3], %[b3]\n\t" \
            "mul.s %[cp3], %[c3], %[b3]\n\t" \
            "add.s %[s0], %[s0], %[ap1]\n\t" \
            "add.s %[s1], %[s1], %[cp1]\n\t" \
            "add.s %[s0], %[s0], %[ap2]\n\t" \
            "add.s %[s1], %[s1], %[cp2]\n\t" \
            "add.s %[s0], %[s0], %[ap3]\n\t" \
            "add.s %[s1], %[s1], %[cp3]\n\t" \
            : [s0] "=&f"(OUT0), [s1] "=&f"(OUT1), \
              [ap1] "=&f"(ps2s_ap1_), [ap2] "=&f"(ps2s_ap2_), [ap3] "=&f"(ps2s_ap3_), \
              [cp1] "=&f"(ps2s_cp1_), [cp2] "=&f"(ps2s_cp2_), [cp3] "=&f"(ps2s_cp3_) \
            : [a0] "f"(A0), [a1] "f"(A1), [a2] "f"(A2), [a3] "f"(A3), \
              [c0] "f"(C0), [c1] "f"(C1), [c2] "f"(C2), [c3] "f"(C3), \
              [b0] "f"(B0), [b1] "f"(B1), [b2] "f"(B2), [b3] "f"(B3)); \
    } while (0)

#else

#define PS2S_ORDERED_DOT4_COP1(OUT, A0, B0, A1, B1, A2, B2, A3, B3) \
    do { (OUT) = (((A0) * (B0) + (A1) * (B1)) + (A2) * (B2)) + (A3) * (B3); } while (0)
#define PS2S_ORDERED_DOT3_ADD_COP1(OUT, A0, B0, A1, B1, A2, B2, ADDEND) \
    do { (OUT) = (((A0) * (B0) + (A1) * (B1)) + (A2) * (B2)) + (ADDEND); } while (0)
#define PS2S_ORDERED_DOT4_PAIR_COP1(OUT0, OUT1, A0, A1, A2, A3, C0, C1, C2, C3, B0, B1, B2, B3) \
    do { \
        const float ps2s_b0_ = (B0), ps2s_b1_ = (B1), ps2s_b2_ = (B2), ps2s_b3_ = (B3); \
        const float ps2s_s0_ = (((A0) * ps2s_b0_ + (A1) * ps2s_b1_) + (A2) * ps2s_b2_) + (A3) * ps2s_b3_; \
        const float ps2s_s1_ = (((C0) * ps2s_b0_ + (C1) * ps2s_b1_) + (C2) * ps2s_b2_) + (C3) * ps2s_b3_; \
        (OUT0) = ps2s_s0_; (OUT1) = ps2s_s1_; \
    } while (0)

#endif

static inline float ps2s_ordered_dot4_cop1(float a0, float b0, float a1, float b1,
    float a2, float b2, float a3, float b3)
{
    float result;
    PS2S_ORDERED_DOT4_COP1(result, a0, b0, a1, b1, a2, b2, a3, b3);
    return result;
}

static inline float ps2s_ordered_dot3_add_cop1(float a0, float b0, float a1, float b1,
    float a2, float b2, float addend)
{
    float result;
    PS2S_ORDERED_DOT3_ADD_COP1(result, a0, b0, a1, b1, a2, b2, addend);
    return result;
}

#endif // PS2S_ORDERED_MATH_H

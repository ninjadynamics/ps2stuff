/*	  Copyright (C) 2000,2001,2002  Sony Computer Entertainment America

       	  This file is subject to the terms and conditions of the GNU Lesser
	  General Public License Version 2.1. See the file "COPYING" in the
	  main directory of this archive for more details.                             */

#ifndef ps2s_cpu_matrix_h
#define ps2s_cpu_matrix_h

#include "ps2s/cpu_vector.h"
#include "ps2s/ordered_math.h"
#include "ps2s/vu0_math.h"

// Independent matrix experiments. Keep NO_ASM / NO_VU0_VECTORS defined: these
// kernels do not import the old patched-GCC vector classes or use either ACC.
// VU0 takes precedence, then EE_COP1, then SCALAR_KERNEL. All three off
// restores the original row-vector/dot implementation below. The VU0
// experiment may differ numerically; it was explicitly authorized as such.
#ifndef PS2S_MATRIX_VU0
#define PS2S_MATRIX_VU0 1
#endif
#ifndef PS2S_MATRIX_SCALAR_KERNEL
#define PS2S_MATRIX_SCALAR_KERNEL 1
#endif
#ifndef PS2S_MATRIX_EE_COP1
#define PS2S_MATRIX_EE_COP1 1
#endif
#if (PS2S_MATRIX_VU0 != 0) && (PS2S_MATRIX_VU0 != 1)
#error PS2S_MATRIX_VU0 must be 0 or 1
#endif
#if (PS2S_MATRIX_SCALAR_KERNEL != 0) && (PS2S_MATRIX_SCALAR_KERNEL != 1)
#error PS2S_MATRIX_SCALAR_KERNEL must be 0 or 1
#endif
#if (PS2S_MATRIX_EE_COP1 != 0) && (PS2S_MATRIX_EE_COP1 != 1)
#error PS2S_MATRIX_EE_COP1 must be 0 or 1
#endif

class cpu_mat_44 {
    cpu_vec_4 col0, col1, col2, col3;

public:
    inline cpu_mat_44() {}
    inline cpu_mat_44(cpu_vec_4 c0, cpu_vec_4 c1, cpu_vec_4 c2, cpu_vec_4 c3)
    {
        col0 = c0;
        col1 = c1;
        col2 = c2;
        col3 = c3;
    }

    inline void
    set_identity()
    {
        col0.set(1, 0, 0, 0);
        col1.set(0, 1, 0, 0);
        col2.set(0, 0, 1, 0);
        col3.set(0, 0, 0, 1);
    }

    inline cpu_vec_4 get_col0() const { return col0; }
    inline cpu_vec_4 get_col1() const { return col1; }
    inline cpu_vec_4 get_col2() const { return col2; }
    inline cpu_vec_4 get_col3() const { return col3; }

    inline void set_col0(const cpu_vec_xyzw& col) { col0 = col; }
    inline void set_col1(const cpu_vec_xyzw& col) { col1 = col; }
    inline void set_col2(const cpu_vec_xyzw& col) { col2 = col; }
    inline void set_col3(const cpu_vec_xyzw& col) { col3 = col; }

    inline cpu_vec_4
    operator*(const cpu_vec_4& rhs) const;

    inline cpu_mat_44
    operator*(const cpu_mat_44& rhs) const;

    inline void
    set_scale(const cpu_vec_3& scale)
    {
        set_identity();
        col0[0] = scale(0);
        col1[1] = scale(1);
        col2[2] = scale(2);
        col3[3] = 1.0f;
    }

    inline void
    set_translate(const cpu_vec_3& offsets)
    {
        set_identity();
        col3 = offsets;
    }

    void set_rotate(float angle, cpu_vec_xyz axis);

    void print() const
    {
        col0.print();
        col1.print();
        col2.print();
        col3.print();
    }

    cpu_mat_44 transpose() const;
};

inline cpu_vec_4
    cpu_mat_44::operator*(const cpu_vec_4& rhs) const
{
    cpu_vec_4 result;

#if PS2S_MATRIX_VU0 && defined(_EE)
    // The public class remains ordinarily aligned. Only these typed local
    // arrays reach LQC2/SQC2, without aliasing a struct as a float array.
    const ps2s_vu0_mat4 matrix = {
        col0.x, col0.y, col0.z, col0.w, col1.x, col1.y, col1.z, col1.w,
        col2.x, col2.y, col2.z, col2.w, col3.x, col3.y, col3.z, col3.w
    };
    const ps2s_vu0_vec4 vector = { rhs.x, rhs.y, rhs.z, rhs.w };
    ps2s_vu0_vec4 transformed;
    PS2S_VU0_MAT4_VEC4_ALIGNED(transformed, matrix, vector);
    result = cpu_vec_4(transformed[0], transformed[1], transformed[2], transformed[3]);
#elif PS2S_MATRIX_EE_COP1 && defined(_EE)
    PS2S_ORDERED_DOT4_PAIR_COP1(result.x, result.y,
        col0.x, col1.x, col2.x, col3.x, col0.y, col1.y, col2.y, col3.y,
        rhs.x, rhs.y, rhs.z, rhs.w);
    PS2S_ORDERED_DOT4_PAIR_COP1(result.z, result.w,
        col0.z, col1.z, col2.z, col3.z, col0.w, col1.w, col2.w, col3.w,
        rhs.x, rhs.y, rhs.z, rhs.w);
#elif PS2S_MATRIX_SCALAR_KERNEL
    // Direct components avoid temporary row objects and checked accessors.
    // Preserve every term, including structural zeros, and the original
    // ((p0 + p1) + p2) + p3 association. Do not enable fast-math for this path.
    result.x = ((col0.x * rhs.x + col1.x * rhs.y) + col2.x * rhs.z) + col3.x * rhs.w;
    result.y = ((col0.y * rhs.x + col1.y * rhs.y) + col2.y * rhs.z) + col3.y * rhs.w;
    result.z = ((col0.z * rhs.x + col1.z * rhs.y) + col2.z * rhs.z) + col3.z * rhs.w;
    result.w = ((col0.w * rhs.x + col1.w * rhs.y) + col2.w * rhs.z) + col3.w * rhs.w;
#else
    cpu_vec_4 row0(col0(0), col1(0), col2(0), col3(0));
    result[0] = row0.dot(rhs);

    cpu_vec_4 row1(col0(1), col1(1), col2(1), col3(1));
    result[1] = row1.dot(rhs);

    cpu_vec_4 row2(col0(2), col1(2), col2(2), col3(2));
    result[2] = row2.dot(rhs);

    cpu_vec_4 row3(col0(3), col1(3), col2(3), col3(3));
    result[3] = row3.dot(rhs);
#endif

    return result;
}

inline cpu_mat_44
    cpu_mat_44::operator*(const cpu_mat_44& rhs) const
{
    cpu_mat_44 result;

#if PS2S_MATRIX_VU0 && defined(_EE)
    // Stage each operand once for the whole product. All input VFs are loaded
    // before output stores, and result remains separate from either operand.
    const ps2s_vu0_mat4 left = {
        col0.x, col0.y, col0.z, col0.w, col1.x, col1.y, col1.z, col1.w,
        col2.x, col2.y, col2.z, col2.w, col3.x, col3.y, col3.z, col3.w
    };
    const ps2s_vu0_mat4 right = {
        rhs.col0.x, rhs.col0.y, rhs.col0.z, rhs.col0.w,
        rhs.col1.x, rhs.col1.y, rhs.col1.z, rhs.col1.w,
        rhs.col2.x, rhs.col2.y, rhs.col2.z, rhs.col2.w,
        rhs.col3.x, rhs.col3.y, rhs.col3.z, rhs.col3.w
    };
    ps2s_vu0_mat4 product;
    PS2S_VU0_MAT4_MUL_ALIGNED(product, left, right);
    result.col0 = cpu_vec_4(product[0], product[1], product[2], product[3]);
    result.col1 = cpu_vec_4(product[4], product[5], product[6], product[7]);
    result.col2 = cpu_vec_4(product[8], product[9], product[10], product[11]);
    result.col3 = cpu_vec_4(product[12], product[13], product[14], product[15]);
#elif PS2S_MATRIX_SCALAR_KERNEL || (PS2S_MATRIX_EE_COP1 && defined(_EE))
    // Read the original columns directly. The result is a separate object,
    // so A = A * B, A = B * A and A = A * A retain their alias semantics.
    result.col0 = *this * rhs.col0;
    result.col1 = *this * rhs.col1;
    result.col2 = *this * rhs.col2;
    result.col3 = *this * rhs.col3;
#else
    result.col0 = *this * rhs.get_col0();
    result.col1 = *this * rhs.get_col1();
    result.col2 = *this * rhs.get_col2();
    result.col3 = *this * rhs.get_col3();
#endif

    return result;
}

#endif // ps2s_cpu_matrix_h

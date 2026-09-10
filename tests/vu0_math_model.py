#!/usr/bin/env python3
"""Source-driven VU0 lane/transfer tests; no EE compiler or VU emulation.

Parse the actual macro assembly, trace each VF lane, and compare its output
expression trees with scalar matrix definitions. Also exercise overlapping
aligned buffers and the exactly-12-byte native Vector3 output. Numerical
checks use host float32 and do not establish EE/VU rounding equivalence.
"""
import ast
import math
from pathlib import Path
import random
import re
import struct

ROOT = Path(__file__).resolve().parents[1]
SOURCE = (ROOT / 'include/ps2s/vu0_math.h').read_text(encoding='utf-8')
CPU = (ROOT / 'include/ps2s/cpu_matrix.h').read_text(encoding='utf-8')
LANES = 'xyzw'
NAMES = ('PS2S_VU0_MAT4_MUL_ALIGNED', 'PS2S_VU0_MAT4_VEC4_ALIGNED',
         'PS2S_VU0_MAT4_VEC3_ROWS', 'PS2S_VU0_MAT4_VEC3_ALIGNED',
         'PS2S_VU0_MAT4_VEC3_BATCH8_ALIGNED', 'PS2S_VU0_DOT3_POINTS4_ALIGNED')


def instructions(name, variant=0):
    blocks = list(re.finditer(r'#define ' + name + r'\([^\n]+\) \\\n(.*?while \(0\))', SOURCE, re.S))
    block = blocks[variant][1]
    quoted = re.findall(r'"([^"\n]*)"', block)
    ops = [ast.literal_eval('"' + value + '"').strip() for value in quoted if r'\n\t' in value]
    assert ops
    assert block.count('__asm__ __volatile__') == 1
    assert ': "memory"' in block
    assert not any(token in ' '.join(ops).lower() for token in ('$acc', '$q', '$i', 'vcall', 'vmadd', 'vmula', 'di ', 'ei '))
    if name.endswith('ROWS'):
        assert set(re.findall(r'\[(\w+)\] "=&r"', block)) == {'x', 'y', 'z'}
    return ops


PROGRAMS = {name: instructions(name) for name in NAMES}
PROGRAM_CASES = list(PROGRAMS.items()) + [(NAMES[4], instructions(NAMES[4], 1))]
assert '#define PS2S_VU0_BATCH_INTERLEAVE 1' in SOURCE
assert '#if PS2S_VU0_BATCH_INTERLEAVE != 0 && PS2S_VU0_BATCH_INTERLEAVE != 1' in SOURCE
assert '#if PS2S_VU0_BATCH_INTERLEAVE' in SOURCE


def f32(value):
    try:
        return struct.unpack('<f', struct.pack('<f', value))[0]
    except OverflowError:
        return math.copysign(math.inf, value)


def bits(values):
    return b''.join(struct.pack('<f', value) for value in values)


def arithmetic(op, left, right, symbolic):
    assert left is not None and right is not None, 'uninitialized lane read'
    if symbolic:
        return (op, left, right)
    return f32(left * right if op == 'mul' else left + right)


def execute(ops, memory, pointers, symbolic):
    vf = {i: [None] * 4 for i in range(1, 32)}
    gpr, reads, writes = {}, [], []
    first_store = None
    for step, op in enumerate(ops):
        match = re.fullmatch(r'lqc2 \$vf(\d+), (0x[0-9a-f]+)\(%\[(\w+)\]\)', op)
        if match:
            register, offset, pointer = int(match[1]), int(match[2], 16), match[3]
            address = pointers[pointer] + offset
            assert address % 16 == 0
            assert first_store is None, 'input load after output store breaks overlap'
            vf[register] = [memory[address + i * 4] for i in range(4)]
            reads.extend(address + i * 4 for i in range(4))
            continue
        match = re.fullmatch(r'v(mul|add)([xyzw]?)\.([xyzw]+) \$vf(\d+), \$vf(\d+), \$vf(\d+)([xyzw]?)', op)
        if match:
            operation, broadcast, mask = match[1], match[2], match[3]
            destination, left, right = map(int, match.group(4, 5, 6))
            assert not match[7] or match[7] == broadcast
            before = list(vf[destination])
            for lane in mask:
                index = LANES.index(lane)
                rindex = LANES.index(broadcast) if broadcast else index
                before[index] = arithmetic(operation, vf[left][index], vf[right][rindex], symbolic)
            vf[destination] = before
            continue
        match = re.fullmatch(r'qmfc2 %\[(\w+)\], \$vf(\d+)', op)
        if match:
            gpr[match[1]] = list(vf[int(match[2])])
            continue
        match = re.fullmatch(r'sqc2 \$vf(\d+), (0x[0-9a-f]+)\(%\[(\w+)\]\)', op)
        if match:
            register, offset, pointer = int(match[1]), int(match[2], 16), match[3]
            address = pointers[pointer] + offset
            assert address % 16 == 0
            values = vf[register]
        else:
            match = re.fullmatch(r'sw %\[(\w+)\], (0x[0-9a-f]+)\(%\[(\w+)\]\)', op)
            assert match, op
            address = pointers[match[3]] + int(match[2], 16)
            assert address % 4 == 0
            values = [gpr[match[1]][0]]
        first_store = step if first_store is None else first_store
        for i, value in enumerate(values):
            assert value is not None, 'uninitialized lane escaped to output'
            memory[address + 4 * i] = value
            writes.append(address + 4 * i)
    return reads, writes


def reference(name, first, second, symbolic):
    out = []
    if name == NAMES[5]:
        for row in range(3):
            for point in range(4):
                products = [arithmetic('mul', second[4 * axis + point], first[4 * row + axis], symbolic)
                            for axis in range(3)]
                out.append(arithmetic('add', arithmetic('add', products[0], products[1], symbolic), products[2], symbolic))
        return out
    width = 4 if name == NAMES[0] else 8 if name == NAMES[4] else 1
    rows = 3 if name == NAMES[2] else 4
    for column in range(width):
        for row in range(rows):
            native = name == NAMES[2]
            affine = name in (NAMES[2], NAMES[3], NAMES[4])
            products = [arithmetic('mul', first[4 * row + k if native else 4 * k + row],
                                   second[k if width == 1 else 4 * column + k], symbolic)
                        for k in range(3 if affine else 4)]
            value = arithmetic('add', products[0], products[1], symbolic)
            value = arithmetic('add', value, products[2], symbolic)
            last = first[4 * row + 3 if native else 12 + row] if affine else products[3]
            value = arithmetic('add', value, last, symbolic)
            out.append(value)
    return out


# Symbolic tracing proves exact lane/operand order, not arithmetic-unit parity.
for name, ops in PROGRAM_CASES:
    first = ['a' + str(i) for i in range(16)]
    second = ['b' + str(i) for i in range(16 if name == NAMES[0] else 32 if name == NAMES[4] else 12 if name == NAMES[5] else 4)]
    memory = {0x100 + 4 * i: value for i, value in enumerate(first)}
    memory.update({0x200 + 4 * i: value for i, value in enumerate(second)})
    pointers = {'lhs': 0x100, 'rhs': 0x200, 'mat': 0x100, 'vec': 0x200, 'out': 0x304 if name == NAMES[2] else 0x300}
    reads, writes = execute(ops, memory, pointers, True)
    expected = reference(name, first, second, True)
    assert writes == [pointers['out'] + 4 * i for i in range(len(expected))]
    assert [memory[address] for address in writes] == expected
    if name == NAMES[2]:
        assert max(address for address in reads if address < 0x200) == 0x12c
        assert len(writes) == 3 and writes[-1] == pointers['out'] + 8
    if name == NAMES[5]:
        assert reads == list(range(0x100, 0x130, 4)) + list(range(0x200, 0x230, 4))
        assert len(ops) == 24 and len(writes) == 12

# Check the real public-type -> aligned-array bridge and reconstruction.
assert 'typedef float ps2s_vu0_mat4[16] __attribute__((aligned(16)));' in SOURCE
assert 'typedef float ps2s_vu0_vec4[4] __attribute__((aligned(16)));' in SOURCE
assert '#define PS2S_MATRIX_VU0 1' in CPU
assert '#if (PS2S_MATRIX_VU0 != 0) && (PS2S_MATRIX_VU0 != 1)' in CPU
assert CPU.count('#if PS2S_MATRIX_VU0 && defined(_EE)') == 2
assert 'cpu_vec_4 col0, col1, col2, col3;' in CPU
for variable, prefix in (('matrix', ''), ('left', ''), ('right', 'rhs.')):
    initializer = re.search(r'const ps2s_vu0_mat4 ' + variable + r'\s*=\s*\{([^}]+)\}', CPU)[1]
    actual = [value.strip() for value in initializer.split(',')]
    assert actual == [prefix + 'col' + str(column) + '.' + lane for column in range(4) for lane in LANES]
for column in range(4):
    actual = re.search(r'result\.col' + str(column) + r' = cpu_vec_4\(([^)]+)\)', CPU)[1]
    assert [int(value) for value in re.findall(r'product\[(\d+)\]', actual)] == list(range(4 * column, 4 * column + 4))
vector_output = re.search(r'result = cpu_vec_4\((transformed\[[^;]+)\);', CPU)[1]
assert [int(value) for value in re.findall(r'transformed\[(\d+)\]', vector_output)] == list(range(4))
assert not re.search(r'\(float\s*\*\)|reinterpret_cast|memcpy', CPU)

# Actual instruction memory accesses, varied overlaps, guard words, and a
# deliberately 4-byte-but-not-16-byte-aligned Vector3 output.
rng = random.Random(0x5600)
comparisons = 0
for iteration in range(800):
    source = [f32(rng.uniform(-4096.0, 4096.0)) for _ in range(128)]
    if iteration % 17 == 0:
        source = [rng.choice((0.0, -0.0, 1.0, -1.0, 2.0**-120, 2.0**120)) for _ in source]
    for name, ops in PROGRAM_CASES:
        for out in (0x180, 0x104 if name == NAMES[2] else 0x100, 0x120, 0x110):
            memory = {0x80 + i * 4: value for i, value in enumerate(source)}
            before = dict(memory)
            pointers = {'lhs': 0x100, 'rhs': 0x120, 'mat': 0x100, 'vec': 0x120, 'out': out}
            first = [memory[0x100 + 4 * i] for i in range(16)]
            second = [memory[0x120 + 4 * i] for i in range(16 if name == NAMES[0] else 32 if name == NAMES[4] else 12 if name == NAMES[5] else 4)]
            expected = reference(name, first, second, False)
            reads, writes = execute(ops, memory, pointers, False)
            assert bits(memory[address] for address in writes) == bits(expected)
            for address, value in before.items():
                if address not in writes:
                    assert bits([memory[address]]) == bits([value]), 'guard/input corruption'
            comparisons += 1

def adjacent_dependencies(ops):
    count = 0
    for first, second in zip(ops, ops[1:]):
        destination = re.match(r'v(?:mul|add)[xyzw]?\.[xyzw]+ \$vf(\d+),', first)
        if not destination:
            continue
        used = re.findall(r'\$vf(\d+)', second)
        if not second.startswith('sqc2'):
            used = used[1:]
        count += destination[1] in used
    return count


interleaved, serial = PROGRAMS[NAMES[4]], PROGRAM_CASES[-1][1]
assert len(interleaved) == len(serial) == 68
assert interleaved[:12] == serial[:12]
assert adjacent_dependencies(interleaved) == 0 and adjacent_dependencies(serial) == 24
assert max(map(int, re.findall(r'\$vf(\d+)', ' '.join(interleaved)))) == 24
assert max(map(int, re.findall(r'\$vf(\d+)', ' '.join(serial)))) == 15

print('PASS: 103 symbolic output lanes, both batch schedules and point-first dot3, no uninitialized output lane, all input loads before stores')
print('PASS: actual typed staging/reconstruction, VU gate/default/precedence and 16-byte buffer types')
print('PASS:', comparisons, 'float32 transfer/overlap/guard cases; native xyz output exactly 12 bytes')
print('PASS: both batch schedules use 68 instructions; adjacent producer/consumer pairs 24 -> 0 (not a cycle prediction)')
print('LIMIT: source model only; no compiler, VU arithmetic emulation, interrupt proof or hardware timing')

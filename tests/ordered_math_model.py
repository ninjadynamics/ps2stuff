#!/usr/bin/env python3
"""Check ordered math source dataflow without invoking a compiler.

The oracle is the original cpu_vec_4::dot C expression. Assembly macros are
parsed into operation trees and must match that expression, including operand
order and association, for each lane. Numeric cases use host IEEE float32;
they test layout/alias integration, not undocumented EE/VU rounding or timing.
"""
import ast
import math
from pathlib import Path
import random
import re
import struct

ROOT = Path(__file__).resolve().parents[1]
MATH = (ROOT / 'include/ps2s/ordered_math.h').read_text(encoding='utf-8')
MATRIX = (ROOT / 'include/ps2s/cpu_matrix.h').read_text(encoding='utf-8')
VECTOR = (ROOT / 'include/ps2s/cpu_vector.h').read_text(encoding='utf-8')
LANES = 'xyzw'


def tree(expression):
    def visit(node):
        if isinstance(node, ast.BinOp):
            operation = {ast.Add: 'add.s', ast.Mult: 'mul.s'}[type(node.op)]
            return (operation, visit(node.left), visit(node.right))
        if isinstance(node, ast.Name):
            return node.id
        if isinstance(node, ast.Attribute):
            return visit(node.value) + '.' + node.attr
        raise AssertionError(ast.dump(node))
    return visit(ast.parse(expression.strip(), mode='eval').body)


def substitute(expression, bindings):
    if isinstance(expression, str):
        return bindings.get(expression, expression)
    return (expression[0], substitute(expression[1], bindings),
            substitute(expression[2], bindings))


def macro(name):
    definition = re.search(r'#define ' + name + r'\(([^\n]*)\) \\\n(.*?while \(0\))',
                           MATH, re.S)
    assert definition, name
    arguments = [item.strip() for item in definition[1].split(',')]
    body = definition[2]
    inputs = dict(re.findall(r'\[(\w+)\]\s+"f"\((\w+)\)', body))
    outputs = dict(re.findall(r'\[(\w+)\]\s+"=&f"\((\w+)\)', body))
    assert len(inputs) + len(outputs) <= 30
    assert set(inputs).isdisjoint(outputs)
    assert len(set(outputs.values())) == len(outputs)
    registers = dict(inputs)
    instructions = re.findall(
        r'"(mul\.s|add\.s) %\[(\w+)\], %\[(\w+)\], %\[(\w+)\]\\n\\t"', body)
    assert instructions
    for op, destination, left, right in instructions:
        assert destination in outputs
        assert left in registers and right in registers, 'read before write'
        registers[destination] = (op, registers[left], registers[right])
    assert set(outputs) <= set(registers)
    assert len(re.findall(r'"(?:mul|add)\.s ', body)) == len(instructions)
    assert not re.search(r'"(?:mula|madda|madd|v\w+)\.', body)
    return arguments, {name: registers[register] for register, name in outputs.items()}, instructions


original = VECTOR.split('cpu_vec_4::dot(const cpu_vec_4& vec) const', 1)[1]
original = original.split('#else', 1)[0]
reference = tree(re.search(r'result\s*=\s*(.*?);', original, re.S)[1])
original_bindings = {lane: 'A' + str(i) for i, lane in enumerate(LANES)}
original_bindings.update({'vec.' + lane: 'B' + str(i) for i, lane in enumerate(LANES)})
dot_reference = substitute(reference, original_bindings)

single_args, single, single_ir = macro('PS2S_ORDERED_DOT4_COP1')
pair_args, pair, pair_ir = macro('PS2S_ORDERED_DOT4_PAIR_COP1')
add_args, add, add_ir = macro('PS2S_ORDERED_DOT3_ADD_COP1')
assert single['OUT'] == dot_reference
assert pair['OUT0'] == dot_reference
assert pair['OUT1'] == substitute(dot_reference, {'A' + str(i): 'C' + str(i) for i in range(4)})
assert add['OUT'] == ('add.s', dot_reference[1], 'ADDEND')
assert [op for op, *_ in single_ir] == ['mul.s'] * 4 + ['add.s'] * 3
assert [op for op, *_ in add_ir] == ['mul.s'] * 3 + ['add.s'] * 3
assert [op for op, *_ in pair_ir] == ['mul.s'] * 8 + ['add.s'] * 6

# Read the actual column/row bindings in cpu_mat_44, not a duplicate matrix
# implementation. A transposition, operand swap or sum reassociation fails.
scalar = {}
for lane, expression in re.findall(r'result\.([xyzw])\s*=\s*(.*?);', MATRIX):
    scalar[lane] = tree(expression)
paired = {}
for call in re.findall(r'PS2S_ORDERED_DOT4_PAIR_COP1\((.*?)\);', MATRIX, re.S):
    values = [item.strip() for item in call.split(',')]
    assert len(values) == len(pair_args)
    bindings = dict(zip(pair_args, values))
    for out in ('OUT0', 'OUT1'):
        paired[bindings[out].split('.')[1]] = substitute(pair[out], bindings)

for lane in LANES:
    bindings = {field: 'col' + str(i) + '.' + lane for i, field in enumerate(LANES)}
    bindings.update({'vec.' + field: 'rhs.' + field for field in LANES})
    expected = substitute(reference, bindings)
    assert scalar[lane] == expected
    assert paired[lane] == expected

# Removing either option reveals a real independent fallback; both old bodies
# remain in the source, and private matrix storage is still four cpu_vec_4s.
for flag in ('PS2S_MATRIX_SCALAR_KERNEL', 'PS2S_MATRIX_EE_COP1'):
    assert '#ifndef ' + flag + '\n#define ' + flag + ' 1' in MATRIX
    assert '#if (' + flag + ' != 0) && (' + flag + ' != 1)' in MATRIX
    assert '#error ' + flag + ' must be 0 or 1' in MATRIX
assert 'cpu_vec_4 col0, col1, col2, col3;' in MATRIX
for i in range(4):
    assert 'result.col%d = *this * rhs.col%d;' % (i, i) in MATRIX
    assert 'result.col%d = *this * rhs.get_col%d();' % (i, i) in MATRIX
assert 'result[3] = row3.dot(rhs);' in MATRIX
assert '*reinterpret_cast' not in MATRIX and 'lqc2' not in MATH


def float32(value):
    try:
        return struct.unpack('<f', struct.pack('<f', value))[0]
    except OverflowError:
        return math.copysign(math.inf, value)


def bits(value):
    return struct.pack('<f', value)


def evaluate(expression, values):
    if isinstance(expression, str):
        return values[expression]
    left, right = evaluate(expression[1], values), evaluate(expression[2], values)
    return float32(left * right if expression[0] == 'mul.s' else left + right)


def reference_product(left, right):
    # Deliberately row-indexed mathematical oracle; candidate expressions use
    # the actual column-member bindings extracted above.
    output = []
    for column in range(4):
        for row in range(4):
            values = {lane: left[4 * i + row] for i, lane in enumerate(LANES)}
            values.update({'vec.' + lane: right[4 * column + i] for i, lane in enumerate(LANES)})
            output.append(evaluate(reference, values))
    return output


def candidate_product(left, right, expressions):
    output = []
    for column in range(4):
        values = {'col' + str(i) + '.' + lane: left[4 * i + row]
                  for i in range(4) for row, lane in enumerate(LANES)}
        values.update({'rhs.' + lane: right[4 * column + i] for i, lane in enumerate(LANES)})
        output.extend(evaluate(expressions[lane], values) for lane in LANES)
    return output


rng = random.Random(0x5900)
edge_bits = (0, 0x80000000, 1, 0x80000001, 0x007fffff, 0x00800000,
             0x3f800000, 0xbf800000, 0x3e800000, 0x40490fdb,
             0x7f7fffff, 0xff7fffff, 0x7f800000, 0xff800000, 0x7fc12345)
edges = [struct.unpack('<f', struct.pack('<I', value))[0] for value in edge_bits]
identity = [float(i // 4 == i % 4) for i in range(16)]
cases = [(identity, identity), ([0.0] * 16, [-0.0] * 16)]
for i in range(1600):
    def generate():
        if i % 3 == 0:
            return [rng.choice(edges) for _ in range(16)]
        return [float32(rng.uniform(-10000, 10000)) for _ in range(16)]
    left, right = generate(), generate()
    cases.extend(((left, right), (left, left)))
    if i % 16 == 0:
        cases.extend(((identity, left), (left, identity)))

for left, right in cases:
    expected = b''.join(map(bits, reference_product(left, right)))
    for implementation in (scalar, paired):
        actual = candidate_product(left, right, implementation)
        assert b''.join(map(bits, actual)) == expected
        # Assignment occurs after the complete returned object is evaluated.
        alias_left, alias_right = list(left), list(right)
        alias_left[:] = candidate_product(alias_left, alias_right, implementation)
        assert b''.join(map(bits, alias_left)) == expected
        alias_left, alias_right = list(left), list(right)
        alias_right[:] = candidate_product(alias_left, alias_right, implementation)
        assert b''.join(map(bits, alias_right)) == expected

# Regression witnesses: the comparison must detect operation order and signed
# zero, not merely approximate numerical equality.
a, b, c, d = 'A0', 'B0', 'A1', 'B1'
assert dot_reference != substitute(dot_reference, {a: b, b: a})
reassociated = ('add.s', ('add.s', ('mul.s', a, b), ('mul.s', c, d)),
                ('add.s', ('mul.s', 'A2', 'B2'), ('mul.s', 'A3', 'B3')))
assert reassociated != dot_reference
assert bits(0.0) != bits(-0.0)
print('PASS: 3 asm operation trees, 4 scalar + 4 paired lane bindings, gate/storage checks')
print('PASS:', len(cases), 'float32 matrix cases x 2 backends x 3 alias arrangements')
print('LIMIT: no compiler, EE execution, undocumented hardware rounding or performance claim')

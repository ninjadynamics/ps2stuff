#!/usr/bin/env python3
"""Source/byte-layout checks for PS2S_DIRECT_PACKET_TAGS; no EE compiler needed.

The new DMA-word expression is read from packet.h. The reference assigns each
manual-defined field independently, as the legacy bitfields do. Length patches
are checked over their full valid ranges, with guard bytes and TTE payloads.
This does not establish generated instruction selection or hardware performance.
"""

import ast
from pathlib import Path
import random
import re
import struct


HEADER = Path(__file__).resolve().parents[1] / "include/ps2s/packet.h"
SOURCE = HEADER.read_text(encoding="utf-8")
FIELDS = (("QWC", 0, 16), ("PCE", 26, 2), ("ID", 28, 3),
          ("IRQ", 31, 1), ("ADDR", 32, 31), ("SPR", 63, 1))
RESERVED = ((1 << 10) - 1) << 16
U64 = (1 << 64) - 1


def source_word_expression():
    expr = re.search(r"const uint64_t word = (.*?);", SOURCE, re.S).group(1)
    expr = re.sub(r"\(uint(?:32|64)_t\)", "", expr)
    expr = re.sub(r"\b(0x[0-9a-fA-F]+|[0-9]+)u\b", r"\1", expr)
    tree = ast.parse(" ".join(expr.split()), mode="eval")
    allowed = (ast.Expression, ast.BinOp, ast.BitOr, ast.BitAnd, ast.LShift,
               ast.Constant, ast.Name, ast.Load)
    assert all(isinstance(node, allowed) for node in ast.walk(tree))
    assert {node.id for node in ast.walk(tree) if isinstance(node, ast.Name)} == {
        name for name, _, _ in FIELDS}
    return compile(tree, str(HEADER), "eval")


def reference_fields(previous, values):
    for name, shift, width in FIELDS:
        mask = ((1 << width) - 1) << shift
        previous = (previous & ~mask) | ((values[name] << shift) & mask)
    return previous


def verify_dma_word(expression, values, previous, payload):
    actual = eval(expression, {"__builtins__": {}}, values)
    assert 0 <= actual <= U64
    expected = reference_fields(previous, values)
    assert actual == expected & ~RESERVED
    for name, shift, width in FIELDS:
        assert (actual >> shift) & ((1 << width) - 1) == values[name] & ((1 << width) - 1)
    packet = bytearray(b"guard123" + struct.pack("<Q", previous) + payload + b"endguard")
    original = bytes(packet)
    struct.pack_into("<Q", packet, 8, actual)
    assert packet[:8] == original[:8] and packet[16:] == original[16:]


def main():
    assert re.search(r"#ifndef PS2S_DIRECT_PACKET_TAGS\s+#define PS2S_DIRECT_PACKET_TAGS 1", SOURCE)
    assert "reinterpret_cast<Packet::TagWord*>(tag) = word" in SOURCE
    assert "reinterpret_cast<Packet::TagHalfword*>(pOpenTag)" in SOURCE
    assert "reinterpret_cast<Packet::TagHalfword*>(pOpenVifCode) = (uint16_t)numQuads" in SOURCE
    assert "reinterpret_cast<unsigned char*>(pOpenVifCode)[2] = (unsigned char)unpackNUM" in SOURCE
    assert "tag->QWC  = QWC;" in SOURCE and "pOpenVifCode->immediate = numQuads;" in SOURCE

    expression = source_word_expression()
    rng = random.Random(0x505332)
    cases = 0
    for count in (0, 1, 255, 256, 65535, 65536, 131072, 0xffffffff):
        for tag_id in range(8):
            for address in (0, 16, 0x01fffff0, 0x70003ff0, 0x80000000, 0xfffffff0):
                values = dict(QWC=count, PCE=3, ID=tag_id, IRQ=1, ADDR=address, SPR=1)
                verify_dma_word(expression, values, U64, bytes(range(8)))
                cases += 1
    for _ in range(50000):
        values = {name: rng.getrandbits(32) for name, _, _ in FIELDS}
        verify_dma_word(expression, values, rng.getrandbits(64), rng.randbytes(8))
        cases += 1

    # Exact QWC/DIRECT low-halfword patches, including each zero encoding and
    # an overflowing argument (legacy bitfield truncation), at aligned slots.
    halfword_cases = 0
    for count in range(65537):
        original = bytes(rng.getrandbits(8) for _ in range(32))
        for offset in (0, 4, 16):
            actual = bytearray(original)
            struct.pack_into("<H", actual, offset, count & 0xffff)
            expected = bytearray(original)
            old = int.from_bytes(expected[offset:offset + 8], "little")
            expected[offset:offset + 8] = ((old & ~0xffff) | (count & 0xffff)).to_bytes(8, "little")
            assert actual == expected
            halfword_cases += 1

    byte_cases = 0
    for count in range(257):
        for cmd in range(256):
            previous = (cmd << 24) | rng.getrandbits(24)
            actual = bytearray(struct.pack("<I", previous))
            actual[2] = count & 0xff
            legacy_count = 0 if count == 256 else count
            expected = (previous & ~0xff0000) | (legacy_count << 16)
            assert actual == struct.pack("<I", expected)
            byte_cases += 1

    print(f"PASS: {cases} DMA tags; {halfword_cases} QWC/DIRECT patches; "
          f"{byte_cases} UNPACK patches; reserved bits zeroed, TTE/guards and encodings preserved")


if __name__ == "__main__":
    main()

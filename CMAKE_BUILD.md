# Building ps2stuff with CMake

This document describes how to build ps2stuff using CMake instead of the traditional Makefile.

## Prerequisites

- PS2DEV environment installed and configured
- `PS2DEV` environment variable set
- CMake 3.13 or later

## Building

### Basic Build

```bash
mkdir build
cd build
cmake ..
make
```

### Debug Build

To enable debug symbols and `_DEBUG` definition:

```bash
cmake -DDEBUG=ON ..
make
```

### Building with Tests

To build the test executables (when available):

```bash
cmake -DBUILD_TESTS=ON ..
make
```

## Installing

To install the library and headers to `$PS2SDK/ports`:

```bash
make install
```

This will:
- Install `libps2stuff.a` to `$PS2SDK/ports/lib/`
- Install all headers from `include/ps2s/` to `$PS2SDK/ports/include/ps2s/`

## Configuration Options

The following CMake options are available:

| Option | Default | Description |
|--------|---------|-------------|
| `DEBUG` | OFF | Enable debug build with `_DEBUG` definition |
| `BUILD_TESTS` | OFF | Build test executables |

## Build Flags

The CMake build automatically applies the following flags:

- `-DNO_VU0_VECTORS` - Disables VU0 vector code (currently broken)
- `-DNO_ASM` - Disables assembly optimizations
- `-Wno-strict-aliasing` - Suppresses strict aliasing warnings
- `-Wno-conversion-null` - Suppresses conversion null warnings

## CMake Toolchain

The build uses the PS2DEV CMake toolchain file located at:
```
$PS2DEV/share/ps2dev.cmake
```

This toolchain file is automatically detected when `PS2DEV` is set.

## Clean Build

To perform a clean build:

```bash
rm -rf build
mkdir build
cd build
cmake ..
make
```

## Comparison with Makefile Build

The CMake build produces the same output as the traditional Makefile:
- Same compiler flags
- Same source files
- Same install locations
- Compatible library format

## Migration Notes

The CMake build system was designed to be compatible with the existing Makefile build. Both build systems can coexist in the repository.

### Key Differences:

1. **Out-of-source builds**: CMake uses a separate `build/` directory
2. **Dependency tracking**: CMake automatically handles dependencies
3. **Cross-platform**: CMake can generate build files for different build systems

## Troubleshooting

### PS2DEV not found

If you get an error about PS2DEV not being set:

```bash
export PS2DEV=/path/to/ps2dev
export PS2SDK=$PS2DEV/ps2sdk
```

### Toolchain file not found

Make sure the toolchain file exists at `$PS2DEV/share/ps2dev.cmake`.

### Build errors

Try a clean build:

```bash
rm -rf build
mkdir build
cd build
cmake ..
make
```

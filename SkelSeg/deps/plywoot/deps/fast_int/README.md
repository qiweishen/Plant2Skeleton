fast_int
--------

The `fast_int` library provides header-only C++11 implementations for the C++ `from_chars` functions for integer types. Performance of `fast_int` is at least as good as `std::from_chars` on the machines tested so far.

Example
-------

Simple example that converts some text to an integer:

```C++
#include <cassert>
#include <cstdint>
#include <cstring>

const char *s{"12345"};
std::int32_t n{};
fast_int::from_chars_result result = fast_int::from_chars(s, s + std::strlen(s), n);
std::assert(!result.ec);
std::assert(n == 12345);
```

Usage
-----

Only one public function is provided by the library:

```C++
template<typename T>
from_chars_result from_chars(const char *first, const char *last, T &value) noexcept;
```

This assumes that `T` is of some integral type. In case this precondition is not met, a compiler error is issued. Another precondition is that the textual number is in base 10. Hexadecimal numbers for example are not (yet) supported.

Other than these limitations, this tries to mimic behavior of [`std::from_chars`](https://en.cppreference.com/w/cpp/utility/from_chars) as closely as possible for integral types.

The result structure `fast_int::from_chars_result` is similar to that of `std::from_chars_result`. Please refer to the documentation of [`std::from_chars`](https://en.cppreference.com/w/cpp/utility/from_chars) for more details.

Benchmark results
-----------------

The benchmarks are somewhat limited, but the results give some impression on the performance of this library relative to other solutions. The following results were obtained on an AMD Ryzen 5 3600 6-Core processor using a Kingston A2000 NVMe SSD. PLYbench was compiled using GCC 13.21, with optimization level -O3.

```
2022-08-24T21:39:09+02:00
Running build/benchmarks
Run on (12 X 3600 MHz CPU s)
CPU Caches:
  L1 Data 32 KiB (x6)
  L1 Instruction 32 KiB (x6)
  L2 Unified 512 KiB (x6)
  L3 Unified 16384 KiB (x2)
Load Average: 0.45, 0.41, 0.27
----------------------------------------------------------------------------------
Benchmark                                        Time             CPU   Iterations
----------------------------------------------------------------------------------
BM_fast_int<std::int64_t>/100000              5126 ns         5121 ns       137953
BM_fast_int_swar<std::int64_t>/100000         4296 ns         4291 ns       162853
BM_from_chars<std::int64_t>/100000            6031 ns         6025 ns       116828
BM_fast_float<std::int64_t>/100000            5318 ns         5313 ns       130951

BM_fast_int<std::uint64_t>/100000             4257 ns         4253 ns       164435
BM_fast_int_swar<std::uint64_t>/100000        3098 ns         3095 ns       226380
BM_from_chars<std::uint64_t>/100000           5628 ns         5622 ns       123234
BM_fast_float<std::uint64_t>/100000           4840 ns         4835 ns       139433

BM_fast_int<std::int64_t>/1000000             5890 ns         5884 ns       116377
BM_fast_int_swar<std::int64_t>/1000000        4976 ns         4971 ns       139970
BM_from_chars<std::int64_t>/1000000           8803 ns         8793 ns        96929
BM_fast_float<std::int64_t>/1000000           6717 ns         6711 ns       102308

BM_fast_int<std::uint64_t>/1000000            5050 ns         5045 ns       139329
BM_fast_int_swar<std::uint64_t>/1000000       3764 ns         3760 ns       188653
BM_from_chars<std::uint64_t>/1000000          6720 ns         6713 ns       104054
BM_fast_float<std::uint64_t>/1000000          5079 ns         5074 ns       137876
```

Dependencies
------------

This library has no external dependencies. To be able to use the library, the simplest approach is to just drop `fast_int.hpp` into your project directly. To be able to install the header files and CMake configuration files, the following dependencies are required:

* [CMake](https://cmake.org/)
* [ninja](https://ninja-build.org/)

To be able to build the unit tests, the following dependency is required:

* [Catch2](https://github.com/catchorg/Catch2)

To be able to build the benchmarks, the following dependency is required:

* [Benchmark](https://github.com/google/benchmark)

In case these dependencies are not found by CMake, building of the unit tests and/or benchmarks is automatically disabled.

To be able to benchmark against `fast_float`'s `from_chars` implementation, the following dependency is required:

* [fast_float](https://github.com/fastfloat/fast_float)

Installation
------------

The `fast_int` library is a header-only library. The simplest way to include it in your project is to directly drop `fast_int.hpp` in your project. An installation target is provided by the CMake files to be able to install the header in a location of your choice. A separate `configure` script is provided to make it a bit more convenient to execute CMake. For example, to install the header files in your `$HOME/.local` directory, issue:

```
$ ./configure --prefix=$HOME/local
$ ninja -C build install
```

To build the unit tests and benchmarks in release mode:

```
$ ./configure --build-type=release
$ ninja -C build
```

CMake
-----

Installing this library will automatically install CMake configuration files that allow you to find `fast_int` from other projects:

```cmake
find_package(fast_int REQUIRED)
```

This will add a header-only library target `fast_int::fast_int` you can link to for targets that depend on this library.

Limitations
-----------

Only base 10 integers are supported for now.

Credit
------

This implementation was almost one-to-one provided by Daniel Lemire in the issue tracker of the [`fast_float`](https://github.com/fastfloat/fast_float) library, in particular [issue #86](https://github.com/fastfloat/fast_float/issues/86), where someone requests `fast_int` functionality for the [`fast_float`](https://github.com/fastfloat/fast_float) library. Since this library was implemented, someone provided a `from_chars` implementation for `fast_float` that performs a little bit better than the `from_chars` implementation in the standard library on my machine. However, it is not faster than the SWAR version provided by this library as the benchmark results show.

The SWAR implementation provided by `fast_int` was inspired by [work done by Wojciech Muła](http://0x80.pl/notesen/2014-10-12-parsing-decimal-numbers-part-1-swar.html), and the various blog posts of Daniel Lemire on the topic, e.g. [Quickly parsing eight digits](https://lemire.me/blog/2018/10/03/quickly-parsing-eight-digits/).

License
-------

Licensed under the MIT license.

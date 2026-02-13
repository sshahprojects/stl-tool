#!/bin/bash
set -e
cd "$(dirname "$0")"
CXX="${CXX:-clang++}"
SDK=$(xcrun --show-sdk-path 2>/dev/null || true)
if [ -n "$SDK" ]; then
  CXXINC="$SDK/usr/include/c++/v1"
  CXXFLAGS="-std=c++17 -isysroot $SDK -stdlib=libc++ -I$CXXINC -I../src"
else
  CXXFLAGS="-std=c++17 -I../src"
fi
$CXX $CXXFLAGS -o test_runner test_runner.cpp ../src/stl_reader.cpp
echo "Run tests: ./test_runner (from tests/ directory)"

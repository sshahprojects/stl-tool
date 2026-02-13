#!/bin/bash
set -e
CXX="${CXX:-clang++}"
SDK=$(xcrun --show-sdk-path 2>/dev/null || true)
if [ -n "$SDK" ]; then
  CXXINC="$SDK/usr/include/c++/v1"
  CXXFLAGS="-std=c++17 -isysroot $SDK -stdlib=libc++ -I$CXXINC"
else
  CXXFLAGS="-std=c++17"
fi
$CXX $CXXFLAGS -o stl_tool main.cpp stl_reader.cpp
echo "Run: ./stl_tool"

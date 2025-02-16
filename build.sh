#!/bin/sh
set -e

release=0
if [[ "$1" == "release" ]]; then
    release=1
fi

if [[ "$release" == "1" ]]; then
    echo "-----RELEASE BUILD-----"
    clang -O3 \
      -Iinclude/ \
      -Ilib/glfw/include \
      -I/Library/Developer/CommandLineTools/SDKs/MacOSX.sdk/usr/include \
      -framework Cocoa -framework OpenGL -framework IOKit -framework CoreVideo \
      -Llib/glfw/bin/macos-arm64 \
      -lglfw3 \
      -Wno-nullability-completeness \
      -o humanim_debug \
      main.c
    echo "-----RELEASE BUILD-----"
fi

if [[ "$release" == "0" ]]; then
  echo "-----DEBUG BUILD-----"
  clang -g -O0 \
    -Iinclude/ \
    -Ilib/glfw/include \
    -I/Library/Developer/CommandLineTools/SDKs/MacOSX.sdk/usr/include \
    -framework Cocoa -framework OpenGL -framework IOKit -framework CoreVideo \
    -Llib/glfw/bin/macos-arm64 \
    -lglfw3 \
    -Wno-nullability-completeness \
    -o humanim_debug \
    main.c
    echo "-----DEBUG BUILD-----"
fi

./humanim_debug
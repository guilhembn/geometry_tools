# geometry_tools

[![Build Status](https://github.com/rustyducks/geometry_tools/actions/workflows/cpp_linux_x86.yml/badge.svg)](https://github.com/rustyducks/geometry_tools/actions/workflows/cpp_linux_x86.yml)

A library of tools for (2D) geometry for robotic applications. Based on Eigen3.
Contains helpers such as: 
- **Point**: Represents a point (or a vector)
- **Angle**
- **PointOriented**: A point with an angle (a pose)
- **Speed**
- **Path**
- **Trajectory**

## Installation
Install dependencies (*Eigen3*):

```bash
sudo apt update && sudo apt install libeigen3-dev
```

Configure, build and install:

```bash
mkdir build && cd build
cmake -DCMAKE_INSTALL_PREFIX=</path/to/your/workspace> ..
make -j4
make install  # May require 'sudo' depending on the workspace path you specified
```

### Crosscompiling for ARM linux (*e.g.* raspberrypi)

Install dependencies (*Eigen3*), it is an header only library, so nothing particular for cross-compiling:

```bash
sudo apt update && sudo apt install libeigen3-dev
```

Configure, build and install:

```bash
mkdir buildarm && cd buildarm
export LINUX_ARM_TOOLCHAIN_PATH=</path/to/toolchain/folder>  # Optional, defaults to /usr/lib/ccache
cmake -DCMAKE_INSTALL_PREFIX=</path/to/your/workspace/armlinux> -DCROSSCOMPILE_ARM=ON ..
make -j4
make install
```
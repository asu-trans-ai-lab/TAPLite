# TAPLite

TAPLite is a lightweight traffic assignment engine for networks encoded in GMNS. It features a special implementation of the Frank-Wofle algorithm that traces paths for each valid OD pair.

## Quick Start

### Installation
TAPLite is available on [PyPI](https://pypi.org/project/taplite/).

```bash
pip install taplite
```

### Traffic Assignment
#### One-Time Call
```python
import taplite as tap

tap.assignment()
```

#### Recursive Call
Recursive call is suitable for multi-scenario analyses with respect to changes in demand, network topology, link capacity, and so on.

```python
import taplite as tap
from multiprocessing import freeze_support

if __name__ == '__main__':
    freeze_support()

    while True:
        # make some changes to the input
        tap.assignment()
        # save your result so that it would not be overwritten in the next run
```

## Build TAPLite from Scratch

**1. Build the C++ Shared Library**

```bash
# from the root directory of TAPLite
cmake -S . -B build -DBUILD_EXE=OFF
cmake --build build
```

You may encounter a CMake error regarding find_package(OpenMP) if you are on Apple Silicon. It is essential to inform CMake where to find the Homebrew-installed libomp package.

```bash
# from the root directory of TAPLite
cmake -S . -B build -DBUILD_EXE=OFF -DCMAKE_PREFIX_PATH=$(brew --prefix)/opt/libomp
cmake --build build
```

**2. Build and Install the Python Package**
```bash
# from the root directory of TAPLite
python -m pip install .
```
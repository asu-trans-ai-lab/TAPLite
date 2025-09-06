# TAPLite

TAPLite is a lightweight traffic assignment and simulation engine for networks encoded in GMNS.

## Quick Start

### Installation
```
pip install taplite
```

### Traffic Assignment

#### One-Time Call
```
import taplite as tap

tap.assignment()
```

#### Recursive Call
```
import taplite as tap
from multiprocessing import freeze_support

if __name__ == '__main__':
    freeze_support()

    while True:
        tap.assignment()
```

### Traffic Simulation

Note that assignment() must be invoked in the same code block prior to simulation().
```
import taplite as tap

tap.assignment()
tap.simulation()
```
## Build TAPLite from Scratch

**1. Build the C++ Shared Library**

```
# from the root directory of TAPLite
cmake -S . -B build -DBUILD_EXE=OFF
cmake --build build
```
**2. Build and Install the Python Package**
```
# from the root directory of TAPLite
python -m pip install .
```
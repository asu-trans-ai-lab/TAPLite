# TAPLite

TAPLite is a lightweight traffic assignment and simulation engine for networks encoded in GMNS.

## Quick Start

### Installation
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
```python
import taplite as tap
from multiprocessing import freeze_support

if __name__ == '__main__':
    freeze_support()

    while True:
        tap.assignment()
```

### Traffic Simulation

Note that assignment() must be invoked in the same code block prior to simulation().
```python
import taplite as tap

tap.assignment()
tap.simulation()
```
## Build TAPLite from Scratch

**1. Build the C++ Shared Library**

```bash
# from the root directory of TAPLite
cmake -S . -B build -DBUILD_EXE=OFF
cmake --build build
```
**2. Build and Install the Python Package**
```bash
# from the root directory of TAPLite
python -m pip install .
```
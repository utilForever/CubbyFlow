CubbyFlow is voxel-based fluid simulation engine for computer games based on [Jet framework](https://github.com/doyubkim/fluid-engine-dev) that was created by [Doyub Kim](https://twitter.com/doyub).
The host code is built on C++23 and can be compiled with commonly available compilers such as VC++. Optional CUDA device sources use C++17.

### Key Features

- SPH and PCISPH fluid simulators
- Stable fluids-based smoke simulator
- Level set-based liquid simulator
- PIC, FLIP, and APIC fluid simulators
- Upwind, ENO and FMM level set solvers
- Converters between signed distance function and triangular mesh
- Voxel surface reconstruction

Every simulator has both 2-D and 3-D implementations.

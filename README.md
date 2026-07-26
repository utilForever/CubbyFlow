<h1 align="center">CubbyFlow</h1>

<p align="center">
  <img src="./Medias/Logos/Logo.png" width="400" alt="CubbyFlow logo" />
</p>
<p align="center">
  <b>A voxel-based fluid simulation engine for computer games</b>
</p>
<p align="center">
  <a href="./LICENSE"><img src="https://img.shields.io/badge/License-MIT-yellow.svg" alt="License: MIT" /></a>
  <a href="https://github.com/utilForever/CubbyFlow/actions/workflows/windows.yml"><img src="https://github.com/utilForever/CubbyFlow/actions/workflows/windows.yml/badge.svg?branch=main" alt="Windows" /></a>
  <a href="https://github.com/utilForever/CubbyFlow/actions/workflows/ubuntu.yml"><img src="https://github.com/utilForever/CubbyFlow/actions/workflows/ubuntu.yml/badge.svg?branch=main" alt="Ubuntu" /></a>
  <a href="https://github.com/utilForever/CubbyFlow/actions/workflows/macos.yml"><img src="https://github.com/utilForever/CubbyFlow/actions/workflows/macos.yml/badge.svg?branch=main" alt="macOS" /></a>
  <a href="https://github.com/utilForever/CubbyFlow/actions/workflows/windows-cuda.yml"><img src="https://github.com/utilForever/CubbyFlow/actions/workflows/windows-cuda.yml/badge.svg?branch=main" alt="Windows CUDA" /></a>
  <a href="https://github.com/utilForever/CubbyFlow/actions/workflows/ubuntu-cuda.yml"><img src="https://github.com/utilForever/CubbyFlow/actions/workflows/ubuntu-cuda.yml/badge.svg?branch=main" alt="Ubuntu CUDA" /></a>
  <br />
  <a href="https://github.com/utilForever/CubbyFlow/actions/workflows/ubuntu-codecov.yml"><img src="https://github.com/utilForever/CubbyFlow/actions/workflows/ubuntu-codecov.yml/badge.svg?branch=main" alt="Code Coverage" /></a>
  <a href="https://codecov.io/gh/utilForever/CubbyFlow"><img src="https://codecov.io/gh/utilForever/CubbyFlow/branch/main/graph/badge.svg" alt="Codecov" /></a>
  <a href="https://github.com/utilForever/CubbyFlow/actions/workflows/ubuntu-sonarcloud.yml"><img src="https://github.com/utilForever/CubbyFlow/actions/workflows/ubuntu-sonarcloud.yml/badge.svg?branch=main" alt="Static Analysis" /></a>
  <a href="https://sonarcloud.io/summary/new_code?id=utilForever_CubbyFlow"><img src="https://sonarcloud.io/api/project_badges/measure?project=utilForever_CubbyFlow&metric=alert_status" alt="Quality Gate Status" /></a>
  <a href="https://sonarcloud.io/summary/new_code?id=utilForever_CubbyFlow"><img src="https://sonarcloud.io/api/project_badges/measure?project=utilForever_CubbyFlow&metric=ncloc" alt="Lines of Code" /></a>
  <br />
  <a href="https://sonarcloud.io/summary/new_code?id=utilForever_CubbyFlow"><img src="https://sonarcloud.io/api/project_badges/measure?project=utilForever_CubbyFlow&metric=sqale_rating" alt="Maintainability Rating" /></a>
  <a href="https://sonarcloud.io/summary/new_code?id=utilForever_CubbyFlow"><img src="https://sonarcloud.io/api/project_badges/measure?project=utilForever_CubbyFlow&metric=reliability_rating" alt="Reliability Rating" /></a>
  <a href="https://sonarcloud.io/summary/new_code?id=utilForever_CubbyFlow"><img src="https://sonarcloud.io/api/project_badges/measure?project=utilForever_CubbyFlow&metric=security_rating" alt="Security Rating" /></a>
  <a href="https://sonarcloud.io/summary/new_code?id=utilForever_CubbyFlow"><img src="https://sonarcloud.io/api/project_badges/measure?project=utilForever_CubbyFlow&metric=bugs" alt="Bugs" /></a>
  <a href="https://sonarcloud.io/summary/new_code?id=utilForever_CubbyFlow"><img src="https://sonarcloud.io/api/project_badges/measure?project=utilForever_CubbyFlow&metric=vulnerabilities" alt="Vulnerabilities" /></a>
  <a href="https://sonarcloud.io/summary/new_code?id=utilForever_CubbyFlow"><img src="https://sonarcloud.io/api/project_badges/measure?project=utilForever_CubbyFlow&metric=sqale_index" alt="Technical Debt" /></a>
</p>

CubbyFlow is based on the [Jet framework](https://github.com/doyubkim/fluid-engine-dev) created by [Doyub Kim](https://twitter.com/doyub). It provides matching 2-D and 3-D C++ and Python APIs for fluid simulation. Host code uses C++23; optional CUDA device code uses C++17.

## Key Features

- Basic math and geometry operations and data structures
- Spatial query accelerators
- SPH and PCISPH fluid simulators
- Stable fluids-based smoke simulator
- Level set-based liquid simulator
- PIC, FLIP, and APIC fluid simulators
- Upwind, ENO, and FMM level set solvers
- Jacobi, Gauss-Seidel, SOR, MG, CG, ICCG, and MGPCG linear system solvers
- Spherical, SPH, Zhu & Bridson, and Anisotropic kernel for points-to-surface converter
- Converters between signed distance function and triangular mesh
- C++ and Python API
- Intel TBB, OpenMP, HPX and C++11 multi-threading backends

Every simulator has both 2-D and 3-D implementations.

## Quick Start

### Prerequisites

- CMake 3.31.6 or newer
- A C++23 compiler
- A bootstrapped [vcpkg](https://github.com/microsoft/vcpkg) checkout
- Python when building `pyCubbyFlow`
- A CUDA toolkit only when building the optional CUDA backend

Set `VCPKG_ROOT` to your vcpkg checkout. CubbyFlow's manifest installs the required libraries during CMake configuration.

### 1. Clone

```sh
git clone https://github.com/utilForever/CubbyFlow.git
cd CubbyFlow
```

### 2. Build the C++ API

For macOS, Linux, or Windows Subsystem for Linux:

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DUSE_CUDA=OFF
cmake --build build --config Release
```

For Windows:

```bat
cmake -S . -B build -A x64 -DUSE_CUDA=OFF
cmake --build build --config Release
```

Run an example:

```sh
./build/bin/HybridLiquidSim
```

On Windows, the executable is `build\bin\Release\HybridLiquidSim.exe`.

### 3. Install the Python API

```sh
python -m pip install .
```

### Docker Image

```sh
docker pull cubbyflow/cubbyflow:latest
docker run -it cubbyflow/cubbyflow
# Inside the container:
/app/build/bin/HybridLiquidSim
```

See [Documents/Install.md](./Documents/Install.md) for platform-specific build, test, and installation instructions.

## Architecture at a Glance

| Area                | Paths                                         |
| ------------------- | --------------------------------------------- |
| Public C++ API      | `Includes/Core/`                              |
| Core implementation | `Sources/Core/`                               |
| Optional CUDA       | `Includes/Core/CUDA/`, `Sources/Core/CUDA/`   |
| Python bindings     | `Includes/API/Python/`, `Sources/API/Python/` |
| Tests               | `Tests/`                                      |
| Examples            | `Examples/`                                   |

Most behavior starts in the core library and is shared by matching 2-D and 3-D APIs. Python bindings expose the same core types, while CUDA remains optional. See [ARCHITECTURE.md](./ARCHITECTURE.md) for the full design and code-reading guide.

## Development

Build and run the focused CPU unit-test target:

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DUSE_CUDA=OFF -DBUILD_EXAMPLES=OFF -DBUILD_TESTS=ON
cmake --build build --target UnitTests --config Release
./build/bin/UnitTests
```

On Windows, run `build\bin\Release\UnitTests.exe`. Contributor workflow, dimensional parity, Python/CUDA synchronization, and validation rules are
documented in [AGENTS.md](./AGENTS.md).

## Documentation

Generated API reference and additional documentation are available on [the project website](https://utilforever.github.io/CubbyFlow/).

## Examples

Source code is available under [Examples](./Examples). These images were rendered with [Mitsuba](https://www.mitsuba-renderer.org/); more demos are on
[the project website](https://utilforever.github.io/CubbyFlow/Examples).

### Dam-Break Solvers

<table width="100%">
  <tr>
    <th width="33%">PCISPH</th>
    <th width="33%">Level set</th>
    <th width="33%">FLIP</th>
  </tr>
  <tr>
    <td><img src="./Medias/Screenshots/PCISPH_dam_breaking.png" width="100%" alt="PCISPH dam break" /></td>
    <td><img src="./Medias/Screenshots/LevelSet_dam_breaking.png" width="100%" alt="Level-set dam break" /></td>
    <td><img src="./Medias/Screenshots/FLIP_dam_breaking.png" width="100%" alt="FLIP dam break" /></td>
  </tr>
</table>

<table width="100%">
  <tr>
    <th width="50%">PIC</th>
    <th width="50%">APIC</th>
  </tr>
  <tr>
    <td><img src="./Medias/Screenshots/PIC_dam_breaking.png" width="100%" alt="PIC dam break" /></td>
    <td><img src="./Medias/Screenshots/APIC_dam_breaking.png" width="100%" alt="APIC dam break" /></td>
  </tr>
</table>

### Level-Set Viscosity

<table width="100%">
  <tr>
    <th width="50%">High viscosity</th>
    <th width="50%">Low viscosity</th>
  </tr>
  <tr>
    <td><img src="./Medias/Screenshots/level_set_bunny_drop_high_viscosity.png" width="100%" alt="High-viscosity level-set bunny drop" /></td>
    <td><img src="./Medias/Screenshots/level_set_bunny_drop_low_viscosity.png" width="100%" alt="Low-viscosity level-set bunny drop" /></td>
  </tr>
</table>

### Smoke Advection

<table width="100%">
  <tr>
    <th width="50%">Linear</th>
    <th width="50%">Cubic spline</th>
  </tr>
  <tr>
    <td><img src="./Medias/Screenshots/rising_smoke_linear.png" width="100%" alt="Rising smoke with linear advection" /></td>
    <td><img src="./Medias/Screenshots/rising_smoke_cubic.png" width="100%" alt="Rising smoke with cubic-spline advection" /></td>
  </tr>
</table>

## Presentations

- [NDC 2018](https://www.slideshare.net/utilforever/ndc-2018-95260566)

## Articles

- NDC 2018
  - [[NDC2018] 유체역학 엔진이 직면한 문제와 미래](http://www.inven.co.kr/webzine/news/?news=198413)
  - [[NDC18] 게임에 쓸 수 있는 유체역학 엔진, 어렵지만 꿈은 아니다](http://www.gamevu.co.kr/news/articleView.html?idxno=8464)

## How To Contribute

Contributions are welcome through issues and pull requests. Read [AGENTS.md](./AGENTS.md) for the repository's coding, testing, and review
expectations before making a change.

## Contact

You can contact me via e-mail (utilForever at gmail.com). I am always happy to answer questions or help with any issues you might have, and please be sure to share any additional work or your creations with me, I love seeing what other people are making.

## License

CubbyFlow is licensed under the [MIT License](./LICENSE).

Copyright &copy; 2017-2026 [Chris Ohk](https://github.com/utilForever)

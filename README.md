# CubbyFlow

<img src="./Medias/Logos/Logo.png" width=256 height=256 alt="CubbyFlow logo" />

[![License](https://img.shields.io/badge/Licence-MIT-blue.svg)](./LICENSE) ![Windows](https://github.com/CubbyFlow/CubbyFlow/workflows/Windows/badge.svg) ![Ubuntu](https://github.com/CubbyFlow/CubbyFlow/workflows/Ubuntu/badge.svg) ![macOS](https://github.com/CubbyFlow/CubbyFlow/workflows/macOS/badge.svg) ![Ubuntu - Codecov](https://github.com/CubbyFlow/CubbyFlow/workflows/Ubuntu%20-%20Codecov/badge.svg) [![Build Status](https://travis-ci.com/CubbyFlow/CubbyFlow.svg?branch=main)](https://travis-ci.com/CubbyFlow/CubbyFlow)

[![codecov](https://codecov.io/gh/CubbyFlow/CubbyFlow/branch/master/graph/badge.svg)](https://codecov.io/gh/CubbyFlow/CubbyFlow)
[![Codacy Badge](https://app.codacy.com/project/badge/Grade/54d8ed92a3ce4ad988be48dd2dbdeada)](https://www.codacy.com/gh/CubbyFlow/CubbyFlow/dashboard?utm_source=github.com&utm_medium=referral&utm_content=CubbyFlow/CubbyFlow&utm_campaign=Badge_Grade)
[![CodeFactor](https://www.codefactor.io/repository/github/CubbyFlow/CubbyFlow/badge)](https://www.codefactor.io/repository/github/CubbyFlow/CubbyFlow)
[![Discord](https://img.shields.io/discord/667686826093445129.svg)](https://discord.gg/3gsWZM8)

[![Quality Gate Status](https://sonarcloud.io/api/project_badges/measure?project=utilForever_CubbyFlow&metric=alert_status)](https://sonarcloud.io/dashboard?id=utilForever_CubbyFlow) [![Lines of Code](https://sonarcloud.io/api/project_badges/measure?project=utilForever_CubbyFlow&metric=ncloc)](https://sonarcloud.io/dashboard?id=utilForever_CubbyFlow) [![Maintainability Rating](https://sonarcloud.io/api/project_badges/measure?project=utilForever_CubbyFlow&metric=sqale_rating)](https://sonarcloud.io/dashboard?id=utilForever_CubbyFlow) [![Reliability Rating](https://sonarcloud.io/api/project_badges/measure?project=utilForever_CubbyFlow&metric=reliability_rating)](https://sonarcloud.io/dashboard?id=utilForever_CubbyFlow) [![Security Rating](https://sonarcloud.io/api/project_badges/measure?project=utilForever_CubbyFlow&metric=security_rating)](https://sonarcloud.io/dashboard?id=utilForever_CubbyFlow)

CubbyFlow is voxel-based fluid simulation engine for computer games based on [Jet framework](https://github.com/doyubkim/fluid-engine-dev) that was created by [Doyub Kim](https://twitter.com/doyub).
The host code is built on C++23 and can be compiled with commonly available compilers such as g++, clang++, or Microsoft Visual Studio. Optional CUDA device sources remain on C++17 for the supported CUDA toolchains. CI covers current macOS, Ubuntu, and Windows environments.

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

You will need CMake and [vcpkg](https://github.com/microsoft/vcpkg) to build the code. Set `VCPKG_ROOT` to your vcpkg checkout; CubbyFlow's manifest installs the required libraries during CMake configuration. If you're using Windows, you also need Visual Studio.

First, clone the code:

```sh
git clone https://github.com/CubbyFlow/CubbyFlow.git
cd CubbyFlow
```

### C++ API

For macOS or Linux or Windows Subsystem for Linux (WSL):

```sh
cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE="$VCPKG_ROOT/scripts/buildsystems/vcpkg.cmake"
cmake --build build
```

For Windows:

```bat
cmake -S . -B build -A x64 -DCMAKE_TOOLCHAIN_FILE="%VCPKG_ROOT%/scripts/buildsystems/vcpkg.cmake"
cmake --build build --config Release
```

Now run some examples, such as:

```
bin/HybridLiquidSim
```

### Python API

Build and install the package by running

```
pip install -U .
```

### Docker

```
docker pull cubbyflow/cubbyflow:latest
```

Now run hybrid simulation example:

```
docker run -it cubbyflow/cubbyflow
[inside docker container]
/app/build/bin/HybridLiquidSim
```

### More Instructions of Building the Code

To learn how to build, test, and install the SDK, please check out [INSTALL.md](./Documents/Install.md).

## Documentation

Project design is described in [ARCHITECTURE.md](./ARCHITECTURE.md). Generated
API reference and other documentation are available on [the project website](https://utilforever.github.io/CubbyFlow/).

## Examples

Here are some of the example simulations generated using CubbyFlow framework. Corresponding example codes can be found under [Examples](./Examples). All images are rendered using [Mitsuba renderer](https://www.mitsuba-renderer.org/) and the Mitsuba scene files can be found from [the demo directory](./Demos). Find out more demos from [the project website](https://utilforever.github.io/CubbyFlow/Examples).

#### PCISPH Simulation Example

![PCISPH_dam_breaking](./Medias/Screenshots/PCISPH_dam_breaking.png "PCISPH Example")

#### Level Set Simulation Example

![Level-set_dam_breaking](./Medias/Screenshots/LevelSet_dam_breaking.png "Level Set Example")

#### FLIP Simulation Example

![FLIP_dam_breaking](./Medias/Screenshots/FLIP_dam_breaking.png "FLIP Example")

#### PIC Simulation Example

![PIC_dam_breaking](./Medias/Screenshots/PIC_dam_breaking.png "PIC Example")

#### APIC Simulation Example

![APIC_dam_breaking](./Medias/Screenshots/APIC_dam_breaking.png "APIC Example")

#### Level Set Example with Different Viscosity (high / low)

![level_set_bunny_drop_high_viscosity](./Medias/Screenshots/level_set_bunny_drop_high_viscosity.png "Level Set Bunny Drop - High Viscosity")
![level_set_bunny_drop_low_viscosity](./Medias/Screenshots/level_set_bunny_drop_low_viscosity.png "Level Set Bunny Drop - Low Viscosity")

#### Smoke Simulation with Different Advection Methods (Linear / Cubic-Spline)

![rising_smoke_linear](./Medias/Screenshots/rising_smoke_linear.png "Rising Smoke - Linear")
![rising_smoke_cubic](./Medias/Screenshots/rising_smoke_cubic.png "Rising Smoke - Cubic")

## Presentations

- [NDC 2018](https://www.slideshare.net/utilforever/ndc-2018-95260566)

## Articles

- NDC 2018
  - [[NDC2018] 유체역학 엔진이 직면한 문제와 미래](http://www.inven.co.kr/webzine/news/?news=198413)
  - [[NDC18] 게임에 쓸 수 있는 유체역학 엔진, 어렵지만 꿈은 아니다](http://www.gamevu.co.kr/news/articleView.html?idxno=8464)

## How To Contribute

Contributions are always welcome, either reporting issues/bugs or forking the repository and then issuing pull requests when you have completed some additional coding that you feel will be beneficial to the main project. If you are interested in contributing in a more dedicated capacity, then please contact me.

## Contact

You can contact me via e-mail (utilForever at gmail.com). I am always happy to answer questions or help with any issues you might have, and please be sure to share any additional work or your creations with me, I love seeing what other people are making.

## License

CubbyFlow is licensed under the [MIT License](./LICENSE).

Copyright &copy; 2017-2026 [Chris Ohk](https://github.com/utilForever)

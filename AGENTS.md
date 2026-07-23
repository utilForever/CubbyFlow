# AGENTS.md

Guidance for AI coding agents working in this repository. Humans should start with [README.md](README.md); this file puts project-specific rules in the order an agent usually needs them.

## What this repository is

CubbyFlow is a voxel-based fluid simulation engine with C++23 host code and optional C++17 CUDA device code. It is based on the Jet framework and contains:

- CPU implementations of math, geometry, grids, particles, spatial search, level sets, pressure solvers, and fluid solvers.
- Matching 2-D and 3-D APIs for most simulation domains.
- An optional CUDA backend under `Includes/Core/CUDA/` and `Sources/Core/CUDA/`.
- A Python module named `pyCubbyFlow`, exposed through pybind11.
- C++, CUDA, Python, manual, memory-performance, and time-performance tests.
- Standalone simulation and conversion examples under `Examples/`.

The public C++ API lives under `Includes/Core/`; implementations live under `Sources/Core/`. Start there for almost every behavior change.

## Golden rules

1. **Keep 2-D and 3-D behavior aligned.** Most core algorithms, data structures, aliases, explicit template instantiations, bindings, and tests have dimensional counterparts. Check the sibling implementation before changing one side. Some geometry is inherently 3-D, so follow the existing domain rather than creating a meaningless counterpart.
2. **Keep the public API and Python-visible behavior in sync.** When changing a public type, method, default, enum, or solver behavior, inspect `Includes/API/Python/`, `Sources/API/Python/`, `Sources/API/Python/main.cpp`, and `Tests/PythonTests/`.
3. **Fix shared behavior at the shared layer.** Use `rg` to find every caller, override, binding, test, and dimensional specialization before editing. Avoid one-off guards in callers when the invariant belongs in a common base class or utility.
4. **Use existing patterns before adding code.** Search the neighboring domain for templates, builders, aliases, numerical helpers, parallel loops, serialization code, and tests. Do not add a new abstraction or dependency when the repository already has the required shape.
5. **Use CMake targets as the source of truth.** Read the root and nearest `CMakeLists.txt` before adding files, dependencies, compile definitions, or platform-specific behavior. Reconfigure CMake after adding source files; several targets use `GLOB` or `GLOB_RECURSE` without automatic reconfigure.
6. **Preserve C++23 portability.** CI builds with GCC, Clang, and MSVC across Linux, macOS, and Windows. Avoid compiler extensions unless they are isolated behind existing CMake checks.
7. **Treat warnings as failures.** `CUBBYFLOW_WARNINGS_AS_ERRORS` defaults to `ON`. Fix warnings in project code instead of suppressing them globally.
8. **Keep CUDA optional.** CPU-only configurations must continue to work. CUDA-only code belongs in the existing CUDA directories and should not leak into ordinary builds without guards. Host code uses C++23; CUDA device code remains C++17 for the supported toolchains.
9. **Do not hand-edit generated files alone.** FlatBuffers schemas live in `Sources/Core/Flatbuffers/schema/` and checked-in generated headers live in `Sources/Core/Flatbuffers/generated/`. A schema change must update both; use the FlatBuffers version constrained by `vcpkg.json`.
10. **Run the smallest relevant check.** Documentation-only changes need only document validation. Behavior changes need focused C++, Python, or CUDA coverage before broader validation.

## How the core code is shaped

### Dimensional templates

Many public types use `template <size_t N>` and expose readable aliases such as `Sphere2` and `Sphere3`. Their `.cpp` files commonly end with explicit instantiations:

```cpp
template class Sphere<2>;
template class Sphere<3>;
```

When extending this pattern:

- Put dimension-independent logic in the shared template.
- Preserve `Foo2`/`Foo3` and pointer aliases when the public type exposes them.
- Add both explicit instantiations when both dimensions are supported.
- Update both `Foo2Tests.cpp` and `Foo3Tests.cpp`, or the existing combined test file, when behavior applies to both.
- Check Python registration for matching `AddFoo2(m)` and `AddFoo3(m)` calls.

### Public headers and implementations

- Use project includes such as `<Core/Geometry/Sphere.hpp>`.
- Keep code in the `CubbyFlow` namespace.
- Public declarations and Doxygen comments belong in `Includes/Core/`.
- Non-inline implementation belongs in the matching `Sources/Core/` domain.
- Template implementation headers use the existing `-Impl.hpp` convention when definitions must be visible to callers.
- Follow nearby ownership aliases and builder APIs. If a type already provides `GetBuilder()`, `Build()`, and `MakeShared()`, extend that builder instead of adding a second construction mechanism.

### Python bindings

Python binding declarations mirror the core domains under `Includes/API/Python/`; definitions mirror them under `Sources/API/Python/`. Every exposed binding is registered in `Sources/API/Python/main.cpp`.

For a new or changed Python-visible API:

1. Update the binding declaration and implementation.
2. Add or preserve the registration call in `main.cpp` in dependency order.
3. Follow existing Python names and camelCase property conventions; do not mechanically expose C++ spelling when nearby bindings translate it.
4. Add a focused `Tests/PythonTests/test_*.py` case.
5. Build/install `pyCubbyFlow` and run that test file.

The root build adds the Python target only when `USE_CUDA=OFF` and `BUILD_SONARCLOUD=OFF`. Configure CPU-only when working directly on bindings.

### Parallel execution

Parallel helpers live in `Includes/Core/Utils/Parallel.hpp` and its implementation files. Select a backend with `CUBBYFLOW_TASKING_SYSTEM`:

- `TBB`
- `OpenMP`
- `HPX`
- `CPP11Thread`
- `Serial`

The default preference is TBB, then OpenMP, HPX, and the C++11 thread backend. Use `Serial` for focused debugging, but keep behavior correct across backends.

## Common task flow

1. Read the issue and identify the observable behavior, not just the named file.
2. Locate the public declaration under `Includes/Core/` and implementation under `Sources/Core/`.
3. Search all callers, subclasses, templates, aliases, bindings, and tests with `rg`.
4. Inspect the corresponding 2-D or 3-D implementation and reuse its shape.
5. Make the smallest change at the shared source of the behavior.
6. Update Python or CUDA code only when the affected behavior crosses those boundaries.
7. Add or update the closest focused regression test.
8. Format touched C++ and CUDA files with `.clang-format`.
9. Build the narrowest target and run the narrowest test filter.
10. Review the final diff for generated files, build output, or unrelated formatting before handing it off.

## Change-specific checklists

### Core algorithm or solver

- Check both dimensional instantiations and tests.
- Reuse `Math`, `Matrix`, `FDM`, `Utils`, and existing solver helpers.
- Preserve numerical tolerances and nearby convergence conventions.
- Run the closest GoogleTest suite; add performance tests only when performance is the requested behavior.

### Public API

- Update declarations, implementation, aliases, builders, and documentation.
- Search examples and downstream core callers for signature changes.
- Check whether the API is exposed through pybind11.
- Prefer backward-compatible defaults unless the task explicitly changes the contract.

### CUDA

- Keep host and device implementations behaviorally consistent where they represent the same operation.
- Update `.cu` sources and `Tests/CUDATests/` together.
- Configure with `USE_CUDA=ON` and build `CUDATests`.
- Run CUDA tests only when a CUDA-capable GPU is available; compilation still matters without one.

### Serialization

- Treat `.fbs` files as the source and `_generated.h` files as derived output.
- Update 2-D and 3-D schemas together when the serialized model is shared.
- Check existing serialize/deserialize callers and round-trip tests.
- Avoid changing stored layouts casually; compatibility is part of the API.

### Dependency or build configuration

- Native dependencies belong in `vcpkg.json`.
- Local vcpkg overlay ports live under `Libraries/` and are enabled by `vcpkg-configuration.json`; currently `Libraries/cnpy/` is the local port.
- Python test dependencies belong in `requirements.txt`.
- Do not vendor a new library when the standard library or an existing dependency covers the need.
- Validate at least one CPU-only configuration and any specifically affected optional configuration.

## Repository map

| Area                | Paths                                                      | What to check                                                      |
| ------------------- | ---------------------------------------------------------- | ------------------------------------------------------------------ |
| Core public API     | `Includes/Core/`                                           | Headers, templates, aliases, builders, public contracts            |
| Core implementation | `Sources/Core/`                                            | CPU algorithms and explicit template instantiations                |
| CUDA backend        | `Includes/Core/CUDA/`, `Sources/Core/CUDA/`                | Optional GPU types, kernels, and solver implementations            |
| Python binding      | `Includes/API/Python/`, `Sources/API/Python/`              | pybind11 declarations, implementations, and module registration    |
| Unit tests          | `Tests/UnitTests/`                                         | GoogleTest/GMock regression coverage                               |
| CUDA tests          | `Tests/CUDATests/`                                         | doctest coverage for CUDA code                                     |
| Python tests        | `Tests/PythonTests/`                                       | pytest coverage for `pyCubbyFlow`                                  |
| Manual tests        | `Tests/ManualTests/`                                       | Visual or scenario-driven checks; not the default regression suite |
| Performance tests   | `Tests/MemPerfTests/`, `Tests/TimePerfTests/`              | Memory checks and benchmark targets                                |
| Examples            | `Examples/`                                                | C++ and Python consumers of public APIs                            |
| Data and assets     | `Resources/`                                               | Meshes and reusable test/example inputs                            |
| FlatBuffers         | `Sources/Core/Flatbuffers/`                                | Schemas and checked-in generated headers                           |
| Build configuration | `CMakeLists.txt`, `Builds/CMake/`, nested `CMakeLists.txt` | Targets, options, warnings, tasking, and coverage                  |
| Native dependencies | `vcpkg.json`, `vcpkg-configuration.json`, `Libraries/`     | Manifest, baseline, overrides, and overlay ports                   |
| Python packaging    | `setup.py`, `requirements.txt`                             | CMake-backed extension build and pytest requirements               |
| CI                  | `.github/workflows/`                                       | Supported operating systems, compilers, CUDA builds, and coverage  |
| Documentation       | `README.md`, `Documents/`, `Documents/doxygen/`            | Installation and generated API documentation inputs                |

## Configure and build

Prerequisites are CMake 3.31.6 or newer, a C++23 compiler, Python for the Python module, and a bootstrapped vcpkg checkout. Set `VCPKG_ROOT` or `VCPKG_INSTALLATION_ROOT`, or pass `CMAKE_TOOLCHAIN_FILE` explicitly.

### Focused CPU unit-test build

Disable examples and CUDA, then build only `UnitTests`:

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DUSE_CUDA=OFF -DBUILD_EXAMPLES=OFF -DBUILD_TESTS=ON
cmake --build build --target UnitTests --config Release
```

The root build writes executables to `build/bin/` and libraries to `build/lib/`. Multi-config generators place release executables under `build/bin/Release/`.

### Useful CMake options

| Option                         | Default | Use                                                                  |
| ------------------------------ | ------- | -------------------------------------------------------------------- |
| `USE_CUDA`                     | `ON`    | Enable CUDA when a toolkit is found; otherwise falls back to CPU     |
| `BUILD_TESTS`                  | `ON`    | Build unit, manual, CUDA, and performance test targets as applicable |
| `BUILD_EXAMPLES`               | `ON`    | Build simulation and conversion examples                             |
| `BUILD_COVERAGE`               | `OFF`   | Enable GCC Debug coverage support                                    |
| `BUILD_SONARCLOUD`             | `OFF`   | Use the reduced SonarCloud build shape                               |
| `CUBBYFLOW_WARNINGS_AS_ERRORS` | `ON`    | Promote compiler warnings to errors                                  |
| `CUBBYFLOW_TASKING_SYSTEM`     | auto    | Select TBB, OpenMP, HPX, CPP11Thread, or Serial                      |

Do not reuse a build directory after changing generator, architecture, toolchain, or incompatible CUDA settings. Use a separate build directory for each such configuration.

## Run tests

Tests are executable targets rather than CTest registrations. Run the binaries directly.

### C++ unit tests

```sh
./build/bin/UnitTests
```

On Windows release builds:

```bat
build\bin\Release\UnitTests.exe
```

Use a GoogleTest filter during iteration:

```sh
./build/bin/UnitTests --gtest_filter=Sphere2.*
```

### Python tests

Build and install the extension, then run the focused file first:

```sh
python -m pip install -r requirements.txt
python -m pip install .
python -m pytest Tests/PythonTests/test_sphere.py -q
python -m pytest Tests/PythonTests/
```

### CUDA tests

```sh
cmake -S . -B build-cuda -DCMAKE_BUILD_TYPE=Release -DUSE_CUDA=ON -DBUILD_EXAMPLES=OFF -DBUILD_TESTS=ON
cmake --build build-cuda --target CUDATests --config Release
./build-cuda/bin/CUDATests
```

On Windows, use `build-cuda\bin\Release\CUDATests.exe`. A successful CUDA build without a capable GPU does not prove runtime correctness; follow the CI behavior and skip execution only when the hardware is unavailable.

### Other targets

- `ManualTests` exercises larger scenarios and may produce output files.
- `MemPerfTests` checks memory-oriented cases.
- `TimePerfTests` contains benchmark-based performance measurements.

Build and run these only for work that touches their purpose. Do not substitute performance or manual targets for regression tests.

## Validation matrix

Choose checks by the files and behavior changed:

| Change                       | Minimum validation                                                           |
| ---------------------------- | ---------------------------------------------------------------------------- |
| Documentation only           | Review rendered Markdown, links, paths, and `git diff --check`               |
| Core implementation          | Focused `UnitTests` filter                                                   |
| Shared 2-D/3-D template      | Focused tests for both dimensions                                            |
| Public Python-visible API    | Focused C++ test plus focused pytest file                                    |
| CUDA implementation          | CUDA target compilation plus focused CUDA test when GPU is available         |
| CMake or dependency manifest | Fresh configure and build of affected target                                 |
| Tasking/parallel code        | Focused test with affected backend; include `Serial` when debugging ordering |
| Serialization schema         | Regenerated headers plus focused round-trip tests                            |
| Performance-sensitive code   | Correctness test first; benchmark only after correctness passes              |

## CI expectations

The workflows under `.github/workflows/` are the full compatibility contract:

- `ubuntu.yml`, `macos.yml`, and `windows.yml` configure Release builds, compile the project, run `UnitTests`, install `pyCubbyFlow`, and run `Tests/PythonTests/`.
- `ubuntu-cuda.yml` and `windows-cuda.yml` configure with `USE_CUDA=ON`, build `CUDATests`, and execute them only when the runner exposes a CUDA-capable GPU.
- `ubuntu-codecov.yml` configures a GCC Debug build with `BUILD_COVERAGE=ON`, runs `UnitTests`, removes system and test files from the lcov report, and uploads the result to Codecov.
- `ubuntu-sonarcloud.yml` uses a reduced Debug build with `BUILD_SONARCLOUD=ON`, runs unit tests with coverage, and submits static analysis to SonarCloud.
- `docs.yml` builds Doxygen documentation and deploys it to the `gh-pages` branch after pushes to `main`.

When changing build logic, dependencies, public APIs, or platform-specific code, inspect every affected workflow rather than only the local platform. Keep action pins and platform/compiler matrices unchanged unless CI maintenance is part of the task. Report any CI-equivalent check that could not be run locally; do not claim cross-platform or GPU validation from one local configuration.

## Style and testing conventions

- Follow `.clang-format`: four-space indentation, 80-column C++ limit, sorted includes, and project brace style.
- Format only touched C++ and CUDA files; avoid repository-wide formatting in an unrelated change.
- Match existing filenames: public type files use `Type.hpp`/`Type.cpp`, unit tests use `Type2Tests.cpp` and `Type3Tests.cpp` when split by dimension, and Python tests use `test_type.py`.
- Unit tests use GoogleTest/GMock macros such as `TEST`, `EXPECT_*`, and `ASSERT_*`. CUDA tests use doctest `TEST_CASE` and `CHECK` macros.
- Prefer one focused regression scenario over broad fixtures or new test frameworks.
- Use the existing `RESOURCES_DIR` compile definition for C++ fixtures rather than depending on the current working directory.
- Preserve existing copyright headers in C++ files.
- Keep comments about intent, invariants, numerical reasoning, or non-obvious constraints. Do not narrate straightforward code.

## Commit conventions

- Keep each commit focused on one logical change. Do not mix cleanup, dependency upgrades, broad formatting, and behavior changes unless they are inseparable.
- Use the conventional prefix that matches the change: `feat:`, `fix:`, `refactor:`, `test:`, `docs:`, `build:`, `ci:`, or `chore:`.
- Commit regression tests with the behavior they protect.
- Keep Python binding updates, dimensional counterparts, schemas, and generated output in the same commit as the source change that requires them.
- Do not commit build output, test logs, caches, or unrelated local changes.
- An `Assisted-by:` trailer is optional when the maintainer wants agent assistance recorded.

## Leave alone unless asked

- Build output under `build/`, CMake-generated files, test logs, Python caches, and IDE state such as `.vs/`.
- Large media and resource files under `Medias/` and `Resources/`.
- Local vcpkg ports under `Libraries/` and dependency versions in `vcpkg.json`.
- Generated FlatBuffers headers unless the matching schema is intentionally changing.
- CI platform/compiler matrices, action pins, coverage settings, and deployment configuration.

## Before handing off

Confirm that:

1. The fix is at the shared source of the behavior and all callers were checked.
2. Supported 2-D and 3-D paths remain aligned.
3. Python and CUDA surfaces were updated when relevant.
4. The smallest relevant regression check passes.
5. Touched C++ and CUDA files are formatted.
6. The diff contains no build output, caches, or unrelated rewrites.
7. Any skipped platform, GPU, or performance validation is reported explicitly.

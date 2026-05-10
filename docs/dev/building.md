# Building from Source

This page summarises build options and common issues when compiling OpenICC.

## CMake options

| Option | Default | Description |
|--------|---------|-------------|
| `CMAKE_BUILD_TYPE` | `Release` | `Release`, `Debug`, `RelWithDebInfo` |
| `BUILD_WITH_MARCH_NATIVE` | `OFF` | Enable `-march=native` optimizations |

Example:

```bash
cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_WITH_MARCH_NATIVE=ON
make -j$(nproc)
```

## C++ standard

The project requires **C++17** because of template deduction in the SO3 spline implementation. The CMakeLists.txt sets this automatically.

## Known issues

- **Eigen alignment**: In some builds you may see alignment errors with TheiaSfM views. If this happens, uncomment the `EIGEN_MAX_ALIGN_BYTES=0` line in `CMakeLists.txt`.
- **OpenCV version mismatch**: Make sure the OpenCV version used to build TheiaSfM matches the one found by OpenICC.

## Python scripts

The Python orchestration scripts rely on the compiled applications being available in `build/applications/`. You can override the path with `--path_to_build`.

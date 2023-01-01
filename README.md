# Railway Operation Simulator

This is a remake of Railway Operation Simulator to use Open Source and Standard Library modules so the software can be developed across operating systems.
The aim is to remove the dependence on Borland-C++ and thus C++ Builder from Embarcadero.

Currently the code base has been designed to be compilable using [CMake](https://cmake.org) which is widely used build system compatible with Linux, Windows and macOS. The contained `CMakeLists.txt` also includes a macro which will download Boost.

## Status

The core components of the simulator have been converted to standard library with the minimal requirement from Boost of the `chrono` library for timekeeping. The most challenging part now comes from developing a new UI and the asynchronous processes required to run a simulation.

Currently the [Nana](https://github.com/cnjinhao/nana) GUI library has been used as a start.

## Compilation

Make sure to checkout submodules as well as this repository:

```sh
git clone --recursive -b artemis-beta/stdlib-migration https://github.com/Railway-Op-Sim/railway-dot-exe.git
```

The code is usually compiled using Clang, however compilation should also work with GCC. It is also recommended that [Ninja](https://ninja-build.org/) be used as the build system, it is faster than GNU Make as it removes the requirement to create human readable generations scripts.

### Linux

Run configuration for cmake:

```sh
cmake -Bbuild
```

The recommendation however is to use Clang as a compiler, and Ninja as the generator:

```sh
CC=$(which clang) CXX=$(which clang++) cmake -Bbuild -G Ninja 
```

then run the compilation:

```sh
cmake --build build
```

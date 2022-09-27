# Cross-compiling requires CMake 2.6 or newer. Example:
# cmake . -DCMAKE_TOOLCHAIN_FILE=XCompile.txt -DHOST=x86_64-w64-mingw32
# Where 'i686-w64-mingw32' is the host prefix for your cross-compiler. If you
# already have a toolchain file setup, you may use that instead of this file.

# the name of the target operating system
set(CMAKE_SYSTEM_NAME Windows)

# which compilers to use for C and C++
set(CMAKE_C_COMPILER "${HOST}-gcc")
set(CMAKE_CXX_COMPILER "${HOST}-g++")
set(CMAKE_RC_COMPILER "${HOST}-windres")

# here is the target environment located
set(CMAKE_FIND_ROOT_PATH "/usr/${HOST}")

# here is where stuff gets installed to
set(CMAKE_INSTALL_PREFIX "${CMAKE_FIND_ROOT_PATH}" CACHE STRING
    "Install path prefix, prepended onto install directories." FORCE)

# set env vars so that pkg-config will look in the appropriate directory for
# .pc files (as there seems to be no way to force using ${HOST}-pkg-config)
set(ENV{PKG_CONFIG_LIBDIR} "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig")
set(ENV{PKG_CONFIG_PATH} "")

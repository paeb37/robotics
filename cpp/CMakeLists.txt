cmake_minimum_required(VERSION 3.20)
project(cartpole_mpc LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

# Hide everything by default and export only the extern "C" entry points.
#
# This is not tidiness. pydrake ships its own OSQP inside libdrake.so, and this
# library links its own copy. A process that loads both -- which is exactly what
# happens when the Python simulation calls this backend -- can otherwise resolve
# OSQP symbols to whichever copy loaded first. Hidden visibility makes the two
# copies invisible to each other.
set(CMAKE_CXX_VISIBILITY_PRESET hidden)
set(CMAKE_VISIBILITY_INLINES_HIDDEN ON)

# No version constraint. Eigen's config uses same-major-version matching, so
# asking for 3.3 rejects the Eigen 5.x that Homebrew now ships. Nothing here
# depends on version-specific behaviour.
find_package(Eigen3 REQUIRED NO_MODULE)
message(STATUS "Eigen3 version: ${Eigen3_VERSION}")

# OSQP is fetched and pinned rather than taken from the system, so the build is
# reproducible in CI and Docker without depending on a package manager.
include(FetchContent)
FetchContent_Declare(
  osqp
  GIT_REPOSITORY https://github.com/osqp/osqp.git
  GIT_TAG        v1.0.0
  GIT_SHALLOW    TRUE
)
set(OSQP_BUILD_SHARED_LIB OFF CACHE BOOL "" FORCE)
set(OSQP_BUILD_STATIC_LIB ON CACHE BOOL "" FORCE)
set(OSQP_BUILD_DEMO_EXE OFF CACHE BOOL "" FORCE)
set(OSQP_BUILD_UNITTESTS OFF CACHE BOOL "" FORCE)
# 64-bit indices, double precision. Pinned explicitly rather than left to the
# defaults, because OSQPInt and OSQPFloat change type with these and the ctypes
# bridge on the Python side has to agree.
set(OSQP_USE_LONG ON CACHE BOOL "" FORCE)
set(OSQP_USE_FLOAT OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(osqp)

add_library(cartpole_mpc SHARED
  src/mpc.cpp
  src/c_api.cpp
)

target_include_directories(cartpole_mpc
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(cartpole_mpc
  PRIVATE Eigen3::Eigen
  PRIVATE osqpstatic
)

target_compile_options(cartpole_mpc PRIVATE -Wall -Wextra -O3)

# Keep the artifact next to the build tree; cpp_backend.py looks for it there.
set_target_properties(cartpole_mpc PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}
)

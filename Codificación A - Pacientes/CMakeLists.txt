cmake_minimum_required(VERSION 3.15)
project(ProyectoTFG LANGUAGES CXX)

# ================================
# Usa C++17 en todo el proyecto
set(CMAKE_CXX_STANDARD         17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS        OFF)
# ================================

set(CMAKE_BUILD_TYPE Debug)
add_compile_options(-fsanitize=address -g)
add_link_options   (-fsanitize=address)


# 1) Por defecto, Release si no piden otro build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# 2) Reemplaza -Ofast por -O3 -ffast-math en Release
#    (quita cualquier anterior -Ofast que estuvieras inyectando)
set(CMAKE_C_FLAGS_RELEASE   "-O3 -ffast-math ${CMAKE_C_FLAGS_RELEASE} -g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -ffast-math ${CMAKE_CXX_FLAGS_RELEASE} -g")

# 3) Opcional: silencia el warning de infinito de nlohmann/json
add_compile_options(-Wno-nan-infinity-disabled)

# 4) Añade tu ejecutable
add_executable(main
  main.cc
  src/Functions.cc
  src/ProblemInstance.cc
  src/RandomSolver.cc
  src/MetaHeuristicSolver.cc
  src/Solver.cc
)

# 5) Marca el include de vcpkg como SYSTEM para evitar warnings en sus headers
target_include_directories(main SYSTEM PRIVATE
  ${CMAKE_SOURCE_DIR}/vcpkg/installed/x64-osx/include
)
cmake_minimum_required(VERSION 3.5)
project(Master_project)

set (Sample_VERSION_MAJOR 0)
set (Sample_VERSION_MINOR 0)
set (Sample_VERSION_PATCH 1)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/support/cmake)

find_package (CPLEX)

configure_file(
  "${PROJECT_SOURCE_DIR}/SampleConfig.h.in"
  "${PROJECT_BINARY_DIR}/SampleConfig.h"
  )

include_directories(
SYSTEM
"${PROJECT_BINARY_DIR}"
"${CPLEX_CONCERT_INCLUDE_DIR}"
"${CPLEX_ILOCPLEX_INCLUDE_DIR}"
)

set(CMAKE_EXPORT_COMPILE_COMMANDS "ON")
add_compile_options(-Wall
  -DIL_STD
  -Wfatal-errors
  -pedantic
  -fopenmp
  -DDEBUG
  -fdiagnostics-color=always
  -std=c++14
  -lconcert -lilocplex -lcplex -lm -lpthread -ldl
)
set(BUILD_SHARED_LIBS On)

#set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fopenmp")

add_executable(main main.cpp)

target_link_libraries(main "${CPLEX_CONCERT_LIBRARY}" "${CPLEX_ILOCPLEX_LIBRARY}" "${CPLEX_LIBRARY}" pthread)


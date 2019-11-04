cmake_minimum_required(VERSION 3.5)
project(Master_project)

set (Sample_VERSION_MAJOR 0)
set (Sample_VERSION_MINOR 0)
set (Sample_VERSION_PATCH 1)
set (Project_PATH "${PROJECT_SOURCE_DIR}")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/support/cmake)

find_package (CPLEX)

configure_file(
  "${PROJECT_SOURCE_DIR}/SampleConfig.h.in"
  "${PROJECT_BINARY_DIR}/SampleConfig.h"
  )

include_directories(
SYSTEM
includes
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
  -Wall 
  -lz
  -std=c++14
  -lconcert 
  -lilocplex 
  -lcplex 
  -lm 
  -lpthread 
  -ldl
)
set(BUILD_SHARED_LIBS On)

#set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fopenmp")

file(GLOB_RECURSE SOURCES "src/*.cpp")

add_executable(main ${SOURCES})
#add_library(concert SHARED IMPORTED)
target_link_libraries(main "${CPLEX_CONCERT_LIBRARY}" "${CPLEX_ILOCPLEX_LIBRARY}" "${CPLEX_LIBRARY}" pthread m dl)



#SET(GCC_COVERAGE_COMPILE_FLAGS "-fprofile-arcs -ftest-coverage")
#SET(GCC_COVERAGE_LINK_FLAGS "ldl -lconcert -lilocplex -lcplex -lm -lpthread")

#SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS}")
#SET(CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} ${GCC_COVERAGE_LINK_FLAGS}")

cmake_minimum_required(VERSION 3.16)
project(Master_project)

##### SETs
set (Sample_VERSION_MAJOR 0)
set (Sample_VERSION_MINOR 0)
set (Sample_VERSION_PATCH 1)
set (Project_PATH "${PROJECT_SOURCE_DIR}")
set (CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/support/cmake)
set (BUILD_SHARED_LIBS On)
#SET(GCC_COVERAGE_COMPILE_FLAGS "-fprofile-arcs -ftest-coverage")
#SET(GCC_COVERAGE_LINK_FLAGS "ldl -lconcert -lilocplex -lcplex -lm -lpthread")
#SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS}")
#SET(CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} ${GCC_COVERAGE_LINK_FLAGS}")
#set(CMAKE_CXX_STANDARD 17)
#set(CMAKE_CXX_STANDARD_REQUIRED ON)
#set(CMAKE_CXX_EXTENSIONS OFF)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
#set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fopenmp")

##### Additional páckages 

find_package (CPLEX)

##### Config files 

configure_file(
  "${PROJECT_SOURCE_DIR}/SampleConfig.h.in"
  "${PROJECT_BINARY_DIR}/SampleConfig.h"
  )

##### Include Dirs 

include_directories(
  SYSTEM
  includes
  "${PROJECT_BINARY_DIR}"
  "${CPLEX_CONCERT_INCLUDE_DIR}"
  "${CPLEX_ILOCPLEX_INCLUDE_DIR}"
)

#####  Lemon Options

IF(UNIX)
  SET(INSTALL_BIN_DIR "bin" CACHE STRING
    "Subdir for installing the binaries")
  SET(INSTALL_DOC_DIR "share/doc/${PROJECT_NAME}"
    CACHE STRING
    "Subdir for installing the doc")
  SET(INSTALL_DATA_DIR "share/data/${PROJECT_NAME}"
    CACHE STRING
    "Subdir for installing the map data")
  SET(INSTALL_EXAMPLES_DIR "share/doc/${PROJECT_NAME}/examples"
    CACHE STRING
    "Subdir for installing the examples")
ELSE(UNIX)
  SET(INSTALL_BIN_DIR "." CACHE STRING
                  "Subdir for installing the binaries")
  SET(INSTALL_DOC_DIR "doc" CACHE STRING
                  "Subdir for installing the doc")
  SET(INSTALL_DATA_DIR "data" CACHE STRING
                  "Subdir for installing the map data")
  SET(INSTALL_EXAMPLES_DIR "examples" CACHE STRING
                       "Subdir for installing the examples")
ENDIF(UNIX)

## The next part looks for LEMON. Typically, you don't want to modify it.
##
## First, it tries to use LEMON as a CMAKE subproject by looking for
## it in the 'lemon' or 'deps/lemon' subdirectories or in directory
## given by the LEMON_SOURCE_ROOT_DIR variable.
## If LEMON isn't there, then CMAKE will try to find an installed
## version of LEMON. If it is installed at some non-standard place,
## then you must tell its location in the LEMON_ROOT_DIR CMAKE config
## variable. (Do not hard code it into your config! Others may keep
## LEMON at different places.)

FIND_PATH(LEMON_SOURCE_ROOT_DIR CMakeLists.txt
  PATHS ${CMAKE_SOURCE_DIR}/lemon ${CMAKE_SOURCE_DIR}/deps/lemon
  NO_DEFAULT_PATH
  DOC "Location of LEMON source as a CMAKE subproject")

IF(EXISTS ${LEMON_SOURCE_ROOT_DIR})
  ADD_SUBDIRECTORY(${LEMON_SOURCE_ROOT_DIR} deps/lemon)
  SET(LEMON_INCLUDE_DIRS
    ${LEMON_SOURCE_ROOT_DIR}
    ${CMAKE_BINARY_DIR}/deps/lemon
  )
  SET(LEMON_LIBRARIES lemon)
  UNSET(LEMON_ROOT_DIR CACHE)
  UNSET(LEMON_DIR CACHE)
  UNSET(LEMON_INCLUDE_DIR CACHE)
  UNSET(LEMON_LIBRARY CACHE)
ELSE()
  FIND_PACKAGE(LEMON QUIET NO_MODULE)
  FIND_PACKAGE(LEMON REQUIRED)
ENDIF()

## This line finds doxygen (for document creation)

FIND_PACKAGE(Doxygen)

## These are the include directories used by the compiler.

INCLUDE_DIRECTORIES(
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_BINARY_DIR}
  ${LEMON_INCLUDE_DIRS}
)

IF(CMAKE_COMPILER_IS_GNUCXX)
  SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall")
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
ENDIF(CMAKE_COMPILER_IS_GNUCXX)
ADD_SUBDIRECTORY(src)

## Sometimes MSVC overwhelms you with compiler warnings which are impossible to
## avoid. Then comment out these sections. Normally you won't need it as the
## LEMON include headers suppress these warnings anyway.

#IF(MSVC)
#  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}
#      /wd4250 /wd4355 /wd4503 /wd4800 /wd4996")
# # Suppressed warnings:
# # C4250: 'class1' : inherits 'class2::member' via dominance
# # C4355: 'this' : used in base member initializer list
# # C4503: 'function' : decorated name length exceeded, name was truncated
# # C4800: 'type' : forcing value to bool 'true' or 'false'
# #        (performance warning)
# # C4996: 'function': was declared deprecated
# ENDIF(MSVC)

ENABLE_TESTING()

## The auxiliary doxygen files (.dox) should be placed in the 'doc'
## subdirectory. The next line includes the CMAKE config of that directory.

ADD_SUBDIRECTORY(doc)

#######################################################################
## CPACK configuration
##
## It is used to configure the .exe installer created by CPACK.
## Consider editing these values:
##
## - CPACK_PACKAGE_VENDOR
## - CPACK_PACKAGE_DESCRIPTION_SUMMARY
## - CPACK_NSIS_HELP_LINK
## - CPACK_NSIS_URL_INFO_ABOUT
## - CPACK_NSIS_CONTACT
##
## Additionally, you may want to change the icons/images used by the
## NSIS installer, i.e. these variables:
##
## - CPACK_NSIS_MUI_ICON
## - CPACK_PACKAGE_ICON
## - CPACK_NSIS_INSTALLED_ICON_NAME
## 
## and/or the files they point to.
#######################################################################

##### Compile options 

add_compile_options(-Wall
  -DIL_STD
  -Wfatal-errors
  -pedantic
  -fopenmp
  -DDEBUG
  -fdiagnostics-color=always
  -Wall 
  -lz
  -std=c++17
  -lconcert 
  -lilocplex 
  -lcplex 
  -lm 
  -lpthread 
  -ldl
)

file(GLOB_RECURSE SOURCES "src/*.cpp")

add_executable(main ${SOURCES})

set (CMAKE_CXX_COMPILER "g++-7")

#add_library(concert SHARED IMPORTED)
target_link_libraries(main "${CPLEX_CONCERT_LIBRARY}" "${CPLEX_ILOCPLEX_LIBRARY}" "${CPLEX_LIBRARY}" pthread m dl)

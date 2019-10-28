# - Try to find CPLEX
# Once done this will define
#  CPLEX_FOUND - System has Gurobi
#  CPLEX_INCLUDE_DIRS - The Gurobi include directories
#  CPLEX_LIBRARIES - The libraries needed to use Gurobi

find_path(CPLEX_INCLUDE_DIR 
          NAMES ilcplex/ilocplex.h ilcplex/ilocplexi.h
          PATHS "$ENV{CPLEX_HOME}/cplex/include"
          )

find_path(CONCERT_INCLUDE_DIR 
          NAMES "ilconcert/ilomodel.h"
          PATHS "$ENV{CPLEX_HOME}/concert/include"
          )

find_library( ILO_CPLEX_LIBRARY 
              NAMES ilocplex
              PATHS "$ENV{CPLEX_HOME}/cplex/lib/x86-64_linux/static_pic" 
              )
find_library( CPLEX_LIBRARY 
              NAMES cplex
              PATHS "$ENV{CPLEX_HOME}/cplex/lib/x86-64_linux/static_pic" 
              )

find_library( CONCERT_LIBRARY 
              NAMES concert
              PATHS "$ENV{CPLEX_HOME}/concert/lib/x86-64_linux/static_pic" 
              )

set(CPLEX_INCLUDE_DIRS "${CPLEX_INCLUDE_DIR}" "${CONCERT_INCLUDE_DIR}" )
set(CPLEX_LIBRARIES "${ILO_CPLEX_LIBRARY};${CPLEX_LIBRARY};${CONCERT_LIBRARY}" )

include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set LIBCPLEX_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(CPLEX  DEFAULT_MSG
                                  ILO_CPLEX_LIBRARY CPLEX_LIBRARY CONCERT_LIBRARY CPLEX_INCLUDE_DIR CONCERT_INCLUDE_DIR)

mark_as_advanced(CPLEX_INCLUDE_DIR CONCERT_INCLUDE_DIR ILO_CPLEX_LIBRARY CPLEX_LIBRARY CONCERT_LIBRARY)


# Find the header files

#SET(G2O_LOCAL_DIR ${PROJECT_SOURCE_DIR}/../g2o)
#SET(G2O_DEEPER_LOCAL_DIR ${PROJECT_SOURCE_DIR}/../../g2o)
#SET(G2O_EXTERNAL_DIR ${PROJECT_SOURCE_DIR}/../../external/g2o_wrapper/g2o)
#message(STATUS "Searching for g2o in " $ENV{G2O_ROOT})
set(G2O_SRRG_DIR $ENV{HOME}/source/libraries/g2o)
message("srrg_cmake_modules|searching for g2o in: " ${G2O_SRRG_DIR})

FIND_PATH(G2O_INCLUDE_DIR g2o/core/base_vertex.h
  #${G2O_LOCAL_DIR}
  #${G2O_DEEPER_LOCAL_DIR}
  #${G2O_EXTERNAL_DIR}
  ${G2O_SRRG_DIR}
  $ENV{G2O_ROOT}/include
  $ENV{G2O_ROOT}
  /usr/local/include
  /usr/include
  /opt/local/include
  /sw/local/include
  /sw/include
  NO_DEFAULT_PATH
  )
if(G2O_INCLUDE_DIR)
  message("srrg_cmake_modules|found g2o:  ${G2O_INCLUDE_DIR}")
else()
  set(G2O_SRRG_DIR "")
  message("srrg_cmake_modules|g2o not found")
endif()

# Macro to unify finding both the debug and release versions of the
# libraries; this is adapted from the OpenSceneGraph FIND_LIBRARY
# macro.

MACRO(FIND_G2O_LIBRARY MYLIBRARY MYLIBRARYNAME)

  FIND_LIBRARY("${MYLIBRARY}_DEBUG"
    NAMES "g2o_${MYLIBRARYNAME}_d"
    PATHS
    #${G2O_LOCAL_DIR}/lib/Debug
    #${G2O_LOCAL_DIR}/lib
    #${G2O_DEEPER_LOCAL_DIR}/lib/Debug
    #${G2O_DEEPER_LOCAL_DIR}/lib
    #${G2O_EXTERNAL_DIR}/lib/Debug
    #${G2O_EXTERNAL_DIR}/lib
    ${G2O_SRRG_DIR}/lib/Debug
    ${G2O_SRRG_DIR}/lib
    ${G2O_ROOT}/lib/Debug
    ${G2O_ROOT}/lib
    $ENV{G2O_ROOT}/lib/Debug
    $ENV{G2O_ROOT}/lib
    NO_DEFAULT_PATH
    )

  FIND_LIBRARY("${MYLIBRARY}_DEBUG"
    NAMES "g2o_${MYLIBRARYNAME}_d"
    PATHS
    ~/Library/Frameworks
    /Library/Frameworks
    /usr/local/lib
    /usr/local/lib64
    /usr/lib
    /usr/lib64
    /opt/local/lib
    /sw/local/lib
    /sw/lib
    )

  FIND_LIBRARY(${MYLIBRARY}
    NAMES "g2o_${MYLIBRARYNAME}"
    PATHS
    #${G2O_LOCAL_DIR}/lib/Release
    #${G2O_LOCAL_DIR}/lib
    #${G2O_DEEPER_LOCAL_DIR}/lib/Release
    #${G2O_DEEPER_LOCAL_DIR}/lib
    #${G2O_EXTERNAL_DIR}/lib/Release
    #${G2O_EXTERNAL_DIR}/lib
    ${G2O_SRRG_DIR}/lib/Release
    ${G2O_SRRG_DIR}/lib
    ${G2O_ROOT}/lib/Release
    ${G2O_ROOT}/lib
    $ENV{G2O_ROOT}/lib/Release
    $ENV{G2O_ROOT}/lib
    NO_DEFAULT_PATH
    )

  FIND_LIBRARY(${MYLIBRARY}
    NAMES "g2o_${MYLIBRARYNAME}"
    PATHS
    ~/Library/Frameworks
    /Library/Frameworks
    /usr/local/lib
    /usr/local/lib64
    /usr/lib
    /usr/lib64
    /opt/local/lib
    /sw/local/lib
    /sw/lib
    )

  IF(NOT ${MYLIBRARY}_DEBUG)
    IF(MYLIBRARY)
      SET(${MYLIBRARY}_DEBUG ${MYLIBRARY})
    ENDIF(MYLIBRARY)
  ENDIF( NOT ${MYLIBRARY}_DEBUG)

ENDMACRO(FIND_G2O_LIBRARY LIBRARY LIBRARYNAME)

# Find the core elements
FIND_G2O_LIBRARY(G2O_STUFF_LIBRARY stuff)
FIND_G2O_LIBRARY(G2O_OPENGL_HELPER_LIBRARY opengl_helper)
FIND_G2O_LIBRARY(G2O_CORE_LIBRARY core)
FIND_G2O_LIBRARY(G2O_HIERARCHICAL_LIBRARY hierarchical)

# Find the CLI library
FIND_G2O_LIBRARY(G2O_CLI_LIBRARY cli)

# Find the pluggable solvers
FIND_G2O_LIBRARY(G2O_SOLVER_CHOLMOD solver_cholmod)
FIND_G2O_LIBRARY(G2O_SOLVER_CSPARSE solver_csparse)
FIND_G2O_LIBRARY(G2O_SOLVER_CSPARSE_EXTENSION csparse_extension)
FIND_G2O_LIBRARY(G2O_SOLVER_DENSE solver_dense)
FIND_G2O_LIBRARY(G2O_SOLVER_PCG solver_pcg)
FIND_G2O_LIBRARY(G2O_SOLVER_SLAM2D_LINEAR solver_slam2d_linear)
FIND_G2O_LIBRARY(G2O_SOLVER_STRUCTURE_ONLY solver_structure_only)
FIND_G2O_LIBRARY(G2O_SOLVER_EIGEN solver_eigen)

# Find the predefined types
FIND_G2O_LIBRARY(G2O_TYPES_DATA types_data)
FIND_G2O_LIBRARY(G2O_TYPES_ICP types_icp)
FIND_G2O_LIBRARY(G2O_TYPES_SBA types_sba)
FIND_G2O_LIBRARY(G2O_TYPES_SCLAM2D types_sclam2d)
FIND_G2O_LIBRARY(G2O_TYPES_SIM3 types_sim3)
FIND_G2O_LIBRARY(G2O_TYPES_SLAM2D types_slam2d)
FIND_G2O_LIBRARY(G2O_TYPES_SLAM3D types_slam3d)
FIND_G2O_LIBRARY(G2O_TYPES_SLAM2D_ADDONS types_slam2d_addons)
FIND_G2O_LIBRARY(G2O_TYPES_SLAM3D_ADDONS types_slam3d_addons)

#ia Find plugins
FIND_G2O_LIBRARY(G2O_PLUGIN_TYPES_CHORDAL types_chordal3d)
FIND_G2O_LIBRARY(G2O_PLUGIN_TYPES_MATCHABLES types_matchables)

# G2O solvers declared found if we found at least one solver
SET(G2O_SOLVERS_FOUND "NO")
IF(G2O_SOLVER_CHOLMOD OR G2O_SOLVER_CSPARSE OR G2O_SOLVER_DENSE OR G2O_SOLVER_PCG OR G2O_SOLVER_SLAM2D_LINEAR OR G2O_SOLVER_STRUCTURE_ONLY OR G2O_SOLVER_EIGEN)
  SET(G2O_SOLVERS_FOUND "YES")
ENDIF(G2O_SOLVER_CHOLMOD OR G2O_SOLVER_CSPARSE OR G2O_SOLVER_DENSE OR G2O_SOLVER_PCG OR G2O_SOLVER_SLAM2D_LINEAR OR G2O_SOLVER_STRUCTURE_ONLY OR G2O_SOLVER_EIGEN)

# G2O itself declared found if we found the core libraries and at least one solver
SET(G2O_FOUND "NO")
IF(G2O_STUFF_LIBRARY AND G2O_CORE_LIBRARY AND G2O_INCLUDE_DIR AND G2O_SOLVERS_FOUND)
  SET(G2O_FOUND "YES")
ENDIF(G2O_STUFF_LIBRARY AND G2O_CORE_LIBRARY AND G2O_INCLUDE_DIR AND G2O_SOLVERS_FOUND)

# Set variables for srrg environment
# srrg@dom surround these with IF(G2O_FOUND)??
set(SRRG_G2O_INCLUDE ${G2O_INCLUDE_DIR})
set(SRRG_G2O_LIBRARIES
  ${G2O_STUFF_LIBRARY}
  ${G2O_OPENGL_HELPER_LIBRARY}
  ${G2O_CORE_LIBRARY}
  ${G2O_HIERARCHICAL_LIBRARY}
  ${G2O_CLI_LIBRARY}
  ${G2O_SOLVER_CHOLMOD}
  ${G2O_SOLVER_CSPARSE}
  ${G2O_SOLVER_CSPARSE_EXTENSION}
  ${G2O_SOLVER_DENSE}
  ${G2O_SOLVER_PCG}
  ${G2O_SOLVER_SLAM2D_LINEAR}
  ${G2O_SOLVER_STRUCTURE_ONLY}
  ${G2O_SOLVER_EIGEN}
  ${G2O_TYPES_DATA}
  ${G2O_TYPES_ICP}
  ${G2O_TYPES_SBA}
  ${G2O_TYPES_SCLAM2D}
  ${G2O_TYPES_SIM3}
  ${G2O_TYPES_SLAM2D}
  ${G2O_TYPES_SLAM3D}
  ${G2O_TYPES_SLAM2D_ADDONS}
  ${G2O_TYPES_SLAM3D_ADDONS}
  )

#ia check if we have the chordal plugin
set(G2O_CHORDAL_PLUGIN_FOUND 0)
if(G2O_PLUGIN_TYPES_CHORDAL)
  set(G2O_CHORDAL_PLUGIN_FOUND 1)
  set(SRRG_G2O_LIBRARIES ${SRRG_G2O_LIBRARIES} ${G2O_PLUGIN_TYPES_CHORDAL})
endif()

#ia check if we have the matchable plugin
set(G2O_MATCHABLE_PLUGIN_FOUND 0)
if(G2O_PLUGIN_TYPES_MATCHABLES)
  set(G2O_MATCHABLE_PLUGIN_FOUND 1)
  set(SRRG_G2O_LIBRARIES ${SRRG_G2O_LIBRARIES} ${G2O_PLUGIN_TYPES_MATCHABLES})
endif()

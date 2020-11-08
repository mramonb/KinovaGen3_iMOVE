# Pre-requisites about cmake itself
CMAKE_MINIMUM_REQUIRED(VERSION 3.5)

if(COMMAND cmake_policy)
  cmake_policy(SET CMP0005 NEW) 
  cmake_policy(SET CMP0003 NEW)
   
endif(COMMAND cmake_policy)

# The project name and the type of project
PROJECT(testing_kinova)

SET(EXECUTABLE_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib)
SET(CMAKE_INSTALL_PREFIX /usr/local)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE release)
endif()

if(NOT KORTEX_SUB_DIR)
  set(KORTEX_SUB_DIR "linux_gcc_x86-64")
else()
  set(KORTEX_SUB_DIR "${KORTEX_SUB_DIR}/")
endif()

if(NOT MYO_SUB_DIR)
  set(MYO_SUB_DIR "myolinux_1.1.0_x86_64")
else()
  set(MYO_SUB_DIR "${MYO_SUB_DIR}/")
endif()

# Activate C++ 11
SET(CMAKE_CXX_STANDARD 11)

SET(CMAKE_CXX_FLAGS_DEBUG "-g -Wall -D_REENTRANT")
SET(CMAKE_CXX_FLAGS_RELEASE "-O3 -D_REENTRANT")


#### KORTEX
add_definitions(-D_OS_UNIX)
##################################################################################
set(KORTEX_DIR "${PROJECT_SOURCE_DIR}/lib/kortex_api/${KORTEX_SUB_DIR}")
message("KORTEX_DIR " ${KORTEX_DIR})

##### Link to lib
link_libraries(${KORTEX_DIR}/lib/${CMAKE_BUILD_TYPE}/libKortexApi.a)

# link other libs
link_libraries(pthread)

# add include dir
include_directories(${KORTEX_DIR}/include)
include_directories(${KORTEX_DIR}/include/client)
include_directories(${KORTEX_DIR}/include/common)
include_directories(${KORTEX_DIR}/include/messages)
include_directories(${KORTEX_DIR}/include/client_stubs)
##################################################################################

#### MYO ARMBAND
##################################################################################
set(MYO_DIR "${PROJECT_SOURCE_DIR}/lib/myo_api/${MYO_SUB_DIR}")
message("MYO_DIR " ${MYO_DIR})

##### Link to lib
link_libraries(${MYO_DIR}/lib/libmyolinux.so)

# add include dir
include_directories(${MYO_DIR}/include)
include_directories(${MYO_DIR}/include/myolinux)
##################################################################################

#### PINOCCHIO
##################################################################################
find_path(pinocchio_INCLUDE_DIR "pinocchio" paths "/usr/include" "/usr/local/include" path_suffixes "pinocchio")
message("PINOCCHIO DIR " ${pinocchio_INCLUDE_DIR})
if (NOT pinocchio_INCLUDE_DIR)
    message(FATAL_ERROR "CMake variable pinocchio_INCLUDE_DIR not found.")
endif()

##### Link to lib
link_libraries(/opt/openrobots/lib/libpinocchio.so)

# add include dir
include_directories("${pinocchio_INCLUDE_DIR}")
##################################################################################

ADD_SUBDIRECTORY(src)

FIND_PACKAGE(Doxygen)

FIND_PATH(IRI_DOC_DIR doxygen.conf ${CMAKE_SOURCE_DIR}/doc/iri_doc/)
IF (IRI_DOC_DIR)
  ADD_CUSTOM_TARGET (doc ${DOXYGEN_EXECUTABLE} ${CMAKE_SOURCE_DIR}/doc/iri_doc/doxygen.conf)
ELSE (IRI_DOC_DIR)
  ADD_CUSTOM_TARGET (doc ${DOXYGEN_EXECUTABLE} ${CMAKE_SOURCE_DIR}/doc/doxygen.conf)
ENDIF (IRI_DOC_DIR)

ADD_CUSTOM_TARGET (distclean @echo cleaning cmake files)

IF (UNIX)
  ADD_CUSTOM_COMMAND(
    COMMENT "distribution clean"
    COMMAND make ARGS clean
    COMMAND rm ARGS -rf ${CMAKE_SOURCE_DIR}/build/*

    TARGET  distclean
  )
ELSE(UNIX)
  ADD_CUSTOM_COMMAND(
    COMMENT "distclean only implemented in unix"
    TARGET  distclean
  )
ENDIF(UNIX)

ADD_CUSTOM_TARGET (uninstall @echo uninstall package)

IF (UNIX)
  ADD_CUSTOM_COMMAND(
    COMMENT "uninstall package"
    COMMAND xargs ARGS rm < install_manifest.txt

    TARGET  uninstall
  )
ELSE(UNIX)
  ADD_CUSTOM_COMMAND(
    COMMENT "uninstall only implemented in unix"
    TARGET  uninstall
  )
ENDIF(UNIX)



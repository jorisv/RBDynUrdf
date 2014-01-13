INCLUDE(../../cmake/python.cmake)
FINDPYTHON()

## Define PYTHON_DISTLIB
EXECUTE_PROCESS(
  COMMAND "${PYTHON_EXECUTABLE}" "-c"
  "import sys, os; print os.sep.join(['lib', 'python' + sys.version[:3], 'dist-packages'])"
  OUTPUT_VARIABLE PYTHON_DISTLIB
  ERROR_QUIET)
# Remove final \n of the variable PYTHON_DISTLIB
STRING(REPLACE "\n" "" PYTHON_DISTLIB "${PYTHON_DISTLIB}")


# RBDynUrdf

# create the package in build dir for testing purpose
set(LIBRARY_OUTPUT_PATH ${CMAKE_CURRENT_BINARY_DIR}/rbdyn_urdf)
configure_file(__init__.py ${CMAKE_CURRENT_BINARY_DIR}/rbdyn_urdf/__init__.py COPYONLY)

set(OUTPUT_BINDING ${CMAKE_CURRENT_BINARY_DIR}/rbdyn_urdf.cpp)

# generate python binding code
add_custom_command (
  OUTPUT ${OUTPUT_BINDING}
  COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/generate.py ${OUTPUT_BINDING}
  DEPENDS generate.py
)

# build the library
set(SOURCES ${OUTPUT_BINDING})
include_directories(.)
include_directories(../../src)
include_directories(${PYTHON_INCLUDE_DIRS})

add_library(_rbdyn_urdf SHARED ${SOURCES})
PKG_CONFIG_USE_DEPENDENCY(_rbdyn_urdf urdfdom_headers)
PKG_CONFIG_USE_DEPENDENCY(_rbdyn_urdf urdfdom)
PKG_CONFIG_USE_DEPENDENCY(_rbdyn_urdf RBDyn)

target_link_libraries(_rbdyn_urdf RBDynUrdf)
set_target_properties(_rbdyn_urdf PROPERTIES PREFIX "")


# install rules
set(INSTALL_PATH "${PYTHON_DISTLIB}/rbdyn_urdf/")
install(TARGETS _rbdyn_urdf DESTINATION ${INSTALL_PATH})
install(FILES __init__.py DESTINATION ${INSTALL_PATH})


ENABLE_TESTING()

set(Boost_USE_STATIC_LIBS ON)
set(BOOST_COMPONENTS unit_test_framework)
SEARCH_FOR_BOOST()

include_directories("${PROJECT_SOURCE_DIR}/src")
include_directories(${Boost_INCLUDE_DIRS})

set(HEADERS XYZSarm.h)

macro(addUnitTest name)
  add_executable(${name} ${name}.cpp ${HEADERS})
  target_link_libraries(${name} ${Boost_LIBRARIES} RBDynUrdf)
  PKG_CONFIG_USE_DEPENDENCY(${name} RBDyn urdfdom_headers urdfdom)
  add_test(${name}Unit ${name})
endmacro(addUnitTest)

addUnitTest("XYZSarmTest")

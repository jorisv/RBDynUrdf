set(SOURCES Reader.cpp Writer.cpp UrdfStruct.cpp)
set(HEADERS Reader.h Writer.h UrdfStruct.h)

add_library(RBDynUrdf SHARED ${SOURCES} ${HEADERS})
PKG_CONFIG_USE_DEPENDENCY(RBDynUrdf urdfdom_headers)
PKG_CONFIG_USE_DEPENDENCY(RBDynUrdf urdfdom)
PKG_CONFIG_USE_DEPENDENCY(RBDynUrdf RBDyn)

set(INSTALL_PATH include/RBDynUrdf)
install(FILES ${HEADERS} DESTINATION ${INSTALL_PATH})
install(TARGETS RBDynUrdf DESTINATION "lib")

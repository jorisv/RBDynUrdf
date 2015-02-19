RBDynUrdf
=========

RBDynUrdf reads URDF files and converts them to RBDyn MultiBodyGraph.

## Install

```sh
mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX="/your/install/prefix" -DCMAKE_BUILD_TYPE=RelWithDebInfo
make
make install
```

Note: if you are on a Debian-based distribution (e.g. Ubuntu), you may want to
add the `-DPYTHON_DEB_LAYOUT` flag to the `cmake` command in order to install
this package with the specific Debian layout.

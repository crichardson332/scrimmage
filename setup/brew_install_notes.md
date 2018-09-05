This is a list of dependencies for scrimmage and their availability in homebrew for macOS

KEY
===========================
  y - brew has this package
  n - brew does not have this package
  u - this package is not needed for brew on macOS

LIST
---------------------------
For packages with a different name in homebrew than
Debian, the brew name is listed below, indented

y git
y cmake
y gcc
n build-essential
  xcode-select --install (command line tools)
n librapidxml-dev
y libeigen3-dev
  Eigen
y libgeographic-dev
  GeographicLib
y libboost-thread-dev
y libboost-date-time-dev
y libboost-iostreams-dev
y libboost-program-options-dev
y libboost-regex-dev
y libboost-filesystem-dev
y libboost-system-dev
  Boost
y autoconf
y automake
y libtool
y curl -> already included in base macOS
y unzip


y ccache
y parallel
y libbullet-dev
  bullet
n sphinx-rtd-theme-common
y graphviz
y doxygen
y libopencv-dev
  OpenCV
y libvtk6-dev
  vtk
  Homebrew gives a warning for vtk: 
    Warning: netcdf dependency gcc was built with a different C++ standard library (libstdc++ from clang). This may cause problems at runtime.
  NOTE: need to install vtk version 6.2, which has to be built from source.
n tcl-vtk

y python
  Install with pip:
y python-setuptools
y python-numpy
y python-sphinx
y python-matplotlib
y python-pandas
  Non-pip:
y python-wxtools
y python-wxmpl
  wxpython (for the two above)
  Importing wx in python gives a warning:
    /usr/local/lib/python2.7/site-packages/wx-3.0-osx_cocoa/wx/_core.py:16633: UserWarning: wxPython/wxWidgets release number mismatch
  warnings.warn("wxPython/wxWidgets release number mismatch")

u python-pip
u python-dev

y python3
  To get pip:
    $ brew postinstall python3
  Install with pip3:
y python3-setuptools
y python3-numpy
y python3-sphinx
y python3-matplotlib
y python3-pandas

u python3-pip
u python3-dev

y pip install sphinx-git pydoe
y pip3 install sphinx-git pydoe

---------
3rd party
---------
y pybind11
y grpc
u protobuf

MISSING DEPENDENCIES:
librapidxml-dev
jsbsim
  Must be installed with .dmg:
  https://github.com/downloads/arktools/jsbsim/jsbsim-1.0.0.113-Darwin.dmg
sphinx-rtd-theme-common
tcl-vtk

VTK:
  Needs to build vtk version 6.2 for use with scrimmage. The tarbal is available here:
  https://github.com/Kitware/VTK/archive/v6.2.0.tar.gz
  brew formula can be based on the formula for VTK 8, which is part of homebrew/core


NOTES ON BUILD:
pybind is not being linked properly
#include <condition_variable> needs to be included in 'include/scrimmage/simcontrol/SimControl.h'

msgpack_extension
=================

msgpack c++ extension for robotics libraries

# build
~~~~
    mkdir build
    cd build
    cmake -DCMAKE_BUILD_TYPE=Debug .. -DROOT_INCLUDE_DIRS_HINTS=$HOME/install/include -DROOT_LIBRARY_DIRS_HINTS=$HOME/install/lib 
    make
    ./msgpacktest
~~~~

cmake_minimum_required(VERSION 2.8)
project(msgpackexercise)
set( CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS TRUE )

message(status "proc type: ${CMAKE_SYSTEM_PROCESSOR}")

# http://www.cmake.org/cmake/help/cmake-2.6.html#policy:CMP0002
cmake_policy(SET CMP0002 NEW)
# http://www.cmake.org/cmake/help/cmake-2.6.html#policy:CMP0003
cmake_policy(SET CMP0003 NEW)

# Use, i.e. don't skip the full RPATH for the build tree
set(CMAKE_SKIP_BUILD_RPATH FALSE)

# When building, don't use the install RPATH already
# (but later on when installing)
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)

set(LIB_SUFFIX CACHE STRING "suffix for the library directory need for x86-64 systems that use lib64 ")

# The RPATH to be used when installing
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib${LIB_SUFFIX}")

# Add the automatically determined parts of the RPATH
# which point to directories outside the build tree to the install RPATH
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

if( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )
  add_definitions("-fno-strict-aliasing -Wall")
endif( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )

# BUILD_Debug and BUILD_Release needed by PCL?
if( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX OR COMPILER_IS_CLANG)
  set(CMAKE_CXX_FLAGS_OPTIMIZED "${CMAKE_CXX_FLAGS_OPTIMIZED} -O3 -DNDEBUG -DBOOST_DISABLE_ASSERTS -D_SECURE_SCL=0 -DBUILD_Release") # this practically removes all checks making it a very dangerous options to play with
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -DNDEBUG -DBUILD_Release")
  set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} -O2 -g -DBUILD_Release")
  set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g3 -D_DEBUG -DBUILD_Debug")
else()
  set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -DBUILD_Debug")
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -DBUILD_Release")
endif()

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# MSVC settings
if( MSVC70 OR MSVC71 )
  set(MSVC_PREFIX "vc70")
elseif( MSVC80 )
  set(MSVC_PREFIX "vc80")
elseif( MSVC90 )
  set(MSVC_PREFIX "vc90")
else()
  set(MSVC_PREFIX "vc100")
endif()

# msgpack
set(MSGPACK_INCLUDE_DIRS "$ENV{HOME}/install/include")
set(MSGPACK_LIBRARY_DIRS "$ENV{HOME}/install/lib/")
set(MSGPACK_LIBRARIES msgpack msgpackc)

# zmq
find_package(libzmq)
if (libzmq_FOUND)
  message(STATUS "libzmq found")
else ()
  message(STATUS "libzmq not found")
endif()

# don't install
#if( MSVC )
	#install(DIRECTORY "${MSVC_BINARIES_DIR}/${CMAKE_SYSTEM_PROCESSOR}/${MSVC_PREFIX}/bin/" DESTINATION bin)
#endif()

find_package(Boost)
find_package(PCL)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(${MSGPACK_INCLUDE_DIRS} ${libzmq_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR})
link_directories(${MSGPACK_LIBRARY_DIRS} ${libzmq_LIBRARY_DIRS})
add_executable(msgpacktest main.cpp)
set_target_properties(msgpacktest PROPERTIES COMPILE_FLAGS "-std=c++0x ${Boost_CFLAGS}" LINK_FLAGS "")
target_link_libraries(msgpacktest ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY} ${Boost_THREAD_LIBRARY} ${Boost_DATE_TIME_LIBRARY} ${MSGPACK_LIBRARIES} ${libzmq_LIBRARIES} ${PCL_LIBRARIES})

install(TARGETS msgpacktest DESTINATION bin)

# add make uninstall capability
#configure_file("${CMAKE_CURRENT_SOURCE_DIR}/cmake_uninstall.cmake.in" "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake" IMMEDIATE @ONLY)
#add_custom_target(uninstall "${CMAKE_COMMAND}" -P "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake")

CMAKE_MINIMUM_REQUIRED(VERSION 2.6 FATAL_ERROR)
CMAKE_POLICY(VERSION 2.6)

FILE(GLOB src *.c *.h *.cpp *.hpp)
FILE(GLOB luasrc *.lua)
SET(luasrc ${luasrc} dartwrap.inl)

MESSAGE(STATUS "CMAKE_INSTALL_PREFIX ${CMAKE_INSTALL_PREFIX}")
MESSAGE(STATUS "TORCH_INSTALL_PREFIX ${TORCH_INSTALL_PREFIX}")
MESSAGE(STATUS "LUAROCKS_INSTALL_PREFIX ${LUAROCKS_INSTALL_PREFIX}")

IF ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
    MESSAGE(STATUS "Using Clang")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")        ## Optimize
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ftemplate-depth=1024")
    MESSAGE(STATUS "FLAGS ${CMAKE_CXX_FLAGS}")
# Using GCC.
ELSEIF ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
    MESSAGE(STATUS "Using GCC")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")        ## Optimize
    SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3")        ## Optimize
    MESSAGE(STATUS "FLAGS ${CMAKE_CXX_FLAGS}")
# Using Intel C++.
ELSEIF ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Intel")
    MESSAGE(STATUS "Using Intel C++")
# Using Visual Studio C++.
ELSEIF ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
    MESSAGE(STATUS "Using Visual Studio C++")
ENDIF()


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall")
find_package(DART 6.1.1 REQUIRED
                  PATHS /home/fritz/torch/install
                  COMPONENTS gui utils-urdf)

ADD_TORCH_PACKAGE(dartwrap "${src}" "${luasrc}" "Dart Wrapper")
TARGET_LINK_LIBRARIES(dartwrap  PUBLIC ${DART_LIBRARIES})
TARGET_INCLUDE_DIRECTORIES(dartwrap PUBLIC ${DART_INCLUDE_DIRS})


add_executable(mlhole mlhole.cpp controller.cpp)
target_include_directories(mlhole PUBLIC ${DART_INCLUDE_DIRS})
target_link_libraries(mlhole PUBLIC ${DART_LIBRARIES})


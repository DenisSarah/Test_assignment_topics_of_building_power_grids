cmake_minimum_required(VERSION 3.16)
project(cable_graph_reader CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(core
    src/io.cpp
    src/graph.cpp
    src/geojson_writer.cpp
    src/hdd.cpp 
)

target_include_directories(core PUBLIC include)

add_executable(reader src/main.cpp)
target_link_libraries(reader PRIVATE core)

if(MSVC)
    target_compile_options(core PRIVATE /W4)
    target_compile_options(reader PRIVATE /W4)
else()
    target_compile_options(core PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(reader PRIVATE -Wall -Wextra -Wpedantic)
endif()

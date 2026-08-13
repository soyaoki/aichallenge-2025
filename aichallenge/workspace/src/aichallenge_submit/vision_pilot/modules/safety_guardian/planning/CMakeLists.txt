cmake_minimum_required(VERSION 3.22.1)

find_package(Eigen3 REQUIRED)

add_library(planning
        src/planning.cpp
        src/longitudinal_planning.cpp
        src/lateral_planning.cpp

)

target_include_directories(planning
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_compile_definitions(planning
        PRIVATE
        HAVE_CSTDDEF
)

target_link_libraries(planning
        PUBLIC
        common
        Eigen3::Eigen
)

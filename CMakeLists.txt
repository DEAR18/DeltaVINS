cmake_minimum_required(VERSION 3.16)
project(DeltaVINS)

add_definitions("-std=c++14")


if(${CMAKE_HOST_SYSTEM_PROCESSOR} MATCHES "arm*")
    set(ARM TRUE)
    message("Architecture ARM")
    add_definitions("-DPLATFORM_ARM")
endif()

if(${CMAKE_HOST_SYSTEM_PROCESSOR} STREQUAL "aarch64")
    set(ARM TRUE)
    message("Architecture ARM")
    add_definitions("-DPLATFORM_ARM")
endif()

if(${CMAKE_HOST_SYSTEM_NAME} STREQUAL Windows)
    set(Windows TRUE)
    message("OS:Windows")
endif()

if(${CMAKE_HOST_SYSTEM_NAME} STREQUAL Linux)
    set(Linux TRUE)
    message(OS : Linux)

endif()

set(BUILD_TESTS TRUE)


set(source_root ${PROJECT_SOURCE_DIR})
set(src_dir ${source_root}/src)

# # if (ENABLE_VISUALIZER_TCP)

# #     add_definitions(-DENABLE_VISUALIZER_TCP=1)

# elseif(ENABLE_VISUALIZER)

    add_definitions(-DENABLE_VISUALIZER=1)

# endif ()


add_definitions(-DEIGEN_STACK_ALLOCATION_LIMIT=0)

find_package(OpenCV)
find_package(Pangolin)
find_package(Eigen3)

include_directories(include/framework)
include_directories(3rdParty/CmdParser)

add_library(SlamVisualizer SHARED
            ${source_root}/SlamVisualizer/SlamVisualizer.cpp
            )

target_link_libraries(SlamVisualizer
Eigen3::Eigen
opencv::opencv
pangolin::pangolin
)

file(GLOB src_files
        ${src_dir}/*.cpp
        ${src_dir}/Algorithm/*.cpp
        ${src_dir}/Algorithm/DataAssociation/*.cpp
        ${src_dir}/Algorithm/IMU/*.cpp
        ${src_dir}/Algorithm/vision/*.cpp
        ${src_dir}/Algorithm/vision/camModel/*.cpp
        ${src_dir}/Algorithm/vision/fast/*.cpp
        ${src_dir}/dataStructure/*.cpp
        ${src_dir}/utils/*.cpp
        ${src_dir}/framework/*.cpp
        ${src_dir}/IO/dataBuffer/*.cpp
        ${src_dir}/IO/dataSource/*.cpp
        ${src_dir}/IO/dataOutput/*.cpp
        ${src_dir}/Algorithm/solver/*.cpp
        )

include_directories(
        ${source_root}/include
        ${source_root}/3rdParty
        ${source_root}/3rdParty/serial/include
        ${source_root}/SlamVisualizer
)



add_library( ${CMAKE_PROJECT_NAME} SHARED
        ${src_files}
        )

        target_link_libraries(${CMAKE_PROJECT_NAME}
        Eigen3::Eigen
        opencv::opencv
        pangolin::pangolin
        )


target_precompile_headers(${CMAKE_PROJECT_NAME} PRIVATE include/precompile.h)



if(Linux)

set(LINK_LIBS
    ${CMAKE_PROJECT_NAME}
   pthread)
   
set(VIS_LIB
GL
GLEW
)

endif()

   set(LINK_LIBS
        ${LINK_LIBS}
        SlamVisualizer
        ${VIS_LIB}
   )


add_executable(DeltaVINSTest
        ${source_root}/test/DeltaVINSTest.cpp
        )

target_link_libraries(
        DeltaVINSTest
        ${LINK_LIBS}
)


set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES PUBLIC_HEADER
        "include/framework/slamAPI.h;include/dataStructure/sensorStructure.h"

        )

set_target_properties(${CMAKE_PROJECT_NAME}  PROPERTIES PRIVATE_HEADER
        "include/utils/basicTypes.h")


set(INSTALL_TARGETS
        ${CMAKE_PROJECT_NAME} DeltaVINSTest
        )

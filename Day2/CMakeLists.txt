cmake_minimum_required(VERSION 3.25)
project(Day2)

set(CMAKE_CXX_STANDARD 17)

# ソースファイル
set(SOURCES
        src/main.cpp
        src/Image.cpp
        src/Sphere.cpp
        src/Camera.cpp
        src/Renderer.cpp
        src/Color.cpp
        src/Mesh.cpp
        src/Triangle.cpp
)

set(HEADERS
        src/Ray.h
        src/Sphere.h
        src/Film.h
        src/Camera.h
        src/Body.h
        src/Renderer.h
        src/Material.h
        src/Color.h
        src/Image.h
        src/BubbleLoader.h
        src/Mesh.h
        src/Triangle.h
)

add_executable(${PROJECT_NAME} ${SOURCES} ${HEADERS})

# Eigen3
find_package(Eigen3 CONFIG REQUIRED)
target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen)

# OpenCV
find_package(OpenCV CONFIG REQUIRED)
target_link_libraries(${PROJECT_NAME} PUBLIC opencv_ml opencv_dnn opencv_core opencv_flann opencv_imgcodecs opencv_highgui)

# OpenMP for parallelization
find_package(OpenMP)
if(OPENMP_CXX_FOUND)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} ${OpenMP_SHARED_LINKER_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

# コンパイラ最適化フラグ
if(CMAKE_BUILD_TYPE STREQUAL "Release")
    if(MSVC)
        target_compile_options(${PROJECT_NAME} PRIVATE /O2 /arch:AVX2)
    else()
        target_compile_options(${PROJECT_NAME} PRIVATE -O3 -march=native)
    endif()
endif()
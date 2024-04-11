cmake_minimum_required(VERSION 3.0)
project(myslam)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_STANDARD 14)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

############### dependencies ######################
# Eigen
include_directories("/usr/include/eigen3")

# OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# pangolin
find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

# Sophus
find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

# G2O
find_package(G2O REQUIRED)
include_directories(${G2O_INCLUDE_DIRS})

# glog
find_package(Glog REQUIRED)
include_directories(${GLOG_INCLUDE_DIRS})

# gtest
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})

# gflags
find_package(GFlags REQUIRED)
include_directories(${GFLAGS_INCLUDE_DIRS})

# csparse
find_package(CSparse REQUIRED)
include_directories(${CSPARSE_INCLUDE_DIR})

# caffe (default path: ~/caffe)
add_definitions( -DCPU_ONLY=1 )
if (NOT Caffe_ROOT_DIR)
        set(Caffe_ROOT_DIR "/home/brave/Desktop/soft/caffe")
endif()
MESSAGE("${GREEN}Using Caffe_ROOT_DIR: ${Caffe_ROOT_DIR}${ENDL}")
set(Caffe_LIBS ${Caffe_ROOT_DIR}/build/lib/libcaffe.so)
set(Caffe_INCLUDE_DIRS ${Caffe_ROOT_DIR}/build/install/include)
include_directories(${Caffe_INCLUDE_DIRS})

# cuda (optional)
find_package(CUDA QUIET)
if (${CUDA_FOUND})
        include_directories(${CUDA_INCLUDE_DIRS})
        MESSAGE("CUDA_INCLUDE_DIRS: ${CUDA_INCLUDE_DIRS}")
endif()

set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
# 查找Boost库，包括Filesystem组件
find_package(Boost REQUIRED COMPONENTS filesystem)
# 包含必要的头文件搜索路径
include_directories(${Boost_INCLUDE_DIRS})
link_directories( ${Boost_LIBRARIES})

set(THIRD_PARTY_LIBS
        ${OpenCV_LIBS}
        ${Sophus_LIBRARIES}
        ${Pangolin_LIBRARIES} GL GLU GLEW glut
        g2o_core g2o_stuff g2o_types_sba g2o_solver_csparse g2o_csparse_extension
        ${GTEST_BOTH_LIBRARIES}
        ${GLOG_LIBRARIES}
        ${GFLAGS_LIBRARIES}
        pthread
        ${CSPARSE_LIBRARY}
        fmt
        ${Caffe_LIBS}
        ${Boost_LIBRARIES}
        ${CMAKE_THREAD_LIBS_INIT}
        ${CUDA_LIBRARIES}
        ${CUDA_CUBLAS_LIBRARIES}
        ${CUDA_curand_LIBRARY}
        )

enable_testing()

############### source and test ######################
include_directories(${PROJECT_SOURCE_DIR}/include)
add_subdirectory(src)
add_subdirectory(test)
add_subdirectory(app)

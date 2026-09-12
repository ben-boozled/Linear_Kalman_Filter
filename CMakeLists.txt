cmake_minimum_required(VERSION 3.14.0) # recommended to use FetchContent
project(Linear_Kalman_Filter VERSION 1.0.0 LANGUAGES CXX)

# ======================== Dependencies ========================
add_definitions(-w) # suppress all compilation warnings for targets in the current directory and its subdirectories

include(FetchContent)

# Matplot++
set(MATPLOTPLUSPLUS_VERSION v1.2.2)

FetchContent_Declare(matplotplusplus
        URL https://github.com/alandefreitas/matplotplusplus/archive/refs/tags/${MATPLOTPLUSPLUS_VERSION}.tar.gz
        FIND_PACKAGE_ARGS ${MATPLOTPLUSPLUS_VERSION}
        SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/libs/matplotplusplus)

set(MATPLOTPP_BUILD_EXAMPLES OFF CACHE INTERNAL "") # stops matplotplusplus from building its examples to save time

FetchContent_GetProperties(matplotplusplus)
if(NOT matplotplusplus_POPULATED)
    FetchContent_Populate(matplotplusplus)
    add_subdirectory(${matplotplusplus_SOURCE_DIR} ${matplotplusplus_BINARY_DIR} EXCLUDE_FROM_ALL) 
endif() # checks that matplotplusplus is available, else it downloads matplotplusplus based on the parameters in FetchContent_Declare()
# Important note: FetchContent_Populate() is deprecated in CMake 3.30 and should ideally be replaced with FetchContent_MakeAvailable()
# It is only used here because matplotplusplus has configuration errors when used with FetchContent_MakeAvailable()

# Eigen
set(EIGEN_VERSION 5.0.0)

FetchContent_Declare(eigen
        URL https://gitlab.com/libeigen/eigen/-/archive/5.0.0/eigen-${EIGEN_VERSION}.tar.gz
        FIND_PACKAGE_ARGS ${EIGEN_VERSION}
        SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/libs/Eigen)

set(EIGEN_BUILD_DEMOS OFF CACHE INTERNAL "") # stops Eigen from building its examples to save time

FetchContent_MakeAvailable(eigen) # checks that eigen is available, else it downloads eigen based on the parameters in FetchContent_Declare()

# ======================== Project Specific ========================
set(SOURCE_FILES
    src/main.cpp
    src/kalmanfilter.cpp
    src/pseudosensors.cpp)

add_executable(${PROJECT_NAME} ${SOURCE_FILES})

target_link_libraries(${PROJECT_NAME} 
                        PUBLIC 
                        Eigen3::Eigen
                        matplot) # target names are based on official READMEs

target_include_directories(${PROJECT_NAME} PUBLIC include)

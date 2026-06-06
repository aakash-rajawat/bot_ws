cmake_minimum_required(VERSION 3.16)
project(bot_dugma LANGUAGES CXX CUDA)

if(POLICY CMP0104)
  cmake_policy(SET CMP0104 NEW)
endif()

if(NOT DEFINED CMAKE_CUDA_ARCHITECTURES)
  set(CMAKE_CUDA_ARCHITECTURES 61)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(CUDAToolkit REQUIRED)

set(ROPTLIB_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/ROPTLIB)

if(NOT EXISTS "${ROPTLIB_ROOT}/Solvers/LRBFGS.cpp")
    message(FATAL_ERROR
        "ROPTLIB not found at ${ROPTLIB_ROOT}. "
        "Run: git submodule update --init --recursive third_party/ROPTLIB"
    )
endif()

file(GLOB ROPTLIB_MANIFOLD_SOURCES CONFIGURE_DEPENDS
    ${ROPTLIB_ROOT}/Manifolds/*.cpp
)
file(GLOB ROPTLIB_OTHER_SOURCES CONFIGURE_DEPENDS
    ${ROPTLIB_ROOT}/Others/*.cpp
)
file(GLOB ROPTLIB_PROBLEM_SOURCES CONFIGURE_DEPENDS
    ${ROPTLIB_ROOT}/Problems/*.cpp
)
file(GLOB ROPTLIB_SOLVER_SOURCES CONFIGURE_DEPENDS
    ${ROPTLIB_ROOT}/Solvers/*.cpp
)

add_library(roptlib STATIC
    ${ROPTLIB_MANIFOLD_SOURCES}
    ${ROPTLIB_OTHER_SOURCES}
    ${ROPTLIB_PROBLEM_SOURCES}
    ${ROPTLIB_SOLVER_SOURCES}
    ${ROPTLIB_ROOT}/Others/SparseBLAS/nist_spblas.cpp
    ${ROPTLIB_ROOT}/Others/wavelet/wavelet.cpp
)

target_include_directories(roptlib
    PRIVATE
        ${ROPTLIB_ROOT}
        ${ROPTLIB_ROOT}/BinaryFiles
        ${ROPTLIB_ROOT}/Julia
        ${ROPTLIB_ROOT}/Manifolds
        ${ROPTLIB_ROOT}/Matlab
        ${ROPTLIB_ROOT}/Matlab/ForCpp
        ${ROPTLIB_ROOT}/Matlab/ForMatlab
        ${ROPTLIB_ROOT}/Others
        ${ROPTLIB_ROOT}/Others/SparseBLAS
        ${ROPTLIB_ROOT}/Others/fftw
        ${ROPTLIB_ROOT}/Others/wavelet
        ${ROPTLIB_ROOT}/Problems
        ${ROPTLIB_ROOT}/Solvers
        ${ROPTLIB_ROOT}/cwrapper
        ${ROPTLIB_ROOT}/cwrapper/blas
        ${ROPTLIB_ROOT}/cwrapper/lapack
)

target_compile_features(roptlib PUBLIC cxx_std_17)
target_compile_definitions(roptlib PUBLIC ROPTLIB_WITH_FFTW)

target_link_libraries(roptlib
    PRIVATE
        lapack
        blas
        fftw3
        f2c
        m
)

add_library(bot_dugma SHARED
    src/types.cpp
    src/normalize.cpp
    src/cuda_utils.cu
    src/initialize.cpp
    src/initialize_cuda.cu
    src/dugma_registrar.cpp
    src/e_step.cpp
    src/e_step_cuda.cu
    src/lrbfgs_wrapper.cpp
    src/m_step.cpp
    src/uncertainty_estimator.cpp
    src/uncertainty_estimator_cuda.cu
)

target_include_directories(bot_dugma
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        ${ROPTLIB_ROOT}
        ${ROPTLIB_ROOT}/cwrapper/blas
        ${ROPTLIB_ROOT}/cwrapper/lapack
        ${ROPTLIB_ROOT}/Others
        ${ROPTLIB_ROOT}/Others/SparseBLAS
        ${ROPTLIB_ROOT}/Others/fftw
        ${ROPTLIB_ROOT}/Others/wavelet
        ${CUDAToolkit_INCLUDE_DIRS}
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

set_target_properties(roptlib bot_dugma PROPERTIES
    POSITION_INDEPENDENT_CODE ON
)

target_link_libraries(bot_dugma
    PRIVATE
        Eigen3::Eigen
        CUDA::cudart
        roptlib
)

target_compile_features(bot_dugma PUBLIC cxx_std_17)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

install(
    FILES
        include/bot_dugma/types.hpp
        include/bot_dugma/dugma_registrar.hpp
    DESTINATION include/bot_dugma
)

install(
    TARGETS bot_dugma
    EXPORT export_bot_dugma
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

ament_export_include_directories(include)
ament_export_libraries(bot_dugma)
ament_export_targets(export_bot_dugma HAS_LIBRARY_TARGET)
ament_package()

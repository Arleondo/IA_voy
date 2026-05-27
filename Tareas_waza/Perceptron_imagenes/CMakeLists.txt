cmake_minimum_required(VERSION 4.1)

project(Perceptron_imagenes LANGUAGES CUDA CXX)

set(CMAKE_CUDA_STANDARD 20)
set(CMAKE_CUDA_ARCHITECTURES 86)

add_executable(
        Perceptron_imagenes
        src/main.cpp
        src/mnist_loader.h
        src/perceptron.cu
        src/perceptron.cuh
        src/mnist_loader.cpp
        pruebitas/perceptron.h
)

target_include_directories(
        Perceptron_imagenes PRIVATE include
)

set_target_properties(Perceptron_imagenes PROPERTIES
        CUDA_SEPARABLE_COMPILATION ON)
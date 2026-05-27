#include "perceptron.cuh"

#include <cuda_runtime.h>
#include <device_launch_parameters.h>

#include <vector>
#include <cstdlib>
#include <iostream>

using namespace std;

__global__
void forwardKernel(
    float* input,
    float* weights,
    float* biases,
    float* outputs
)
{
    int neuron = blockIdx.x;
    int pixel = threadIdx.x;

    atomicAdd(
        &outputs[neuron],
        input[pixel]
        * weights[neuron * 784 + pixel]
    );

    if(pixel == 0)
    {
        outputs[neuron]
            += biases[neuron];
    }
}

__global__
void trainKernel(
    float* input,
    float* weights,
    float* biases,
    float* outputs,
    int label,
    float learningRate
)
{
    int neuron = blockIdx.x;
    int pixel = threadIdx.x;

    float target = 0.0f;

    if(neuron == label)
    {
        target = 1.0f;
    }

    float error =
        target - outputs[neuron];

    weights[neuron * 784 + pixel]
        += learningRate
           * error
           * input[pixel];

    if(pixel == 0)
    {
        biases[neuron]
            += learningRate * error;
    }
}

GPUPerceptron::GPUPerceptron()
{
    cudaMalloc(
        &d_weights,
        10 * 784 * sizeof(float)
    );

    cudaMalloc(
        &d_biases,
        10 * sizeof(float)
    );

    cudaMalloc(
        &d_outputs,
        10 * sizeof(float)
    );

    cudaMalloc(
        &d_input,
        784 * sizeof(float)
    );

    vector<float> h_weights(
        10 * 784
    );

    vector<float> h_biases(
        10,
        0.0f
    );

    for(int i = 0;
        i < 10 * 784;
        i++)
    {
        h_weights[i] =
            ((float)rand() / RAND_MAX)
            - 0.5f;
    }

    cudaMemcpy(
        d_weights,
        h_weights.data(),
        10 * 784 * sizeof(float),
        cudaMemcpyHostToDevice
    );

    cudaMemcpy(
        d_biases,
        h_biases.data(),
        10 * sizeof(float),
        cudaMemcpyHostToDevice
    );
}

GPUPerceptron::~GPUPerceptron()
{
    cudaFree(d_weights);
    cudaFree(d_biases);
    cudaFree(d_outputs);
    cudaFree(d_input);
}

void GPUPerceptron::forward(
    const float* input,
    float* outputs
)
{
    cudaMemcpy(
        d_input,
        input,
        784 * sizeof(float),
        cudaMemcpyHostToDevice
    );

    cudaMemset(
        d_outputs,
        0,
        10 * sizeof(float)
    );

    forwardKernel<<<10, 784>>>(
        d_input,
        d_weights,
        d_biases,
        d_outputs
    );

    cudaDeviceSynchronize();

    cudaMemcpy(
        outputs,
        d_outputs,
        10 * sizeof(float),
        cudaMemcpyDeviceToHost
    );
}

void GPUPerceptron::train(
    const float* input,
    int label,
    float learningRate
)
{
    cudaMemcpy(
        d_input,
        input,
        784 * sizeof(float),
        cudaMemcpyHostToDevice
    );

    cudaMemset(
        d_outputs,
        0,
        10 * sizeof(float)
    );

    forwardKernel<<<10, 784>>>(
        d_input,
        d_weights,
        d_biases,
        d_outputs
    );

    cudaDeviceSynchronize();

    trainKernel<<<10, 784>>>(
        d_input,
        d_weights,
        d_biases,
        d_outputs,
        label,
        learningRate
    );

    cudaDeviceSynchronize();
}
#include "perceptron.cuh"

#include <cuda_runtime.h>
#include <device_launch_parameters.h>

#include <vector>
#include <cstdlib>
#include <fstream>
#include <iostream>

using namespace std;

__device__
float stepFunction(float x){
    if(x > 0.0f) return 1.0f;

    return 0.0f;
}

__global__
void forwardKernel(const float* input,const float* weights,const float* biases,float* outputs){
    const int neuron = blockIdx.x;
    const int pixel = threadIdx.x;

    atomicAdd( &outputs[neuron],input[pixel] * weights[neuron * 784 + pixel]);

    if(pixel == 0){ outputs[neuron] += biases[neuron]; }
}

__global__
void activationKernel(float* outputs){
    const int neuron = threadIdx.x;

    outputs[neuron] = stepFunction(outputs[neuron]);
}

__global__
void trainKernel( const float* input,float* weights,float* biases,const float* outputs, const int label, const float learningRate ) {
    const int neuron = blockIdx.x;
    const int pixel = threadIdx.x;
    float target = 0.0f;

    if(neuron == label){ target = 1.0f; }

    float error = target - outputs[neuron];

    // update solo si hay error
    if(error != 0.0f){
        weights[neuron * 784 + pixel] += learningRate * error * input[pixel];

        if(pixel == 0){
            biases[neuron] += learningRate * error;
        }
    }
}

GPUPerceptron::GPUPerceptron(){

    cudaMalloc( &d_weights,10 * 784 * sizeof(float));

    cudaMalloc( &d_biases,10 * sizeof(float) );

    cudaMalloc( &d_outputs,10 * sizeof(float) );

    cudaMalloc( &d_input,784 * sizeof(float) );

    vector<float> h_weights(10 * 784);
    vector<float> h_biases(10,0.0f);

    //pesos pequeños
    for(int i = 0;i < 10 * 784;i++){
        h_weights[i] =((static_cast<float>(rand()) / RAND_MAX) * 0.02f) - 0.01f;
    }

    cudaMemcpy(d_weights,h_weights.data(),10 * 784 * sizeof(float),cudaMemcpyHostToDevice);
    cudaMemcpy(d_biases,h_biases.data(),10 * sizeof(float),cudaMemcpyHostToDevice);
}

GPUPerceptron::~GPUPerceptron(){
    cudaFree(d_weights);
    cudaFree(d_biases);
    cudaFree(d_outputs);
    cudaFree(d_input);
}

void GPUPerceptron::forward( const float* input, float* outputs ) const{
    cudaMemcpy(d_input,input,784 * sizeof(float),cudaMemcpyHostToDevice);

    cudaMemset(d_outputs,0,10 * sizeof(float) );

    forwardKernel<<<10, 784>>>(d_input,d_weights,d_biases,d_outputs);

    activationKernel<<<1, 10>>>(d_outputs);

    cudaDeviceSynchronize();

    cudaMemcpy(outputs,d_outputs,10 * sizeof(float),cudaMemcpyDeviceToHost);
}

void GPUPerceptron::train(const float* input,int label,float learningRate) const {

    cudaMemcpy(d_input,input,784 * sizeof(float),cudaMemcpyHostToDevice);

    cudaMemset(d_outputs,0,10 * sizeof(float));

    forwardKernel<<<10, 784>>>(d_input,d_weights,d_biases,d_outputs);

    activationKernel<<<1, 10>>>(d_outputs);

    trainKernel<<<10, 784>>>(d_input,d_weights,d_biases,d_outputs,label,learningRate);

    cudaDeviceSynchronize();
}

void GPUPerceptron::saveModel(const char* filename) const {
    vector<float> h_weights(10 * 784);

    vector<float> h_biases(10);

    cudaMemcpy(h_weights.data(),d_weights,10 * 784 * sizeof(float),cudaMemcpyDeviceToHost);

    cudaMemcpy(h_biases.data(),d_biases,10 * sizeof(float),cudaMemcpyDeviceToHost);

    ofstream file(filename,ios::binary);

    file.write( reinterpret_cast<char *>(h_weights.data()),10 * 784 * sizeof(float) );

    file.write( reinterpret_cast<char *>(h_biases.data()),10 * sizeof(float) );

    file.close();
}

void GPUPerceptron::loadModel(const char* filename) const {
    vector<float> h_weights(10 * 784);

    vector<float> h_biases(10);

    ifstream file(filename,ios::binary);

    file.read( reinterpret_cast<char *>(h_weights.data()),10 * 784 * sizeof(float) );

    file.read( reinterpret_cast<char *>(h_biases.data()),10 * sizeof(float) );

    file.close();

    cudaMemcpy(d_weights,h_weights.data(),10 * 784 * sizeof(float),cudaMemcpyHostToDevice);

    cudaMemcpy(d_biases,h_biases.data(),10 * sizeof(float),cudaMemcpyHostToDevice);
}

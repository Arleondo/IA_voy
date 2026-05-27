#ifndef PERCEPTRON_IMAGENES_PERCEPTRON_CUH
#define PERCEPTRON_IMAGENES_PERCEPTRON_CUH

#pragma once

class GPUPerceptron
{
public:

    float* d_weights;
    float* d_biases;
    float* d_outputs;
    float* d_input;

    GPUPerceptron();

    ~GPUPerceptron();

    void forward(
        const float* input,
        float* outputs
    );

    void train(
        const float* input,
        int label,
        float learningRate
    );
};

#endif //PERCEPTRON_IMAGENES_PERCEPTRON_CUH
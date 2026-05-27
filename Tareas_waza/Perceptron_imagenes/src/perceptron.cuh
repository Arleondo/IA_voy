#pragma once

class GPUPerceptron{
public:

    float* d_weights{};
    float* d_biases{};
    float* d_outputs{};
    float* d_input{};

    GPUPerceptron();

    ~GPUPerceptron();

    void forward(const float* input,float* outputs) const;

    void train(const float* input,int label,float learningRate) const;

    void saveModel(const char* filename) const;

    void loadModel(const char* filename) const;
};

#ifndef PERCEPTRON_IMAGENES_PERCEPTRON_H
#define PERCEPTRON_IMAGENES_PERCEPTRON_H

#pragma once

#include <vector>

using namespace std;

class Perceptron
{
public:

    vector<vector<float>> weights;
    vector<float> biases;

    Perceptron();

    vector<float> forward(
        const vector<float>& input
    );

    int predict(
        const vector<float>& input
    );

    void train(
        const vector<float>& input,
        int label,
        float learningRate
    );
};


#endif //PERCEPTRON_IMAGENES_PERCEPTRON_H
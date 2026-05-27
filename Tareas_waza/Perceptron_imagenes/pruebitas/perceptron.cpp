#include "../src/perceptron.h"
#include "../src/perceptron.cuh"

#include <cstdlib>
#include <cmath>

using namespace std;

float sigmoid(float x)
{
    return 1.0f /
           (1.0f + exp(-x));
}

Perceptron::Perceptron()
{
    weights.resize(10);

    for(int j = 0; j < 10; j++)
    {
        weights[j].resize(784);

        for(int i = 0; i < 784; i++)
        {
            weights[j][i] =
                ((float)rand() / RAND_MAX)
                - 0.5f;
        }
    }

    biases.resize(10, 0.0f);
}

vector<float> Perceptron::forward(
    const vector<float>& input
)
{
    vector<float> outputs(10);

    vector<float> flatWeights;

    for(int j = 0; j < 10; j++)
    {
        for(int i = 0; i < 784; i++)
        {
            flatWeights.push_back(
                weights[j][i]
            );
        }
    }

    forwardCUDA(
        (float*)input.data(),
        flatWeights.data(),
        biases.data(),
        outputs.data()
    );

    for(int i = 0; i < 10; i++)
    {
        outputs[i] =
            sigmoid(outputs[i]);
    }

    return outputs;
}

int Perceptron::predict(
    const vector<float>& input
)
{
    vector<float> outputs =
        forward(input);

    int bestIndex = 0;

    for(int i = 1; i < 10; i++)
    {
        if(outputs[i]
           > outputs[bestIndex])
        {
            bestIndex = i;
        }
    }

    return bestIndex;
}

void Perceptron::train(
    const vector<float>& input,
    int label,
    float learningRate
)
{
    vector<float> outputs =
        forward(input);

    for(int j = 0; j < 10; j++)
    {
        float target = 0.0f;

        if(j == label)
        {
            target = 1.0f;
        }

        float error =
            target - outputs[j];

        for(int i = 0; i < 784; i++)
        {
            weights[j][i] +=
                learningRate
                * error
                * input[i];
        }

        biases[j] +=
            learningRate
            * error;
    }
}
#include <iostream>
#include <cmath>

#include "mnist_loader.h"
#include "perceptron.cuh"

using namespace std;

float sigmoid(float x)
{
    return 1.0f /
           (1.0f + exp(-x));
}

char getPixelChar(float pixel){
    if(pixel > 0.8f) return '@';
    if(pixel > 0.6f) return '#';
    if(pixel > 0.4f) return '*';
    if(pixel > 0.2f) return '.';

    return ' ';
}

int argmax(float* outputs)
{
    int best = 0;

    for(int i = 1; i < 10; i++)
    {
        if(outputs[i] > outputs[best])
        {
            best = i;
        }
    }

    return best;
}

int main(){

    const MNIST_Data data = loadMNIST(
        R"(D:\IA_voy\Tareas_waza\Perceptron_imagenes\data\train-images.idx3-ubyte)",
        R"(D:\IA_voy\Tareas_waza\Perceptron_imagenes\data\train-labels.idx1-ubyte)",
        5000
    );

    if(data.images.empty())
    {
        cerr << "Dataset vacio"
             << endl;

        return 1;
    }

    GPUPerceptron p;

    cout << endl;
    cout << "=== ENTRENANDO GPU ==="
         << endl << endl;

    for(int epoch = 0;
        epoch < 10;
        epoch++)
    {
        cout << "Epoch "
             << epoch + 1
             << endl;

        for(int i = 0;
            i < data.images.size();
            i++)
        {
            p.train(
                data.images[i].data(),
                data.labels[i],
                0.01f
            );
        }
    }

    cout << endl;
    cout << "=== TEST ==="
         << endl << endl;

    for(int index = 0;
        index < 10;
        index++)
    {
        float outputs[10];

        p.forward(
            data.images[index].data(),
            outputs
        );

        for(int i = 0; i < 10; i++)
        {
            outputs[i] =
                sigmoid(outputs[i]);
        }

        int prediction =
            argmax(outputs);

        cout << "Label real: "
             << data.labels[index]
             << endl;

        cout << "Prediccion: "
             << prediction
             << endl << endl;

        for(int r = 0; r < 28; r++)
        {
            for(int c = 0; c < 28; c++)
            {
                float pixel =
                    data.images[index]
                    [r * 28 + c];

                cout
                    << getPixelChar(pixel);
            }

            cout << endl;
        }

        cout << endl;
    }

    return 0;
}

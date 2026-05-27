#include <iostream>
#include <algorithm>
#include <random>

#include "mnist_loader.h"
#include "perceptron.cuh"


using namespace std;

char getPixelChar(float pixel){
    if(pixel > 0.8f) return '@';
    if(pixel > 0.6f) return '#';
    if(pixel > 0.4f) return '*';
    if(pixel > 0.2f) return '.';

    return ' ';
}

int argmax(const float* outputs){
    int best = 0;

    for(int i = 1; i < 10; i++){
        if(outputs[i] > outputs[best]){
            best = i;
        }
    }
    return best;
}

void showImage(const vector<float>& image){
    for(int r = 0; r < 28; r++){
        for(int c = 0; c < 28; c++){
            float pixel = image[r * 28 + c];
            cout << getPixelChar(pixel);
        }
        cout << endl;
    }
}

int main(){

    GPUPerceptron p;
    int option;

    cout << "\n=== MENU ===\n";

    cout << "1. Entrenar modelo\n";

    cout << "2. Cargar modelo\n\n";

    cout << "Opcion: ";
    cin >> option;
    cout << endl;

    if(option == 1){
        int epochs;
        float learningRate;

        cout << "Epochs: ";
        cin >> epochs;

        cout << "Learning rate: ";
        cin >> learningRate;

        MNIST_Data trainData =
            loadMNIST(
                "data/train-images.idx3-ubyte",
                "data/train-labels.idx1-ubyte",
                60000
            );

        cout << "\n---- Entrenando ----\n\n";

        vector<int> indices(trainData.images.size());

        for(int i = 0; i < indices.size(); i++){
            indices[i] = i;
        }

        random_device rd;
        mt19937 g(rd());

        for(int epoch = 0; epoch < epochs; epoch++){

            cout << "Epoch " << epoch + 1 << endl;

            shuffle( indices.begin(), indices.end(), g );

            for(int idx = 0; idx < indices.size(); idx++){
                int i = indices[idx];

                p.train( trainData.images[i].data(), trainData.labels[i], learningRate );
            }
        }

        p.saveModel( "mnist_model.bin" );

        cout << "\nModelo guardado.\n";
    }
    else if(option == 2){
        p.loadModel( "mnist_model.bin" );

        cout << "\nModelo cargado.\n";
    }

    int datasetOption;

    cout << "\n---- DATASET ----\n";

    cout << "1. Train dataset\n";

    cout << "2. Test dataset\n";

    cout << "\nOpcion: ";
    cin >> datasetOption;

    MNIST_Data data;

    if(datasetOption == 1){
        data = loadMNIST(
            "data/train-images.idx3-ubyte",
            "data/train-labels.idx1-ubyte",
            100
        );
    }
    else{
        data = loadMNIST(
            "data/t10k-images.idx3-ubyte",
            "data/t10k-labels.idx1-ubyte",
            100
        );
    }

    cout << "\n=== TEST ===\n\n";

    int correct = 0;

    for(int index = 0; index < data.images.size(); index++) {
        float outputs[10];

        p.forward(data.images[index].data(), outputs);

        int prediction = argmax(outputs);

        if(prediction == data.labels[index]){ correct++; }

        cout << "Imagen" << index << endl;

        cout << "Label real: " << data.labels[index] << endl;

        cout << "Prediccion: " << prediction << "\n\n";

        showImage(data.images[index]);

        cout << endl;
    }

    const float accuracy = static_cast<float>(correct) / data.images.size() * 100.0f;

    cout << "\nAccuracy: " << accuracy << "%\n";

    return 0;
}

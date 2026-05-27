// =========================
// mnist_loader.cpp
// =========================

#include "mnist_loader.h"

#include <fstream>
#include <iostream>

using namespace std;

int reverseInt(int i){
    unsigned char c1, c2, c3, c4;

    c1 = i & 255;
    c2 = (i >> 8) & 255;
    c3 = (i >> 16) & 255;
    c4 = (i >> 24) & 255;

    return (static_cast<int>(c1) << 24) + (static_cast<int>(c2) << 16)+ (static_cast<int>(c3) << 8)+ c4;
}

MNIST_Data loadMNIST(const string& imagesPath,const string& labelsPath,int limit){
    ifstream imageFile(imagesPath, ios::binary);
    ifstream labelFile(labelsPath, ios::binary);

    if(!imageFile)
    {
        cerr << "No se pudo abrir images file" << endl;
        exit(1);
    }

    if(!labelFile)
    {
        cerr << "No se pudo abrir labels file" << endl;
        exit(1);
    }

    int magic = 0;
    int numImages = 0;
    int rows = 0;
    int cols = 0;

    imageFile.read((char*)&magic, 4);
    magic = reverseInt(magic);

    imageFile.read((char*)&numImages, 4);
    numImages = reverseInt(numImages);

    imageFile.read((char*)&rows, 4);
    rows = reverseInt(rows);

    imageFile.read((char*)&cols, 4);
    cols = reverseInt(cols);

    int magicLabels = 0;
    int numLabels = 0;

    labelFile.read((char*)&magicLabels, 4);
    magicLabels = reverseInt(magicLabels);

    labelFile.read((char*)&numLabels, 4);
    numLabels = reverseInt(numLabels);

    cout << "Numero de imagenes: " << numImages << endl;
    cout << "Rows: " << rows << endl;
    cout << "Cols: " << cols << endl;

    MNIST_Data data;

    for(int n = 0; n < limit; n++){

        vector<float> image(rows * cols);

        for(int i = 0; i < rows * cols; i++){

            unsigned char temp = 0;
            imageFile.read((char*)&temp, 1);
            image[i] = temp / 255.0f;
        }

        unsigned char label = 0;
        labelFile.read((char*)&label, 1);

        data.images.push_back(image);
        data.labels.push_back((int)label);
    }

    cout << "Imagenes cargadas: " << data.images.size() << endl;

    return data;
}
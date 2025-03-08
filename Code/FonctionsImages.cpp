#include <iostream>
#include "Image.h"

using namespace std;

void SLICC(int argc, char *argv[]) {
    if (argc != 3) {
        cout << "Usage: " << argv[0] << " NomImageIn.ppm NomImageOut.ppm " << endl;
        exit(1);
    }

    string inputFilename = argv[1];
    string outputFilename = argv[2];

    Image img(inputFilename, Image::PPM);
    img.read();
    Image imgLAB = img.RGBtoLAB();
    imgLAB.write(outputFilename);
    int k = 5000;
    int m = 20;
    int N = img.getSize();
    imgLAB.SLICC(k, m, N);
    imgLAB.write(outputFilename);
}
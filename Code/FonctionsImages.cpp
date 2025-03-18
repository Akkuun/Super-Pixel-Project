#include <iostream>
#include "Image.h"

using namespace std;

void SLICC(int argc, char *argv[]) {
    if (argc != 3) {
        cout << "Usage: " << argv[0] << " NomImageIn.ppm NomImageOut.ppm" << endl;
        exit(1);
    }

    string inputFilename = argv[1];
    string outputFilename = argv[2];


    //récupération image PPM
    Image img(inputFilename, Image::PPM);
    img.read();
    //conversion en LAB
    Image imgLAB = img.RGBtoLAB();
    imgLAB.write(outputFilename);
    int k = 40000; // Nombre de clusters
    int m = 60; //résolution spatiale
    int N = img.getSize();
    //SLICC
    imgLAB.SLICC(k, m, N);
    imgLAB.write(outputFilename);
    Image imgOUT = imgLAB.LABtoRGB();
    //écriture de l'image Superpixels
    imgOUT.write(outputFilename);
    cout << "Fin SLICC" << endl;
    cout << img.PSNR(imgOUT) << endl;








}


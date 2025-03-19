#include <iostream>
#include "Image.h"

using namespace std;

void SLICC(int argc, char *argv[]) {
    if (argc != 4) {
        cout << "Usage: " << argv[0] << " NomImageIn.ppm NomImageOut.ppm NomImageOutQuantification.ppm" << endl;
        exit(1);
    }

    string inputFilename = argv[1];
    string outputFilename = argv[2];
    string outputFilenameQuantification = argv[3];

    //récupération image PPM
    Image img(inputFilename, Image::PPM);
    img.read();
    //conversion en LAB
    Image imgLAB = img.RGBtoLAB();
    imgLAB.write(outputFilename);
    int k = 6000; // Nombre de clusters
    int m = 30; //résolution spatiale
    int N = img.getSize();
    //SLICC
    imgLAB.SLICC(k, m, N);
    imgLAB.write(outputFilename);
    Image imgOUT = imgLAB.LABtoRGB();
    //écriture de l'image Superpixels
    imgOUT.write(outputFilename);
    cout << "Fin SLICC" << endl;
    //écriture de l'image compressé spatialement
    Image imgOUTLAB = imgOUT.RGBtoLAB();
    //Image imgCompressee = imgOUTLAB.compressionParQuantification(6);
    //Image imgCompresseeRGB = imgCompressee.LABtoRGB();
    //imgCompresseeRGB.write(outputFilenameQuantification);

//    //calcul PSNR
    //cout << "PSNR : " << img.PSNR(imgCompresseeRGB)  << " dB" << endl;

    imgOUT.genererCourbeDistortion(img, outputFilename);



}


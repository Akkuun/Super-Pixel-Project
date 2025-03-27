#include <iostream>
#include "Image.h"

using namespace std;

void SLICC(int argc, char *argv[]) {
    if (argc != 4) {
        cout << "Usage: " << argv[0] << " NomImageIn.ppm NomImageOutSLICC.ppm GénérerImageContour? (0/1)" << endl;
        exit(1);
    }

    string inputFilename = argv[1];
    string outputFilename = argv[2];
    bool contour = atoi(argv[3]);

    //récupération image PPM
    Image img(inputFilename, Image::PPM);
    img.read();
    //conversion en LAB
    Image imgLAB = img.RGBtoLAB();
    imgLAB.write(outputFilename);
    int k = 6000; // Nombre de clusters
    int m = 30; //résolution spatiale
    int N = img.getSize();

    // Courbe PSNR
    cout << "Debut Courbe PSNR" << endl;
    img.genererCourbePSNR(imgLAB,img,k, 10, 50, N);
    cout << "Fin Courbe PSNR" << endl;

    //SLICC
    imgLAB.SLICC(k, m, N , contour);
    imgLAB.write(outputFilename);
    Image imgOUT = imgLAB.LABtoRGB();
    //écriture de l'image Superpixels
    imgOUT.write(outputFilename);
    cout << "Fin SLICC" << endl;
    Image imgOUTLAB = imgOUT.RGBtoLAB();
    //création de la courbe de distortion pour afficher le PSNR en fonction de nBit lors de la compression par quantification d'espace de chrominnance
    imgOUT.genererCourbeDistortion(imgOUTLAB, outputFilename,img);

}
#include <iostream>
#include "Image.h"

using namespace std;

void SLICC(int argc, char *argv[]) {
    if (argc != 5) {
        cout << "Usage: " << argv[0]
             << " NomImageIn.ppm NomImageOutSLICC.ppm GénérerImageContour? (0/1) GénérerImageCompressé?( 0/1) " << endl;
        exit(1);
    }

    string inputFilename = argv[1];
    string outputFilename = argv[2];
    bool contour = atoi(argv[3]);
    string outputFilenameTurboPixel = outputFilename.substr(0, outputFilename.find_last_of('.')) + "_TurboPIXEL.ppm";
    bool compress = atoi(argv[4]);
    //récupération image PPM
    Image img(inputFilename, Image::PPM);
    img.read();
    //conversion en LAB
    Image imgLAB = img.RGBtoLAB();
    imgLAB.write(outputFilename);
    int k = 5000; // Nombre de clusters
    int m = 60; //résolution spatiale
    int N = img.getSize();

//    // Courbe PSNR
//    cout << "Debut Courbe PSNR" << endl;
//
//    cout << "Fin Courbe PSNR" << endl;
//
//    //SLICC
//    imgLAB.SLICC(k, m, N, contour);
//    imgLAB.write(outputFilename);
//    Image imgOUT = imgLAB.LABtoRGB();
//    //écriture de l'image Superpixels
//    imgOUT.write(outputFilename);
//    cout << "Fin SLICC" << endl;
//    Image imgOUTLAB = imgOUT.RGBtoLAB();
//    //création de la courbe de distortion pour afficher le PSNR en fonction de nBit lors de la compression par quantification d'espace de chrominnance
//    if (compress) {
//        img.genererCourbePSNR(imgLAB,img,k, 10, 50, N);
//
//    }

    // Perform Mean Shift Segmentation
    float spatial_radius = 10.0f;
    float color_radius = 10.0f;
    int max_iterations = 100;
    Image segmentedImg = imgLAB.MeanShiftSegmentation(spatial_radius, color_radius, max_iterations);

    // Convert the segmented image back to RGB
    Image resultImg = segmentedImg.LABtoRGB();

    // Save the segmented image
    resultImg.write(outputFilename);

}



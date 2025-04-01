#include <iostream>
#include "Image.h"

using namespace std;

void SLICC(int argc, char *argv[]) {
    if (argc != 7) {
        cout << "Usage: " << argv[0]
             << " NomImageIn.ppm SLICC? (0/1) GénérerImageContour SLICC? (0/1) GénérerImageCompressé? (0/1) Mean Shift Clustering? (0/1) GénérerImageContour Mean Shift Clustering? (0/1)"
             << endl;
        exit(1);
    }

    string nomFichierEntree = argv[1]; // Nom de l'image d'entrée
    //créer une string qui met SLIC juste avant le .
    bool genererSLICC = atoi(argv[2]); // SLICC ou pas
    bool contourSLICC = atoi(argv[3]); // Générer image contour SLICC ou pas
    bool compressSLICC = atoi(argv[4]); // Générer image compressée SLICC ou pas
    bool genererMeanShift = atoi(argv[5]); // Générer image contour Mean Shift ou pas
    bool contourMeanShift = atoi(argv[6]); // Générer image contour Mean Shift ou pas



    //récupération image PPM
    Image img(nomFichierEntree, Image::PPM);
    img.read();

    if (genererSLICC) {
        Image imgLAB = img.RGBtoLAB();
        imgLAB.write(nomFichierEntree);
        int k = 700 ; // Nombre de clusters
        int m = 55; //résolution spatiale
        int N = img.getSize();
        imgLAB.SLICC(k, m, N, contourSLICC);
        Image imgOUT = imgLAB.LABtoRGB();
        //écriture de l'image Superpixels
        //ajoute dans la string SLIC juste avant le . le nombre de clusters et la résolution spatiale
        string nomFichierSortieSLICC =
                nomFichierEntree.substr(0, nomFichierEntree.find_last_of('.')) + "_SLICC_" + to_string(k) + "_" +
                to_string(m) + ".ppm";
        imgOUT.write(nomFichierSortieSLICC);

        if (compressSLICC) {
            Image imgOUTLAB = imgOUT.RGBtoLAB();
            //création de la courbe de distortion pour afficher le PSNR en fonction de nBit lors de la compression par quantification d'espace de chrominnance
            img.genererCourbePSNR(imgLAB, img, k, 10, 50, N);
        }
    }


    if (genererMeanShift) {
        float spatial_radius = 10.0f;
        float color_radius = 10.0f;
        int max_iterations = 100;

        Image imgLAB = img.RGBtoLAB();
        Image segmentedImg = imgLAB.MeanShiftSegmentation(spatial_radius, color_radius, max_iterations,contourMeanShift);

        // Convert the segmented image back to RGB
        Image resultImg = segmentedImg.LABtoRGB();

        // Save the segmented image
        string nomFichierSortieMean = nomFichierEntree.substr(0, nomFichierEntree.find_last_of('.')) + "_MeanShift.ppm";
        resultImg.write(nomFichierSortieMean);
    }




}



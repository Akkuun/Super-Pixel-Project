#include <iostream>
#include "Image.h"

using namespace std;

void SLICC(int argc, char *argv[]) {
    if (argc != 4) {
        cout << "Usage: " << argv[0] << " NomImageIn.ppm NomImageOut.ppm  NomImageOutPalette.ppm" << endl;
        exit(1);
    }

    string inputFilename = argv[1];
    string outputFilename = argv[2];
    string outputFilenamePalette = argv[3];

    Image img(inputFilename, Image::PPM);
    img.read();
    Image imgLAB = img.RGBtoLAB();
    imgLAB.write(outputFilename);
    int k = 40000; // Nombre de clusters
    int m = 60; //résolution spatiale
    int N = img.getSize();
    imgLAB.SLICC(k, m, N);
    imgLAB.write(outputFilename);
    Image imgOUT=imgLAB.LABtoRGB();
    imgOUT.write(outputFilename);
    cout << "Fin SLICC" << endl;
    cout << img.PSNR(imgOUT) << endl;
    cout << "Transformation en palette" << endl;
    Image SLICC_Compressed = imgOUT.compressionCreationPalette();
    SLICC_Compressed.write(outputFilenamePalette);

}

/*
 * #include "Image.h"
#include <iostream>
#include <string>

using namespace std;

int main(int argc, char *argv[]) {
    if (argc != 3) {
        cout << "Usage: " << argv[0] << " NomImageIn.ppm NomImageOut.ppm " << endl;
        return 1;
    }

    string inputFilename = argv[1];
    string outputFilename = argv[2];

    // Étape 1 : Appliquer la transformation en super-pixels
    Image img(inputFilename, Image::PPM);
    img.read();
    Image imgLAB = img.RGBtoLAB();
    int k = 12000; // Nombre de clusters
    int m = 20; // Résolution spatiale
    int N = img.getSize();
    imgLAB.SLICC(k, m, N);

    // Étape 2 : Appliquer une compression avec pertes (par exemple, en utilisant une bibliothèque de compression JPEG)
    // Note: Vous devez utiliser une bibliothèque de compression JPEG pour cette étape.
    // imgLAB.write("temp.ppm");
    // system("convert temp.ppm -quality 75 compressed.jpg");
    // system("convert compressed.jpg compressed.ppm");
    // Image imgCompressed("compressed.ppm", Image::PPM);
    // imgCompressed.read();

    // Étape 3 : Mesurer le PSNR
    Image imgCompressed = imgLAB.LABtoRGB(); // Simuler la compression pour cet exemple
    float psnr = img.PSNR(imgCompressed);
    cout << "PSNR: " << psnr << endl;

    // Écrire l'image compressée
    imgCompressed.write(outputFilename);

    return 0;
}
 */
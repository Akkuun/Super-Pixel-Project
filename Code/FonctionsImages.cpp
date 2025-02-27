#include <iostream>
#include "Image.h"

using namespace std;

void seuil(int argc, char *argv[]) {
    if (argc != 4) {
        cout << "Usage: " << argv[0] << " NomImageIn.pgm NomImageOut.pgm Seuil" << endl;
        exit(1);
    }

    string inputFilename = argv[1];
    string outputFilename = argv[2];
    int threshold = stoi(argv[3]);

    Image img(inputFilename, Image::PGM);
    img.read();
    img.appliquerSeuil(threshold);
    img.write(outputFilename);
}
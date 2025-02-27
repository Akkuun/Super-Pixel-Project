#ifndef IMAGE_H
#define IMAGE_H

#include <string>
#include "image_ppm.h"

class Image {
public:
    enum Format { PGM, PPM , LAB};

    Image(const std::string &filename, Format format);
    ~Image();
    //fonctions de base
    void read();
    void write(const std::string &filename);
    int getWidth() const { return width; }
    int getHeight() const { return height; }
    OCTET* getData() const { return data; }
    OCTET* copyData() const;
    OCTET *copyData(const OCTET *data, int size);
    OCTET* createData();
    //fonctions de traitement
    void appliquerSeuil(int seuil);
    void SLICC(int k, int m);
    Image RGBtoLAB();

private:
    std::string filename;
    Format format;
    int width, height, size;
    OCTET *data;
};

#endif // IMAGE_H
#ifndef IMAGE_H
#define IMAGE_H

#include <string>
#include "image_ppm.h"

class Image {
public:
    enum Format { PGM, PPM };

    Image(const std::string &filename, Format format);
    ~Image();
    //fonctions de base
    void read();
    void write(const std::string &filename);
    int getWidth() const { return width; }
    int getHeight() const { return height; }
    OCTET* getData() const { return data; }
    //fonctions de traitement
    void appliquerSeuil(int seuil);
    void SLIC(int k, int m);

private:
    std::string filename;
    Format format;
    int width, height, size;
    OCTET *data;
};

#endif // IMAGE_H
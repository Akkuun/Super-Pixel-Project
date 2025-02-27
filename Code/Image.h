#ifndef IMAGE_H
#define IMAGE_H

#include <string>
#include "image_ppm.h"

class Image {
public:
    enum Format { PGM, PPM };

    Image(const std::string &filename, Format format);
    ~Image();

    void read();
    void write(const std::string &filename);
    void appliquerSeuil(int seuil);

    int getWidth() const { return width; }
    int getHeight() const { return height; }
    OCTET* getData() const { return data; }

private:
    std::string filename;
    Format format;
    int width, height, size;
    OCTET *data;
};

#endif // IMAGE_H
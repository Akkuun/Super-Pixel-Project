#include "Image.h"
#include "image_ppm.h"
#include <iostream>
#include <vector>


using namespace std;

struct ClusterCenter {
    float Lk, ak, bk;
    int xk, yk;
};

Image::Image(const std::string &filename, Format format)
        : filename(filename), format(format), width(0), height(0), size(0), data(nullptr) {}

Image::~Image() {
    if (data) {
        free(data);
    }
}

void Image::read() {
    if (format == PGM) {
        lire_nb_lignes_colonnes_image_pgm(const_cast<char *>(filename.c_str()), &height, &width);
        size = width * height;
        allocation_tableau(data, OCTET, size);
        lire_image_pgm(const_cast<char *>(filename.c_str()), data, size);
    } else if (format == PPM) {
        lire_nb_lignes_colonnes_image_ppm(const_cast<char *>(filename.c_str()), &height, &width);
        size = width * height * 3;
        allocation_tableau(data, OCTET, size);
        lire_image_ppm(const_cast<char *>(filename.c_str()), data, size);
    }
}

void Image::write(const std::string &filename) {
    if (format == PGM) {
        ecrire_image_pgm(const_cast<char *>(filename.c_str()), data, height, width);
    } else if (format == PPM) {
        ecrire_image_ppm(const_cast<char *>(filename.c_str()), data, height, width);
    }
}

void Image::appliquerSeuil(int seuil) {
    if (format == PGM) {
        for (int i = 0; i < size; i++) {
            data[i] = (data[i] < seuil) ? 0 : 255;
        }
    } else if (format == PPM) {
        for (int i = 0; i < size; i += 3) {
            data[i] = (data[i] < seuil) ? 0 : 255;
            data[i + 1] = (data[i + 1] < seuil) ? 0 : 255;
            data[i + 2] = (data[i + 2] < seuil) ? 0 : 255;
        }
    }
}

void Image::SLIC(int k, int m) {
    //Initialisation
    //Initialisation des clusters de centres Ck = [Lk, ak,bk,xk,yk]^T

    std::vector<ClusterCenter> clusters(k);

    // Initialize the clusters (example initialization)
    for (int i = 0; i < k; ++i) {
        clusters[i].Lk = 0.0f; // Replace with actual initialization
        clusters[i].ak = 0.0f; // Replace with actual initialization
        clusters[i].bk = 0.0f; // Replace with actual initialization
        clusters[i].xk = 0;    // Replace with actual initialization
        clusters[i].yk = 0;    // Replace with actual initialization
    }



}
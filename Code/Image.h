#ifndef IMAGE_H
#define IMAGE_H

#include <string>
#include "image_ppm.h"
#include <iostream>
#include <vector>
#include <cstring>
#include <cmath>

using namespace std;

struct ClusterCenter {
    float Lk, ak, bk;
    int xk, yk;
};

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
    int getSize() const { return width*height; }
    OCTET* getData() const { return data; }
    OCTET* copyData() const;
    OCTET *copyData(const OCTET *data, int size);
    OCTET* createData();
    int getIndice(int i,int j,int nH,int nW);

    //fonctions de traitement
    void appliquerSeuil(int seuil);
    void SLICC(int &k, int &m, int &N);
    Image RGBtoLAB();
    float calculerDistanceCouleur(ClusterCenter &cluster ,int &i,int &j);
    float calculerDistanceSpatiale(ClusterCenter &cluster, int &i, int &j);
    void calculerNouveauCentre(vector<ClusterCenter> &clusters, vector<int> &labels, int cluster, float &newL, float &newa,
                               float &newb, float &newx, float &newy, float &newDeltaCk);
    int floodFill(int x, int y, vector<int> &newLabels, int &label,vector<int> &labels);
    int affecterSuperPixelVoisin(int x, int y, vector<int> &newLabels, vector<int> &listeComposantesConnexes, vector<int> &labels,
                                 int &tailleSeuilMinimal, vector<ClusterCenter> &clusters);

private:
    std::string filename;
    Format format;
    int width, height, size;
    OCTET *data;
};

#endif // IMAGE_H
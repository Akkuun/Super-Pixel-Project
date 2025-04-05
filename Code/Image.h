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
    unsigned long int xk, yk;
};

struct Point {
    float x, y, L, a, b;
};


class Image {
public:
    enum Format {
        PGM, PPM, LAB
    };

    Image(const std::string &filename, Format format);
    ~Image();
    //fonctions de base
    void read();
    void write(const std::string &filename);
    int getWidth() const { return width; }
    int getHeight() const { return height; }
    int getSize() const { return width * height; }
    OCTET *getData() const { return data; }
    OCTET *copyData() const;
    OCTET *copyData(const OCTET *data, int size);
    OCTET *createData();
    int getIndice(int i, int j, int nH, int nW);
    float PSNR(Image &imageTraitee);

    //fonctions de traitement
    void SLICC(int &k, int &m, int &N, bool &contour);
    Image RGBtoLAB();
    Image LABtoRGB();
    float calculerDistanceCouleur(ClusterCenter &cluster, int &i, int &j);
    float calculerDistanceSpatiale(ClusterCenter &cluster, int &i, int &j);
    void calculerNouveauCentre(vector <ClusterCenter> &clusters, vector<int> &labels, int cluster, float &newL,float &newa,float &newb, float &newx, float &newy, float &newDeltaCk);
    int floodFill(int x, int y, vector<int> &newLabels, int &label, vector<int> &labels);
    int affecterSuperPixelVoisin(int x, int y, vector<int> &newLabels, vector<int> &listeComposantesConnexes,
                                 vector<int> &labels,
                                 int &tailleSeuilMinimal, vector <ClusterCenter> &clusters);
    float calculerTauxCompression(Image &imageCompressee);
    Image compressionParQuantification(int nBit);
    float calculerEntropieImage();
    void genererCourbeDistortion(Image &imgSLICC, const string &outputFilenameBase, Image &imgDeBase);
    void highlightContours(const vector<int> &labels);
    void highlightContoursPoints(const vector<Point> &points);
    Image MeanShiftSegmentation(float spatial_radius, float color_radius, int max_iterations, bool contour);
    int calculer_norme_gradian(int i, int j, Image &imgGRIS);
    void genererCourbePSNR(Image &imgLAB, Image &imgDeBase, int minK, int maxK, int minM, int maxM, int N);
    void compressionPallette(Image &imgSuperPixel, const string &outputFilenameBase);
    void kmean(OCTET* ImgIn, OCTET* ImgOUT, vector<vector<int>> centroids);

private:
    string filename;
    Format format;
    int width, height, size;
    OCTET *data;

};

#endif // IMAGE_H
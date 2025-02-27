#include "Image.h"
#include "image_ppm.h"
#include <iostream>
#include <vector>
#include <cstring>

using namespace std;
/**
 * \brief Structure représentant un cluster.
 * \details Un cluster est défini par un centre (Lk, ak, bk) et une position (xk, yk).
 * \details Cette structure est utilisée pour l'algorithme SLIC.
 * \param Lk Composante L du centre du cluster.
 * \param ak Composante a du centre du cluster.
 * \param bk Composante b du centre du cluster.
 * \param xk Position x du centre du cluster.
 * \param yk Position y du centre du cluster.
 */
struct ClusterCenter {
    float Lk, ak, bk;
    int xk, yk;
};

/**
 * \brief Constructeur de la classe Image.
 * \param filename Nom du fichier image.
 * \param format Format de l'image (PGM ou PPM).
 */
Image::Image(const std::string &filename, Format format)
        : filename(filename), format(format), width(0), height(0), size(0), data(nullptr) {}

/**
 * \brief Destructeur de la classe Image.
 * \details Libère la mémoire allouée pour les données de l'image.
 */
Image::~Image() {
    if (data) {
        free(data);
    }
}

/**
 * \brief Lit les données de l'image depuis un fichier.
 * \details Les données de l'image sont stockées dans le tableau data.
 */
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

/**
 * \brief Écrit les données de l'image dans un fichier.
 * \param filename Nom du fichier image.
 */
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

Image Image::RGBtoLAB() {
    //pour l'instant retourne une copie de l'image
    Image img(filename, format);
    img.width = width;
    img.height = height;
    img.size = size;
    img.data = (OCTET *) malloc(size * sizeof(OCTET));
    memcpy(img.data, data, size * sizeof(OCTET));
    return img;

}



/**
 * \brief Applique l'algorithme SLICC sur l'image.
 * \param k Nombre de clusters.
 * \param m Paramètre de compacité.
 */
void Image::SLICC(int k, int m) {

    // PHASE 1 : Initialisation
    //1.1 Convertir l’image RGB en CIELab.
    //1.2 Définir le nombre de superpixels souhaité et calculer le pas de grille \( S = \sqrt{\frac{N}{K}} \).
    //1.3 Placer les centres de clusters Ck sur une grille régulière (avec un léger ajustement pour éviter les bords).
    //1.4 Initialiser la matrice des labels  L(x, y) à -1 et la matrice des distances   D(x, y) à INF


    //1.1 Convertir l’image RGB en CIELab.



    // PHASE 2 : Assignation des pixels aux clusters
    //2.1 Pour chaque centre de cluster \( C_k \) :
    //2.2 Parcourir les pixels dans une fenêtre locale de taille \( 2S \times 2S \).
    //2.3 Pour chaque pixel \( P(x, y) \) dans cette région :
    //- Calculer la **distance couleur** \( d_{lab} = || C_k^{lab} - P^{lab} || \).
    //- Calculer la **distance spatiale** \( d_{xy} = || C_k^{xy} - P^{xy} || \).
    //- Calculer la distance totale :
    //D = d_{lab} + \frac{m}{S} \cdot d_{xy}

    //- Si \( D < D(x, y) \), mettre à jour \( D(x, y) \) et assigner \( L(x, y) = k \).

    // PHASE 3 : Mise à jour des centres des superpixels
    //3.1 Pour chaque cluster \( C_k \) :
    //- Calculer le **nouveau centre** comme la moyenne des pixels lui appartenant.
    //3.2 Répéter **PHASE 2 et 3** jusqu'à convergence (ΔCk < seuil).

    // PHASE 4 : Correction de la connectivité (SLICC spécifique)
    //4.1 Parcourir l'image pour détecter les superpixels non connexes :
    //Effectuer un **flood fill** pour identifier les **composantes connexes** de chaque superpixel.
    //- Si une composante est **trop petite**, l’assigner au superpixel voisin le plus proche.
    //4.2 Mettre à jour les labels \( L(x, y) \) après fusion des petits segments.



}
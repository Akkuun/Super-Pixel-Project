#include "Image.h"
#include "image_ppm.h"
#include <iostream>
#include <vector>
#include <cstring>
#include <cmath>

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
 * \details Les données de l'image sont stockées dans le tableau data.
 * \details Si le format de l'image est PPM, les données sont écrites dans une seule image .ppm
 * \details Si le format de l'image est PGM, les données sont écrites dans une seule image .pgm
 * \details Les images .pgm sont nommées en ajoutant "_L", "_A" et "_B" au nom du fichier pour chaque composantes correspondante.
 */
void Image::write(const std::string &filename) {
    if (format == PGM) {
        ecrire_image_pgm(const_cast<char *>(filename.c_str()), data, height, width);
    } else if (format == PPM) {
        ecrire_image_ppm(const_cast<char *>(filename.c_str()), data, height, width);
    } else if (format == LAB) {
        OCTET *dataL = copyData();
        OCTET *dataa = copyData();
        OCTET *datab = copyData();

        for (int i = 0; i < size; i += 3) {
            dataL[i / 3] = data[i];       // Extraction des valeurs L
            dataa[i / 3] = data[i + 1];   // Extraction des valeurs a
            datab[i / 3] = data[i + 2];   // Extraction des valeurs b
        }

        ecrire_image_pgm(const_cast<char *>((filename.substr(0, filename.find_last_of('.')) + "_L.pgm").c_str()), dataL,
                         height, width);
        ecrire_image_pgm(const_cast<char *>((filename.substr(0, filename.find_last_of('.')) + "_A.pgm").c_str()), dataa,
                         height, width);
        ecrire_image_pgm(const_cast<char *>((filename.substr(0, filename.find_last_of('.')) + "_B.pgm").c_str()), datab,
                         height, width);
        free(dataL);
        free(dataa);
        free(datab);
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

/**
 * \brief Copie les données de l'image dans un nouveau tableau.
 * \return Un pointeur vers le nouveau tableau contenant les données copiées.
 */
OCTET *Image::copyData() const {
    OCTET *newData = (OCTET *) malloc(size * sizeof(OCTET));
    if (newData == nullptr) {
        std::cerr << "Erreur d'allocation de mémoire pour la copie des données de l'image." << std::endl;
        exit(EXIT_FAILURE);
    }
    memcpy(newData, data, size * sizeof(OCTET));
    return newData;
}

/**
 * \brief Copie les données de l'image dans un nouveau tableau.
 * \param data Le tableau de données à copier.
 * \param size La taille du tableau de données.
 * \return Un pointeur vers le nouveau tableau contenant les données copiées.
 */
OCTET *Image::copyData(const OCTET *data, int size) {
    OCTET *newData = (OCTET *) malloc(size * sizeof(OCTET));
    if (newData == nullptr) {
        std::cerr << "Erreur d'allocation de mémoire pour la copie des données de l'image." << std::endl;
        exit(EXIT_FAILURE);
    }
    memcpy(newData, data, size * sizeof(OCTET));
    return newData;
}

/**
 * \brief Crée un nouveau tableau pour stocker les données de l'image.
 * \return Un pointeur vers le nouveau tableau alloué.
 */
OCTET *Image::createData() {
    OCTET *newData = (OCTET *) malloc(size * sizeof(OCTET));
    if (newData == nullptr) {
        std::cerr << "Erreur d'allocation de mémoire pour la copie des données de l'image." << std::endl;
        exit(EXIT_FAILURE);
    }
    return newData;
}

/**
 * \brief Convertit l'image RGB en CIELab.
 * \return Une nouvelle image en format CIELab.
 */
Image Image::RGBtoLAB() {
    cout << "Début Conversion RGB -> CIELab" << endl;

    // PHASE 1 : Conversion RGB → XYZ
    float* dataXYZ = new float[size];

    //1.1 Normaliser chaque canal RGB dans [0,1].
    for (int i = 0; i < size; i += 3) {
        float R = data[i] / 255.0;
        float G = data[i + 1] / 255.0;
        float B = data[i + 2] / 255.0;

        //1.2 Appliquer la correction gamma inverse (sRGB → Linéaire)
        R = (R > 0.04045) ? pow((R + 0.055) / 1.055, 2.4) : R / 12.92;
        G = (G > 0.04045) ? pow((G + 0.055) / 1.055, 2.4) : G / 12.92;
        B = (B > 0.04045) ? pow((B + 0.055) / 1.055, 2.4) : B / 12.92;

        //1.3 Conversion RGB -> XYZ
        dataXYZ[i]     = 0.4124564 * R + 0.3575761 * G + 0.1804375 * B;
        dataXYZ[i + 1] = 0.2126729 * R + 0.7151522 * G + 0.0721750 * B;
        dataXYZ[i + 2] = 0.0193339 * R + 0.1191920 * G + 0.9503041 * B;
    }

    // PHASE 2 : Conversion XYZ → CIELab
    OCTET* dataLAB = createData();

    for (int i = 0; i < size; i += 3) {
        // 2.1 Normalisation par les valeurs de référence (D65)
        float X = dataXYZ[i] / 0.95047;
        float Y = dataXYZ[i + 1];
        float Z = dataXYZ[i + 2] / 1.08883;

        // 2.2 Transformation non linéaire
        X = (X > 0.008856) ? pow(X, 1.0 / 3.0) : (7.787 * X) + (16.0 / 116.0);
        Y = (Y > 0.008856) ? pow(Y, 1.0 / 3.0) : (7.787 * Y) + (16.0 / 116.0);
        Z = (Z > 0.008856) ? pow(Z, 1.0 / 3.0) : (7.787 * Z) + (16.0 / 116.0);

        // 2.3 Conversion en CIELab
        float L = (116.0 * Y) - 16.0;
        float a = 500.0 * (X - Y);
        float b = 200.0 * (Y - Z);

        // 2.4 Conversion en OCTET (valeurs entières)
        dataLAB[i]     = static_cast<OCTET>(L * (255.0 / 100.0));  // Normalisation de [0,100] à [0,255]
        dataLAB[i + 1] = static_cast<OCTET>(a + 128);              // Normalisation de [-128,127] à [0,255]
        dataLAB[i + 2] = static_cast<OCTET>(b + 128);
    }

    // Nettoyage mémoire
    delete[] dataXYZ;

    // PHASE 3 : Stockage des valeurs CIELab dans une nouvelle image
    Image img(filename, LAB);
    img.width = width;
    img.height = height;
    img.size = size;
    img.data = copyData(dataLAB, size);

    free(dataLAB);

    cout << "Fin Conversion RGB -> CIELab" << endl;
    return img;
}


/**
 * \brief Applique l'algorithme SLICC sur l'image.
 * \param k Nombre de clusters -> plus l'image est grande, plus on augmente.
 * \param m Paramètre de compacité.
 * \param N Nombre de superpixels souhaité -> plus l'image est grande, plus on augmente.
 */
void Image::SLICC(int k, int m, int N) {

    // PHASE 1 : Initialisation
    //1.1 Convertir l’image RGB en CIELab. -> ok
    //1.2 Définir le nombre de superpixels souhaité et calculer le pas de grille avec S = sqrt(N / k).
    float S = sqrt(static_cast<float>(N) / k);


    //1.3 Placer les centres de clusters Ck sur une grille régulière (avec un léger ajustement pour éviter les bords).
    vector<ClusterCenter> clusters(k);
    //pour chaque cluster, on prend un pixel de l'image comme centre
    for (int clusterActuel = 0; clusterActuel < k; clusterActuel++) {
        int x = static_cast<int>((S / 2) + (i % static_cast<int>(width / S)) * S);
        int y = static_cast<int>((S / 2) + (i / static_cast<int>(width / S)) * S);
        clusters[clusterActuel].xk = x;
        clusters[clusterActuel].yk = y;
        clusters[clusterActuel].Lk = data[(y * width + x) * 3];
        clusters[clusterActuel].ak = data[(y * width + x) * 3 + 1];
        clusters[clusterActuel].bk = data[(y * width + x) * 3 + 2];
    }

    //1.4 Initialiser la matrice des labels  L(x, y) à -1 et la matrice des distances   D(x, y) à INF
    vector<int> labels(size, -1);
    vector<float> distances(size, INFINITY);

    //1.5 Initialiser le seuil de convergence ΔCk
    float seuil = 1.0;

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
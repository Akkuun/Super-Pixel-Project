#include "Image.h"
#include "image_ppm.h"
#include <iostream>
#include <vector>
#include <cstring>
#include <cmath>
#include <queue>
#include <omp.h>
#include <climits>
#include <fstream>
#include <cfloat>
#include <algorithm>

using namespace std;


/**
 * \brief Constructeur de la classe Image.
 * \param filename Nom du fichier image.
 * \param format Format de l'image (PGM ou PPM).
 * \return Une nouvelle instance de la classe Image.
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

/**
 * \brief Calcule l'indice d'un pixel dans un tableau 1D.
 * \param i Coordonnée y du pixel.
 * \param j Coordonnée x du pixel.
 * \param nH Hauteur de l'image.
 * \param nW Largeur de l'image.
 * \return L'indice du pixel dans le tableau 1D.
 */
int Image::getIndice(int i, int j, int nH, int nW) {
    i = std::min(std::max(i, 0), nH - 1);
    j = std::min(std::max(j, 0), nW - 1);
    return i * nW + j;
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
    if (size <= 0 || static_cast<unsigned int>(size) > INT_MAX / sizeof(OCTET)) {
        std::cerr << "Erreur: taille de l'image invalide ou trop grande pour l'allocation de mémoire." << std::endl;
        exit(EXIT_FAILURE);
    }

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
    float *dataXYZ = new float[size];

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
        dataXYZ[i] = 0.4124564 * R + 0.3575761 * G + 0.1804375 * B;
        dataXYZ[i + 1] = 0.2126729 * R + 0.7151522 * G + 0.0721750 * B;
        dataXYZ[i + 2] = 0.0193339 * R + 0.1191920 * G + 0.9503041 * B;
    }

    // PHASE 2 : Conversion XYZ → CIELab
    OCTET *dataLAB = createData();

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
        dataLAB[i] = static_cast<OCTET>(L * (255.0 / 100.0));  // Normalisation de [0,100] à [0,255]
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


Image Image::LABtoRGB() {
    cout << "Début Conversion CIELab -> RGB" << endl;

    // PHASE 1 : Conversion CIELab → XYZ
    float *dataXYZ = new float[size];

    //1.1 Normaliser chaque canal LAB dans [0,1].
    for (int i = 0; i < size; i += 3) {
        float L = (data[i] / 255.0) * 100.0;  // Convertir L de [0,255] à [0,100]
        float A = ((data[i + 1] - 128.0) / 128.0) * 127.0;  // Convertir A de [0,255] à [-128,127]
        float B = ((data[i + 2] - 128.0) / 128.0) * 127.0;  // Convertir B de [0,255] à [-128,127]

        //1.2 Conversion LAB -> XYZ
        float fy = (L + 16.0) / 116.0;
        float fx = fy + (A / 500.0);
        float fz = fy - (B / 200.0);

        dataXYZ[i] = 95.047 * ((fx > 0.206893) ? pow(fx, 3) : (fx - 16.0 / 116.0) / 7.787);
        dataXYZ[i + 1] = 100.000 * ((fy > 0.206893) ? pow(fy, 3) : (fy - 16.0 / 116.0) / 7.787);
        dataXYZ[i + 2] = 108.883 * ((fz > 0.206893) ? pow(fz, 3) : (fz - 16.0 / 116.0) / 7.787);
    }

    // PHASE 2 : Conversion XYZ → RGB
    OCTET *dataRGB = createData();

    for (int i = 0; i < size; i += 3) {

        float X = dataXYZ[i];
        float Y = dataXYZ[i + 1];
        float Z = dataXYZ[i + 2];

        // 2.1 Conversion XYZ vers linéaire RGB (sRGB)
        double R = 3.2406 * (X / 100.0) - 1.5372 * (Y / 100.0) - 0.4986 * (Z / 100.0);
        double G = -0.9689 * (X / 100.0) + 1.8758 * (Y / 100.0) + 0.0415 * (Z / 100.0);
        double B = 0.0557 * (X / 100.0) - 0.2040 * (Y / 100.0) + 1.0570 * (Z / 100.0);

        // 2.2 Appliquer la correction gamma
        R = (R <= 0.0031308) ? (12.92 * R) : (1.055 * pow(R, 1.0 / 2.4) - 0.055);
        G = (G <= 0.0031308) ? (12.92 * G) : (1.055 * pow(G, 1.0 / 2.4) - 0.055);
        B = (B <= 0.0031308) ? (12.92 * B) : (1.055 * pow(B, 1.0 / 2.4) - 0.055);

        // 2.3 On clamp les résultats afin d'éviter d'avoir des artéfacts
        R = std::max(0.0, std::min(1.0, R));
        G = std::max(0.0, std::min(1.0, G));
        B = std::max(0.0, std::min(1.0, B));

        // 2.4 Mise à l'échelle [0,255]
        dataRGB[i] = static_cast<OCTET>(R * 255.0);
        dataRGB[i + 1] = static_cast<OCTET>(G * 255.0);
        dataRGB[i + 2] = static_cast<OCTET>(B * 255.0);
    }

    // Nettoyage mémoire
    delete[] dataXYZ;

    // PHASE 3 : Stockage des valeurs RGB dans une nouvelle image
    Image img(filename, PPM);
    img.width = width;
    img.height = height;
    img.size = size;
    img.data = copyData(dataRGB, size);

    free(dataRGB);

    cout << "Fin Conversion CIELab -> RGB" << endl;
    return img;
}


/**
 * \brief Calcule la distance entre un pixel et un cluster.
 * \details La distance est donnée par la formule : d_lab = ||C_k^{lab} - P^{lab} ||
 * \param cluster Centre du cluster.
 * \param i Coordonnée x du pixel.
 * \param j Coordonnée y du pixel.
 * \return La distance entre le pixel et le cluster.
 */
float Image::calculerDistanceCouleur(ClusterCenter &cluster, int &i, int &j) {
    // Calcul de la distance couleur
    int index = getIndice(i, j, height, width) * 3;
    float dL = data[index] - cluster.Lk;
    float da = data[index + 1] - cluster.ak;
    float db = data[index + 2]- cluster.bk;
    return sqrt(dL * dL + da * da + db * db);
}

/**
 * \brief Calcule la distance spatiale entre un pixel et un cluster.
 * \details La distance est donnée par la formule : d_xy = ||(x_k, y_k) - (x, y)||
 * \param cluster Centre du cluster.
 * \param i Coordonnée x du pixel.
 * \param j Coordonnée y du pixel.
 * \return La distance spatiale entre le pixel et le cluster.
 */
float Image::calculerDistanceSpatiale(ClusterCenter &cluster, int &i, int &j) {
    // Calcul de la distance spatiale
    float dx = cluster.xk - i;
    float dy = cluster.yk - j;
    return sqrt(dx * dx + dy * dy);
}

/**
 * \brief Calcule le nouveau centre d'un cluster.
 * \details Le nouveau centre est la moyenne des pixels lui appartenant (qui ont le même label).
 * \param clusters Liste des clusters.
 * \param labels Liste des labels des pixels.
 * \param newL Nouvelle composante L du centre.
 * \param newa Nouvelle composante a du centre.
 * \param newb Nouvelle composante b du centre.
 * \param newx Nouvelle position x du centre.
 * \param newy Nouvelle position y du centre.
 * \param newDeltaCk Nouveau deltaCk.
 */
void Image::calculerNouveauCentre(vector <ClusterCenter> &clusters, vector<int> &labels, int cluster, float &newL,
                                  float &newa,
                                  float &newb, float &newx, float &newy, float &newDeltaCk) {
    float sumL = 0, suma = 0, sumb = 0, sumx = 0, sumy = 0;
    int nbPixels = 0;
    for (int i = 0; i < size / 3; i++) {
        if (labels[i] == cluster) {
            sumL += data[i * 3];
            suma += data[i * 3 + 1];
            sumb += data[i * 3 + 2];
            sumx += i % width;
            sumy += i / width;
            nbPixels++;
        }
    }
    if (nbPixels == 0) return;

    newL = sumL / nbPixels;
    newa = suma / nbPixels;
    newb = sumb / nbPixels;
    newx = sumx / nbPixels;
    newy = sumy / nbPixels;
    newDeltaCk = sqrt((newL - clusters[cluster].Lk) * (newL - clusters[cluster].Lk) +
                      (newa - clusters[cluster].ak) * (newa - clusters[cluster].ak) +
                      (newb - clusters[cluster].bk) * (newb - clusters[cluster].bk) +
                      (newx - clusters[cluster].xk) * (newx - clusters[cluster].xk) +
                      (newy - clusters[cluster].yk) * (newy - clusters[cluster].yk));
}

/**
 * \brief Applique un floodFill sur le pixel i,j pour identifier les composantes connexes.
 * @param x la coordonnée x du pixel
 * @param y la coordonnée y du pixel
 * @param newLabels la référence de la liste des nouveaux labels pour classer les composantes connexes
 * @param label la référence du label actuel passé en paramtre de la fonction
 * @param componentSize la référence de la liste des tailles des composantes connexes
 * @return la taille de la composante connexe
 */
int Image::floodFill(int x, int y, vector<int> &newLabels, int &label, vector<int> &labels) {
    queue <pair<int, int>> q;
    q.push({x, y});
    int index = getIndice(x, y, height, width);
    newLabels[index] = label;
    int size = 0;

    while (!q.empty()) {
        auto [cx, cy] = q.front();
        q.pop();
        size++;

        for (int dx = -1; dx <= 1; dx++) {
            for (int dy = -1; dy <= 1; dy++) {
                int nx = cx + dx;
                int ny = cy + dy;
                if (nx >= 0 && nx < height && ny >= 0 && ny < width) {
                    int nIndex = getIndice(nx, ny, height, width);
                    if (newLabels[nIndex] == -1 && labels[nIndex] == labels[index]) {
                        newLabels[nIndex] = label;
                        q.push({nx, ny});
                    }
                }
            }
        }
    }

    return size;
}

/**
 * \brief Affecte un superpixel voisin à un pixel si sa composante connexe est trop petite.
 * @param x la coordonnée x du pixel
 * @param y la coordonnée y du pixel
 * @param newLabels la référence de la liste des nouveaux labels pour classer les composantes connexes
 * @param listeComposantesConnexes la référence de la liste des tailles des composantes connexes
 * @param labels la référence de la liste des labels des pixels
 * @param tailleSeuilMinimal la référence de la taille minimale des composantes connexes
 * @param clusters la référence de la liste des clusters
 * @return le label du superpixel voisin
 */
int Image::affecterSuperPixelVoisin(int x, int y, vector<int> &newLabels, vector<int> &listeComposantesConnexes,
                                    vector<int> &labels, int &tailleSeuilMinimal, vector <ClusterCenter> &clusters) {
    float minDistance = INFINITY;
    int bestLabel = -1;

    for (int dx = -1; dx <= 1; dx++) {
        for (int dy = -1; dy <= 1; dy++) {
            int nx = x + dx;
            int ny = y + dy;
            if (nx >= 0 && nx < height && ny >= 0 && ny < width) {
                int nIndex = getIndice(nx, ny, height, width);
                if (newLabels[nIndex] != newLabels[getIndice(x, y, height, width)]) {
                    float distance = calculerDistanceCouleur(clusters[newLabels[nIndex]], x, y);
                    if (distance < minDistance) {
                        minDistance = distance;
                        bestLabel = newLabels[nIndex];
                    }
                }
            }
        }
    }

    return bestLabel;
}

/**
 * \brief Applique l'algorithme SLICC sur l'image.
 * \param k Nombre de clusters -> plus l'image est grande, plus on augmente.
 * \param m Paramètre de compacité.
 * \param N Nombre de pixels de l'image.
 */
void Image::SLICC(int &k, int &m, int &N, bool &contour) {
    // PHASE 1 : Initialisation
    //1.1 Convertir l’image RGB en CIELab. -> ok
    //1.2 Définir le nombre de superpixels souhaité et calculer le pas de grille avec S = sqrt(N / k).
    // N / k donne le nombre total de super pixel présent dans la grille donc on fait sqrt pour récupérer la longueur du carré de S
    float S = sqrt(static_cast<float>(N) / k);
    cout << "Initialisation des centres de clusters" << endl;
    //1.3 Placer les centres de clusters Ck sur une grille régulière (avec un léger ajustement pour éviter les bords).
    vector <ClusterCenter> clusters;

    int indexCluster=0;
    for (int y = S / 2; y < height; y += S) {
        for (int x = S / 2; x < width; x += S) {
            // x et y sont les coordonnées du centre du cluster au sein de la grille S
            // on place les centres des clusters sur la grille régulière S et on ajoute un pas de S/2 pour les placer au centre

            int index=getIndice(y,x,height,width);
            ClusterCenter clusterActuel;
            clusterActuel.xk=x;
            clusterActuel.yk=y;
            // on met dans le cluster les valeurs de couleur du pixel L a b
            clusterActuel.Lk=data[index*3];
            clusterActuel.ak=data[index*3+1];
            clusterActuel.bk=data[index*3+2];
            clusters.push_back(clusterActuel);
        }
    }

    cout << "fin initilisation des centres de clusters" << endl;

    //1.4 Initialiser la matrice des labels (indices des centres)  L(x, y) à -1 et la matrice des distances   D(x, y) à INF
    // dans notre cas les labels sont les indices des clusters
    vector<int> labels(width * height, -1); // 3 composantes par pixel
    vector<float> distances(width * height, INFINITY);

    //1.5 Initialiser le seuil de convergence ΔCk
    float seuil = 0.1;
    float deltaCk = 0;
    float lastDeltaCk = INFINITY;
    int iteration = 0;
    cout << "debut convergence des centres de clusters" << endl;
    //3.2 Répéter **PHASE 2 et 3** jusqu'à convergence (ΔCk < seuil)
    while (std::abs(lastDeltaCk - deltaCk) > seuil) {
        lastDeltaCk = deltaCk;
        deltaCk = 0;
        iteration += 1;
        //2.1 Pour chaque centre de cluster C_k
#pragma omp parallel for
        for (int i = 0; i < clusters.size(); i++) {
            //2.2 Pour chaque pixel P(x, y) dans une fenêtre 2S x 2S autour de C_k
            for (int dx = -S; dx <= S; dx++) {
                for (int dy = -S; dy <= S; dy++) {
                    //2.3 Calculer la distance D(P, C_k) entre le pixel P et le centre C_k
                    int x = clusters[i].xk + dx;
                    int y = clusters[i].yk + dy;
                    if (x >= 0 && x < width && y >= 0 && y < height) {
                        int index = getIndice(y, x, height, width);
                        float dC = calculerDistanceCouleur(clusters[i], y, x);
                        float dS = sqrt(dx * dx + dy * dy);
                        float D = dC + (m / S) * dS;

                        if (D < distances[index]) {
                            distances[index] = D;
                            labels[index] = i;
                        }
                    }
                }
            }
        }

#pragma omp parallel for reduction(+:deltaCk)
        // PHASE 3 : Mise à jour des centres des superpixels
        for (int cluster = 0; cluster < k; cluster++) {
            float newL, newa, newb, newx, newy, newDeltaCk;
            //3.1 Calculer le nouveau centre C_k en moyennant les pixels lui appartenant.
            calculerNouveauCentre(clusters, labels, cluster, newL, newa, newb, newx, newy, newDeltaCk);

            clusters[cluster].Lk = newL;
            clusters[cluster].ak = newa;
            clusters[cluster].bk = newb;
            clusters[cluster].xk = newx;
            clusters[cluster].yk = newy;
            //3.3 Calculer la variation ΔCk du centre C_k
            deltaCk += newDeltaCk;
        }
        cout << "Iteration: " << iteration << ", deltaCk: " << deltaCk << endl;
    }
    cout << "convergence atteinte" << endl;

//     // PHASE 4 : Correction de la connectivité (SLICC spécifique)
//     vector<int> listeComposantesConnexes;
//     vector<int> newLabels(size / 3, -1);
//     int label = 0;
//     cout << "debut de la correction de la connectivié via floodFill" << endl;
// // 4.1 Parcourir l'image pour détecter les superpixels non connexes
//     for (int i = 0; i < height; i++) {
//         for (int j = 0; j < width; j++) {
//             int index = getIndice(i, j, height, width);
//             if (newLabels[index] == -1) {
//                 // Effectuer un flood fill pour identifier les composantes connexes
//                 listeComposantesConnexes.push_back(floodFill(i, j, newLabels, label, labels));
//                 label++;
//             }
//         }
//     }
//     cout << "fin de la correction de la connectivié via floodFill" << endl;
//     cout << "debut fusion des petis segments" << endl;
// //// 4.2 Fusionner les petits segments TODO A DECOMMENTER POUR FLOOD FILL
//     int tailleSeuilMinimal = size / (3 * k);
//     for (int i = 0; i < height; i++) {
//         for (int j = 0; j < width; j++) {
//             int index = getIndice(i, j, height, width);
//             // Affecter au superpixel voisin le plus proche
//             int bestLabel = affecterSuperPixelVoisin(i, j, newLabels, listeComposantesConnexes, labels, clusters);
//             // si le pixel n'est pas affecté à un superpixel voisin, on le laisse tel quel
//             if (bestLabel != -1) {
//                 newLabels[index] = bestLabel;
//             }
//         }
//     }
//     cout << "fin fusion des petis segments" << endl;
//     // Mettre à jour les labels
//     labels = newLabels;

    // PHASE 5 : Coloration des superpixels
    //5.1 Colorer les superpixels avec la moyenne des couleurs de leurs pixels.
#pragma omp parallel for collapse(2)
    for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++) {
            int index = getIndice(i, j, height, width);
            int cluster = labels[index];
            data[index * 3] = clusters[cluster].Lk;
            data[index * 3 + 1] = clusters[cluster].ak;
            data[index * 3 + 2] = clusters[cluster].bk;
        }
    }
    if (contour) {
        this->highlightContours(labels);
    }


    cout << "Fin SLICC" << endl;
}


/**
 * \brief Calculer le PSNR entre 2 images
 * \param Image1 : Image d'origine sans traitements appliqués
 * \param Image2 : Image d'origine avec traitements appliqués
 * \return Le PSNR entre les 2 images
 */
float Image::PSNR(Image &imageTraitee) {
    float mse = 0.0f;
    for (int i = 0; i < imageTraitee.height; i++) {
        for (int j = 0; j < imageTraitee.width; j++) {
            int index = getIndice(i, j, imageTraitee.height, imageTraitee.width);
            mse += (this->data[index] - imageTraitee.data[index]) * (this->data[index] - imageTraitee.data[index]);
        }
    }
    mse /= (imageTraitee.width * imageTraitee.height);
    return 10 * log10(255 * 255 / mse);
}


float Image::calculerTauxCompression(Image &imageCompressee) {
    float entropieOriginale = this->calculerEntropieImage();
    cout << "Entropie originale : " << entropieOriginale << " bit/pixels" << endl;
    float entropieCompressee = imageCompressee.calculerEntropieImage();
    cout << "Entropie compressee : " << entropieCompressee << " bit/pixels" << endl;
    return entropieCompressee / entropieOriginale;
}

/**
 * \brief crée une image compressée par quantification des couleurs RGB
 * @param nBit
 * @return une image compressée par quantification
 */
Image Image::compressionParQuantification(int nBit) {
    cout << "Début de la compression par quantification" << endl;
    int levels = pow(2, nBit);
    int step = 256 / levels;

    OCTET *dataCompressed = createData();

    for (int i = 0; i < size; i += 3) {
        dataCompressed[i] = data[i]; // Luminance remains unchanged
        dataCompressed[i + 1] = (data[i + 1] / step) * step; // Quantize a
        dataCompressed[i + 2] = (data[i + 2] / step) * step; // Quantize b
    }

    Image imgCompressed(filename, format);
    imgCompressed.width = width;
    imgCompressed.height = height;
    imgCompressed.size = size;
    imgCompressed.data = copyData(dataCompressed, size);

    free(dataCompressed);

    cout << "Fin de la compression par quantification" << endl;
    return imgCompressed;
}

/**
 * \brief Calcule l'entropie de l'image.
 * \return L'entropie de l'image.
 */
float Image::calculerEntropieImage() {
    float entropie = 0.0f;
    int histogramme[256] = {0};

    for (int i = 0; i < size; i++) {
        histogramme[data[i]]++;
    }

    for (int i = 0; i < 256; i++) {
        if (histogramme[i] != 0) {
            float proba = (float) histogramme[i] / size;
            entropie += proba * log2(proba);
        }
    }
    return -entropie;
}
/**
 * \brief Génère la courbe de distorsion pour l'image SLICC.
 * @param imgSLICC
 * @param outputFilenameBase
 * @param imgDeBase
 */
void Image::genererCourbeDistortion(Image &imgSLICC, const string &outputFilenameBase, Image &imgDeBase) {
    ofstream dataFile(outputFilenameBase + ".dat");
    if (!dataFile.is_open()) {
        cerr << "Erreur lors de l'ouverture du fichier de données pour la courbe de distortion." << endl;
        return;
    }
    for (int nBit = 1; nBit <= 8; ++nBit) {
        Image imgCompressee = imgSLICC.compressionParQuantification(nBit);
        Image imgCompresseeRGB = imgCompressee.LABtoRGB();
        //écrire l'image compressée obtenue
        imgCompresseeRGB.write(outputFilenameBase + "_nBit" + to_string(nBit) + ".ppm");
        float psnr = imgDeBase.PSNR(imgCompresseeRGB);
        dataFile << nBit << " " << psnr << endl;
    }

    dataFile.close();

    ofstream gnuplotScript(outputFilenameBase + ".plt");
    if (!gnuplotScript.is_open()) {
        cerr << "Erreur lors de l'ouverture du fichier de script GNUPLOT." << endl;
        return;
    }

    gnuplotScript << "set terminal png size 800,600\n";
    gnuplotScript << "set output '" << outputFilenameBase << ".png'\n";
    gnuplotScript << "set title 'PSNR en fonction de nBit'\n";
    gnuplotScript << "set xlabel 'nBit'\n";
    gnuplotScript << "set ylabel 'PSNR (dB)'\n";
    gnuplotScript << "plot '" << outputFilenameBase << ".dat' using 1:2 with lines title 'PSNR'\n";

    gnuplotScript.close();


    if (system(("gnuplot " + outputFilenameBase + ".plt").c_str()) == -1) {
        cerr << "Erreur lors de l'exécution de gnuplot." << endl;
    }
}

/**
 * \brief Met en évidence les contours des superpixels dans l'image.
 * @param labels Liste des labels des pixels.
 */
void Image::highlightContours(const vector<int> &labels) {
    cout << "debut highligth des contours" << endl;
    // 5.2 Mettre en évidence les contours des superpixels
    // 5.2.1 Initialiser les valeurs des contours
    vector<int> contours(size / 3, 0);
    // 5.2.2 Parcourir l'image pour détecter les contours
    for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++) {
            int index = getIndice(i, j, height, width);
            for (int dx = -1; dx <= 1; dx++) {
                for (int dy = -1; dy <= 1; dy++) {
                    int nx = i + dx;
                    int ny = j + dy;
                    if (nx >= 0 && nx < height && ny >= 0 && ny < width) {
                        int nIndex = getIndice(nx, ny, height, width);
                        if (labels[index] != labels[nIndex]) {
                            contours[index] = 1;
                        }
                    }
                }
            }
        }
    }
    Image imgHighligthContours(filename, PPM);
    imgHighligthContours.width = width;
    imgHighligthContours.height = height;
    imgHighligthContours.size = size;
    imgHighligthContours.data = createData();

    // 5.2.3 Mettre en évidence les contours
    memcpy(imgHighligthContours.data, data, size * sizeof(OCTET));
    for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++) {
            int index = getIndice(i, j, height, width);
            if (contours[index] == 1) {
                imgHighligthContours.data[index * 3] = data[index * 3];
                imgHighligthContours.data[index * 3 + 1] = data[index * 3 + 1];
                imgHighligthContours.data[index * 3 + 2] = data[index * 3 + 2];
            } else {
                imgHighligthContours.data[index * 3] = 255;
                imgHighligthContours.data[index * 3 + 1] = 255;
                imgHighligthContours.data[index * 3 + 2] = 255;
            }
        }
    }
    ecrire_image_ppm(const_cast<char *>((filename.substr(0, filename.find_last_of('.')) + "_contours.ppm").c_str()),
                     imgHighligthContours.data, height, width);
    cout << "fin highligth des contours" << endl;
}

/**
 * \brief Génère une courbe de PSNR sur une image de base avec un K donné et un intervalle de M et  N.
 * @param imgDeBase
 * @param K
 * @param minM
 * @param maxM
 * @param N
 */
void Image::genererCourbePSNR(Image &imgLAB, Image &imgDeBase, int K, int minM, int maxM, int N){
    double PSNR=0.;
    bool contour=false;
    string curbName="./CourbePSNR"+std::to_string(K)+".dat";
    ofstream dataFile(curbName);
    if (!dataFile.is_open()) {
        cerr << "Erreur lors de l'ouverture du fichier de données pour la courbe de distortion." << endl;
        return;
    }
    for (int m=minM; m<=maxM; m+=10){
        //dataFile << m ;
        cout << "m=" << m <<endl;
        Image imgSLICC=imgDeBase.RGBtoLAB();
        imgSLICC.SLICC(K,m,N,contour);
        Image imgComp=imgSLICC.LABtoRGB();
        PSNR=imgDeBase.PSNR(imgComp);
        cout << "PSNR=" << PSNR <<endl;
        dataFile << m << ' ' << PSNR << ' ' << endl;
    }

    ofstream gnuplotScript("./CourbePSNR"+std::to_string(K)+".plt");
    if (!gnuplotScript.is_open()) {
        cerr << "Erreur lors de l'ouverture du fichier de script GNUPLOT." << endl;
        return;
    }
    gnuplotScript << "set terminal png size 800,600\n";
    gnuplotScript << "set output '" << "CourbePSNR" << ".png'\n";
    gnuplotScript << "set title 'PSNR en fonction de K et N'\n";
    gnuplotScript << "set xlabel 'M'\n";
    gnuplotScript << "set ylabel 'PSNR (dB)'\n";
    gnuplotScript << "plot '" << curbName << "' with lines title 'PSNR en Fonction pour k=" << std::to_string(K)<< "'\n";

    gnuplotScript.close();

    system(("gnuplot ./CourbePSNR" + std::to_string(K) + ".plt").c_str());
}
/**
 * \brief Convertit une image RGB en PGM.
 * @return L'image convertie au format PGM.
 */
Image Image::RGBtoPGM() {
    Image img(filename, PGM);
    img.width = width;
    img.height = height;
    img.size = size;
    img.data = createData();
    for (int i = 0; i < size; i += 3) {
        // Calculer la luminance Y
        float Y = 0.299 * data[i] + 0.587 * data[i + 1] + 0.114 * data[i + 2];
        img.data[i / 3] = static_cast<OCTET>(Y);
    }
    return img;
}

/**
 * \brief Effectue la segmentation Super pixel par Mean Shift sur l'image.
 * @param spatial_radius : Rayon spatial pour la recherche de voisins.
 * @param color_radius  : Rayon colorimétrique pour la recherche de voisins.
 * @param max_iterations  : Nombre maximum d'itérations pour la convergence.
 * @return L'image segmentée en format LAB
 */
Image Image::MeanShiftSegmentation(float spatial_radius, float color_radius, int max_iterations) {

    cout << "Début de la segmentation par Mean Shift" << endl;
    vector<Point> points;
    for (int i = 0; i < height; ++i) {
        for (int j = 0; j < width; ++j) {
            int index = getIndice(i, j, height, width) * 3;
            points.push_back({(float)j, (float)i, (float)data[index], (float)data[index + 1], (float)data[index + 2]});
        }
    }

    vector<Point> shifted_points = points;
    float convergence_threshold = 1e-3;
    for (int iter = 0; iter < max_iterations; ++iter) {
        cout << "Iteration: " << iter + 1 << endl;
        bool converged = true;

        // Parcours de chaque pixel
        for (size_t i = 0; i < points.size(); ++i) {
            float sum_x = 0, sum_y = 0, sum_L = 0, sum_a = 0, sum_b = 0, weight_sum = 0;

            // Recherche des pixels voisins dans un rayon spatial donné
            for (size_t j = 0; j < points.size(); ++j) {
                float dx = points[i].x - points[j].x;
                float dy = points[i].y - points[j].y;
                float spatial_dist = dx * dx + dy * dy;

                float dL = points[i].L - points[j].L;
                float da = points[i].a - points[j].a;
                float db = points[i].b - points[j].b;
                float color_dist = dL * dL + da * da + db * db;

                // Vérification si le pixel est dans la fenêtre de voisinage
                if (spatial_dist < spatial_radius * spatial_radius && color_dist < color_radius * color_radius) {
                    float weight = exp(-spatial_dist / (2 * spatial_radius * spatial_radius) -
                                       color_dist / (2 * color_radius * color_radius));
                    sum_x += points[j].x * weight;
                    sum_y += points[j].y * weight;
                    sum_L += points[j].L * weight;
                    sum_a += points[j].a * weight;
                    sum_b += points[j].b * weight;
                    weight_sum += weight;
                }
            }

            // Mise à jour du point
            if (weight_sum > 0) {
                Point new_point = {sum_x / weight_sum, sum_y / weight_sum,
                                   sum_L / weight_sum, sum_a / weight_sum, sum_b / weight_sum};

                // Vérification de la convergence
                float dist = sqrt(pow(new_point.x - shifted_points[i].x, 2) +
                                  pow(new_point.y - shifted_points[i].y, 2) +
                                  pow(new_point.L - shifted_points[i].L, 2) +
                                  pow(new_point.a - shifted_points[i].a, 2) +
                                  pow(new_point.b - shifted_points[i].b, 2));

                if (dist > convergence_threshold) {
                    converged = false;
                }
                shifted_points[i] = new_point;
            }
        }

        if (converged) {
            cout << "Convergence atteinte en " << iter + 1 << " itérations" << endl;
            break;
        }

        points = shifted_points; // Mise à jour des points pour l'itération suivante
    }

    // Création de l'image segmentée
    Image result(filename, format);
    result.width = width;
    result.height = height;
    result.size = size;
    result.data = createData();

    for (int i = 0; i < height; ++i) {
        for (int j = 0; j < width; ++j) {
            int index = getIndice(i, j, height, width);
            result.data[index * 3] = (OCTET)shifted_points[index].L;
            result.data[index * 3 + 1] = (OCTET)shifted_points[index].a;
            result.data[index * 3 + 2] = (OCTET)shifted_points[index].b;
        }
    }

    return result;
}


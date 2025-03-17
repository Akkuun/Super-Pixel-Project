#include "Image.h"
#include "image_ppm.h"
#include <iostream>
#include <vector>
#include <cstring>
#include <cmath>
#include <queue>
#include <omp.h>
#include <climits>

using namespace std;


/**
 * \brief Constructeur de la classe Image.
 * \param filename Nom du fichier image.
 * \param format Format de l'image (PGM ou PPM).
 * \return Une nouvelle instance de la classe Image.
 */
Image::Image(const std::string &filename, Format format)
        : filename(filename), format(format), width(0), height(0), size(0), data(nullptr), clusters(), labels() {}

/**
 * \brief Destructeur de la classe Image.
 * \details Libère la mémoire allouée pour les données de l'image.
 */
Image::~Image() {
    if (data) {
        free(data);
    }
    if (!clusters.empty()) {
        clusters.clear();
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
    float dL = cluster.Lk - data[index];
    float da = cluster.ak - data[index + 1];
    float db = cluster.bk - data[index + 2];
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
void Image::SLICC(int &k, int &m, int &N) {
    // PHASE 1 : Initialisation
    //1.1 Convertir l’image RGB en CIELab. -> ok
    //1.2 Définir le nombre de superpixels souhaité et calculer le pas de grille avec S = sqrt(N / k).
    // N / k donne le nombre total de super pixel présent dans la grille donc on fait sqrt pour récupérer la longueur du carré de S
    float S = sqrt(static_cast<float>(N) / k);
    cout << "Initialisation des centres de clusters" << endl;
    //1.3 Placer les centres de clusters Ck sur une grille régulière (avec un léger ajustement pour éviter les bords).
    vector <ClusterCenter> clusters(k);
    for (int clusterActuel = 0; clusterActuel < k; clusterActuel++) {
        // x et y sont les coordonnées du centre du cluster au sein de la grille S
        // on place les centres des clusters sur la grille régulière S et on ajoute un pas de S/2 pour les placer au centre

        int x = static_cast<int>((S / 2) + (clusterActuel % static_cast<int>(height / S)) * S);
        int y = static_cast<int>((S / 2) + (clusterActuel / static_cast<int>(width / S)) * S);
        clusters[clusterActuel].xk = x;
        clusters[clusterActuel].yk = y;
        // on met dans le cluster les valeurs de couleur du pixel L a b
        int index = getIndice(y, x, height, width);
        clusters[clusterActuel].Lk = data[index * 3];
        clusters[clusterActuel].ak = data[index * 3 + 1];
        clusters[clusterActuel].bk = data[index * 3 + 2];
    }
    cout << "fin initilisation des centres de clusters" << endl;

    //1.4 Initialiser la matrice des labels (indices des centres)  L(x, y) à -1 et la matrice des distances   D(x, y) à INF
    // dans notre cas les labels sont les indices des clusters
    vector<int> labels(size / 3, -1); // /3 car on a 3 valeurs par pixel(L, a, b)
    vector<float> distances(size / 3, INFINITY);

    //1.5 Initialiser le seuil de convergence ΔCk
    float seuil = 0.00001;
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
        for (int cluster = 0; cluster < k; cluster++) {
            //2.2 Pour chaque pixel P(x, y) dans une fenêtre 2S x 2S autour de C_k
            for (int dx = -S; dx <= S; dx++) {
                for (int dy = -S; dy <= S; dy++) {
                    //2.3 Calculer la distance D(P, C_k) entre le pixel P et le centre C_k
                    int x = clusters[cluster].xk + dx;
                    int y = clusters[cluster].yk + dy;

                    if (x >= 0 && x < width && y >= 0 && y < height) {
                        int index = getIndice(y, x, height, width);
                        float d_lab = calculerDistanceCouleur(clusters[cluster], x, y);
                        float d_xy = calculerDistanceSpatiale(clusters[cluster], x, y);
                        float D = d_lab + (m / S) * d_xy;

                        if (D < distances[index]) {
                            distances[index] = D;
                            labels[index] = cluster;
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

    // PHASE 4 : Correction de la connectivité (SLICC spécifique)
    vector<int> listeComposantesConnexes;
    vector<int> newLabels(size / 3, -1);
    int label = 0;
    cout << "debut de la correction de la connectivié via floodFill" << endl;
// 4.1 Parcourir l'image pour détecter les superpixels non connexes
    for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++) {
            int index = getIndice(i, j, height, width);
            if (newLabels[index] == -1) {
                // Effectuer un flood fill pour identifier les composantes connexes
                listeComposantesConnexes.push_back(floodFill(i, j, newLabels, label, labels));
                label++;
            }
        }
    }
    cout << "fin de la correction de la connectivié via floodFill" << endl;
    cout << "debut fusion des petis segments" << endl;
// 4.2 Fusionner les petits segments
    int tailleSeuilMinimal = size / (3 * k);
    for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++) {
            int index = getIndice(i, j, height, width);
            // Affecter au superpixel voisin le plus proche
            int bestLabel = affecterSuperPixelVoisin(i, j, newLabels, listeComposantesConnexes, labels,
                                                     tailleSeuilMinimal, clusters);
            // si le pixel n'est pas affecté à un superpixel voisin, on le laisse tel quel
            if (bestLabel != -1) {
                newLabels[index] = bestLabel;
            }
        }
    }
    cout << "fin fusion des petis segments" << endl;
    // Mettre à jour les labels
    labels = newLabels;

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
    this->clusters = clusters;
    this->labels = labels;
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

/**
 * \brief Applique K_Mean sur l'image
 * @return L'image avec les couleurs K_Mean
 */
Image Image::K_Mean() {
    vector <vector<int>> classes; // vecteur contenant les classes de couleurs
    int K = 256;
    OCTET *ImgOutFinal = createData();
    OCTET *ImgOutInit = createData();
    OCTET *ImgIn = copyData();
    vector<int> pixelClasses(height * width, 0); // vecteur contenant les pixels des classes

    cout << "debut remplissage K-MEAN 256 couleurs" << endl;
    //on prends 256 couleurs aléatoires pour définir nos classe
    for (int i = 0; i < K; i++) {
        int randIndex = rand() % (width * height);
        //creation d'une couleur
        vector<int> color = {ImgIn[randIndex * 3], ImgIn[randIndex * 3 + 1], ImgIn[randIndex * 3 + 2]};
        //insertion couleur
        classes.push_back(color);
    }
    cout << "fin remplissage K-MEAN 256 couleurs" << endl;
    //on applique l'algorithme de K-Mean
    cout << "debut K-MEAN" << endl;
    bool converged = false;
    while (!converged) {
        pixelClasses = ClassificationPixelKClasse(classes, ImgIn, K);

        for (int i = 0; i < height; i++) {
            for (int j = 0; j < width; j++) {
                //on recupere la couleur de la classe correspondante
                vector<int> color = classes[pixelClasses[getIndice(i, j, width, height)]];
                ImgOutInit[getIndice(i, j, width, height) * 3] = color[0];
                ImgOutInit[getIndice(i, j, width, height) * 3 + 1] = color[1];
                ImgOutInit[getIndice(i, j, width, height) * 3 + 2] = color[2];
            }
        }


        vector <vector<int>> oldClasses = classes; // Sauvegarde des anciennes classes
        calculerMoyenneKClasses(pixelClasses, classes, ImgIn, K);
        // Vérification de la convergence : si les classes ne changent plus, alors elles sont bien séparées
        converged = true;
        for (int i = 0; i < K; i++) {
            for (int j = 0; j < 3; j++) {
                if (abs(classes[i][j] - oldClasses[i][j]) > 1) { // Seuil de tolérance : 1
                    converged = false;
                    break;
                }
            }
            if (!converged) break;
        }
    }

    // Generation de l'image avec les nouvelles couleurs de classes
    for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++) {
            //on recupere la couleur de la classe correspondante
            vector<int> color = classes[pixelClasses[getIndice(i, j, width, height)]];
            ImgOutFinal[getIndice(i, j, width, height) * 3] = color[0];
            ImgOutFinal[getIndice(i, j, width, height) * 3 + 1] = color[1];
            ImgOutFinal[getIndice(i, j, width, height) * 3 + 2] = color[2];
        }
    }
    cout << "fin K-MEAN" << endl;

    Image K_mean(filename + "_K_MEAN", PPM);
    K_mean.width = width;
    K_mean.height = height;
    K_mean.data = ImgOutFinal;
    free(ImgIn);
    free(ImgOutInit);
    return K_mean;


}

/**
 * \brief calcule le taux de compression entre l'image originale et l'image compressée
 * @param imageCompressee
 * @return
 */
float Image::calculerTauxCompression(Image &imageCompressee) {
    return (size * 3) / (imageCompressee.size * 3);
}

/**
 * \brief Répartit les pixels en K classes différentes en calculant leur distance avec les valeur de classes moyennes
 * @param nH
 * @param nW
 * @param classes
 * @param ImgIn
 * @param K
 * @return vecteur contenant les pixels des classes
 */
vector<int>
Image::ClassificationPixelKClasse(const vector <vector<int>> &classes, OCTET *ImgIn, int K) {
    vector<int> pixelClasses(height * width, 0); // vecteur contenant les pixels des classes
    int indiceClasseLaPlusProche = 0;

    for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++) {
            float distanceMin = 1000000;
            //on doit attribuer la classe la plus proche du pixel,
            // on doit alors calculer chaque distance entre le pixel et chaque classe

            for (int k = 0; k < K; k++) {
                //calcul de la distance entre la classe k de la boucle et le point
                float d = distanceEuclidienne(ImgIn, i, j, classes[k]);
                if (d < distanceMin) {
                    distanceMin = d;//si la distance calcule est la plus petite rencontree, alors ça devient la nouvelle distance min
                    indiceClasseLaPlusProche = k;//on met à jour l'indice de la classe la plus proche
                }
            }
            pixelClasses[getIndice(i, j, width, height)] = indiceClasseLaPlusProche;
        }
    }

    return pixelClasses;
}

/**
 * \brief calcul la distance euclidienne entre un pixel et une classe
 * @param ImgIn
 * @param i
 * @param j
 * @param valueClasse
 * @return la distance euclidienne entre le pixel et la classe
 */
float Image::distanceEuclidienne(unsigned char *ImgIn, int i, int j, vector<int> valueClasse) {
    float distance = 0;
    // on fait x, y, z
    //on fait classe 1R, classe 1G, classe 1B
    for (int k = 0; k < 3; k++) {
        distance += pow(valueClasse[k] - ImgIn[getIndice(i, j, width, height) * 3 + k], 2);
    }
    return sqrt(distance);
}

/**
 * \brief Calcul la moyenne des valeurs RGB de chaque classe
 * @param pixelClasses
 * @param classes
 * @param ImgIn
 * @param K
 */
void
Image::calculerMoyenneKClasses(const vector<int> &pixelClasses, vector <vector<int>> &classes,
                               OCTET *ImgIn,
                               int K) {
    vector <vector<int>> sumClasses(K, vector<int>(3,
                                                   0)); // Pour accumuler les composantes R, G, B pour chaque classes parmis K
    vector<int> nbPixelParClasse(K, 0); // Pour accumuler le nombre de pixels par classes

    //on calcul la somme des valeurs RGB de chaque pixel de chaque classe et le nombre de pixel de chaque classe
    for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++) {
            //recupere l'indice du point
            int index = getIndice(i, j, height, width) * 3;
            //recuperation de la classe du pixel de l'index correspondant
            int classe = pixelClasses[getIndice(i, j, width, height)];
            //on ajoute les valeurs RGB du pixel à la classe correspondante
            sumClasses[classe][0] += ImgIn[index];
            sumClasses[classe][1] += ImgIn[index + 1];
            sumClasses[classe][2] += ImgIn[index + 2];
            //on incremente le nombre de pixel de la classe
            nbPixelParClasse[classe]++;
        }
    }

    for (int i = 0; i < K; i++) {
        //on calcule la moyenne de chaque classe
        if (nbPixelParClasse[i] > 0) {
            for (int k = 0; k < 3; k++) classes[i][k] = sumClasses[i][k] / nbPixelParClasse[i];
        } else {
            //si la classe n'a pas de pixel, on met la classe à 0
            classes[i][0] = 0;
            classes[i][1] = 0;
            classes[i][2] = 0;
        }
    }

}



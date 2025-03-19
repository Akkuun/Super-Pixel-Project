/*
Outil de compactage/décompactage de fichier utilisant le codage de Huffman
Compilation sous windows avec Visual C++
Compilation sous Linux : gcc -DLINUX_COMPIL huffman.c -o huffman
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef LINUX_COMPIL            /* Compilation conditionnelle */
 #include <sys/time.h>
#endif

#define CODE_MAX_LEN 32
#define BUF_LEN 512

/* Definition des structures */
struct arbre
{
   short branche0;
   short branche1;
};

struct arbre_data
{
   unsigned long frequence;
   short index_suivant;
};

struct dictionnaire
{
   unsigned char taille;
   char code[CODE_MAX_LEN];
};

/* Prototypes des fonctions */
short huffman_calculer_frequences(FILE *, unsigned long *, unsigned short *);
short huffman_lire_frequences(FILE *);
short huffman_creer_arbre(short);
void huffman_creer_dictionnaire(unsigned char *, short, short);
char huffman_creer_fichier_frequences(FILE *, FILE *);
char huffman_compacter(FILE *, FILE *, FILE *);
char huffman_decompacter(FILE *, FILE *, FILE *);

/* Variables globales */
struct arbre_data *arbre_d;
struct arbre *arbre;
struct dictionnaire *dico;

/*
Calcule la fréquence de chaque caractère dans un fichier et trie la liste de structures
par fréquences croissantes
*src : pointeur sur le fichier
*nbre_octets : pointeur sur une variable recevant la taille du fichier
*nbre_ascii : pointeur sur une variable recevant le nombre de caractères
différents présents dans le fichier source
Retourne l'index de la première structure dans la liste triée
*/
short huffman_calculer_frequences(FILE *src, unsigned long *nbre_octets, unsigned short *nbre_ascii)
{
   int i, r;
   unsigned char buffer[BUF_LEN];
   short index_frequence=0, index_precedent=-1;

   int continuer=1;
   short c1, c2;

   *nbre_octets=0;
   *nbre_ascii=0;

   memset(arbre_d, 0, 512*sizeof(struct arbre_data));

   /*Lecture dans le fichier */
   while ((r=fread(buffer, 1, BUF_LEN, src))>0)
   {
      *nbre_octets+=r;
      for(i=0; i<r; i++)
         /* incrémentation du compteur du caractère correspondant */
         arbre_d[buffer[i]].frequence++;
   }

   /* Chainage des structures avec une fréquence supérieure à 0 */
   for(i=0; i<256; i++)
      if (arbre_d[i].frequence>0)
      {
         (*nbre_ascii)++;
         if (index_precedent==-1)
            index_frequence=i;
         else
            arbre_d[index_precedent].index_suivant=i;
         index_precedent=i;
      }
   if (index_precedent==-1)
      index_frequence=-1;
     else
      arbre_d[index_precedent].index_suivant=-1;

   /* Tri des structures (bubble sort) */
   while (continuer)
   {
      c1=index_frequence;
      continuer=0;
      index_precedent=-1;
      while(c1!=-1)
      {
         if ((c2=arbre_d[c1].index_suivant)!=-1)
         {
            if (arbre_d[c1].frequence>arbre_d[c2].frequence)
            {
               continuer=1;
               if (index_precedent==-1)
                  index_frequence=c2;
               else
                  arbre_d[index_precedent].index_suivant=c2;
               arbre_d[c1].index_suivant=arbre_d[c2].index_suivant;
               arbre_d[c2].index_suivant=c1;
            }
            index_precedent=c1;
            c1=c2;
         }
         else
            c1=c2;
      }
   }

   /* On retourne l'index de la première structure */
   return index_frequence;
}

/*
Lit les fréquences de chaque caractère, inscrites soit dans le fichier
compressé, soit dans un fichier de fréquences spécial
*/
short huffman_lire_frequences(FILE *frq)
{
   unsigned short nbre_ascii;
   unsigned char i;
   short index_frequence=-1, index_precedent=-1;

   /* Lecture de la taille de la table des fréquences */
   fread(&nbre_ascii, 2, 1, frq);

   /* Lecture de la table des fréquences */
   while (nbre_ascii>0)
   {
      /* Lecture du caractère en cours */
      fread(&i, 1, 1, frq);
      /* Lecture de la fréquence du caractère en cours */
      fread((char *)&arbre_d[i].frequence, 4, 1, frq);
      /* Chainage de la structure */
      if (index_frequence==-1)
         index_frequence=i;
      else
         arbre_d[index_precedent].index_suivant=i;
      index_precedent=i;
      nbre_ascii--;
   }
   if (index_precedent==-1)
      return -1;
   arbre_d[index_precedent].index_suivant=-1;
   return index_frequence;
}

/*
Crée un arbre à partir d'une liste de fréquences
index_fréquence : idnex de la première structure dans la liste *arbre_d
Retourne l'index de la racine de l'arbre dans la liste *arbre
*/
short huffman_creer_arbre(short index_frequence)
{
   short i, j, j_save;
   unsigned long somme_frequence;
   short nbre_noeuds=256;
   char struct_inseree=0;

   /* Les structures 0 à 255 correspondent aux caractères, ce sont des terminaisons => -1 */
   for(j=0; j<256; j++)
      arbre[j].branche0=arbre[j].branche1=-1;

   /* Création de l'arbre :
   La mise en commun les deux fréquences les plus faibles crée un nouveau noeud avec une frequence
   égale a la somme des deux fréquences.
   Il s'agit ensuite d'insérer cette nouvelle structure dans la liste triée */
   i=index_frequence;
   while(i!=-1)
   {
      if (arbre_d[i].index_suivant==-1)
      {
         /*printf("Arbre cree\n");*/
         break;
      }
      /*printf("%d\n", arbre_d[i].frequence);
      printf("%d\n", arbre_d[arbre_d[i].index_suivant].frequence); */
      somme_frequence=arbre_d[i].frequence + arbre_d[arbre_d[i].index_suivant].frequence;
      /*printf("Nouveau noeud : %d (%d) et %d (%d) => %d\n", i, arbre_d[i].frequence, 
         arbre_d[i].index_suivant, arbre_d[arbre_d[i].index_suivant].frequence, 
         somme_frequence);*/
      arbre_d[nbre_noeuds].frequence=somme_frequence;
      arbre[nbre_noeuds].branche0=arbre_d[i].index_suivant;
      arbre[nbre_noeuds].branche1=i;
      /* Insertion du nouveau noeud dans la liste triée */
      j_save=-1;
      struct_inseree=0;
      j=i;
      while (j!=-1 && struct_inseree==0)
      {
         if (arbre_d[j].frequence>=somme_frequence)
         {
            if (j_save!=-1)
               arbre_d[j_save].index_suivant=nbre_noeuds;
            arbre_d[nbre_noeuds].index_suivant=j;
            /*printf("Insertion du nouveau noeud : entre %d et %d\n", 
               j_save==-1?-1:arbre_d[j_save].frequence, arbre_d[j].frequence);*/
            struct_inseree=1;
         }
         j_save=j;
         j=arbre_d[j].index_suivant;
      }
      /* Insertion du nouveau noeud a la fin */
      if (struct_inseree==0)
      {
         arbre_d[j_save].index_suivant=nbre_noeuds;
         arbre_d[nbre_noeuds].index_suivant=-1;
         /*printf("Insertion du nouveau noeud à la fin : %d\n", arbre_d[j_save].frequence);*/
      }
      nbre_noeuds++;
      i=arbre_d[i].index_suivant;
      i=arbre_d[i].index_suivant;
   }
   /* On retourne l'index du noeud racine */
   return nbre_noeuds-1;
}

/*
Procédure récursive qui crée un dictionnaire (correspondance entre la valeur ascii
d'un caractère et son codage obtenu avec la compression huffman) à partir d'un arbre
*code : pointeur sur une zone mémoire de taille CODE_MAX_LEN recevant
temporairement le code, au fur et à mesure de la progression dans l'arbre
index : position dans l'arbre (index de la structure courante)
pos : nombre de bits deja inscrits dans *code
*/
void huffman_creer_dictionnaire(unsigned char *code, short index, short pos)
{
   /* On a atteint une terminaison de l'arbre : c'est un caractère */
   if ((arbre[index].branche0==-1) && (arbre[index].branche1==-1))
   {
      /* Copie du code dans le dictionnaire */
      memcpy(dico[index].code, code, CODE_MAX_LEN);
      /*printf("%c: %x - %d\n", index, code[0], pos);*/
      /* taille du code en bits */
      dico[index].taille=(unsigned char)pos;
   }
   /* le noeud possède d'autres branches : on continue à les suivre */
   else
   {
      /* On suit la branche ajoutant un bit valant 0 */
      code[pos/8]&=~(0x80>>(pos%8));
      /* Le "(short)" devant "(pos+1)", c'est juste pour empecher VC++
      de chipoter (Warning : integral size mismatch in argument : 
      conversion supplied) */
      huffman_creer_dictionnaire(code, arbre[index].branche0, (short)(pos+1));
      /* On suit la branche ajoutant un bit valant 1 */
      code[pos/8]|=0x80>>(pos%8);
      huffman_creer_dictionnaire(code, arbre[index].branche1, (short)(pos+1));
   }
}

/*
Crée une table de fréquences à partir de src, l'inscrit dans dst,
puis affiche un tableau des caractères de la table, avec leur fréquence.
*/
char huffman_creer_fichier_frequences(FILE *src, FILE *dst)
{
   short i, compteur=0;
   unsigned long nbre_octets;
   unsigned short nbre_ascii=0;

   i=huffman_calculer_frequences(src, &nbre_octets, &nbre_ascii);

   /*Ecriture de la taille de la table des fréquences */
   fwrite(&nbre_ascii, 1, 1, dst);

   printf("Création du fichier de fréquences...\n");
   printf("%d caractères représentés sur 256\n", nbre_ascii);

   /*Ecriture de la table des fréquences */
    while(i!=-1)
   {
      nbre_ascii=i;
      fwrite(&nbre_ascii, 1, 1, dst);
      fwrite((char *)&arbre_d[i].frequence, 4, 1, dst);
      if ((nbre_ascii>=32 && nbre_ascii<=126) || (nbre_ascii>=161 && nbre_ascii <=255))
         printf("%-4c %-6d", nbre_ascii, arbre_d[i].frequence);
      else
         printf("0x%02x %-6d", nbre_ascii, arbre_d[i].frequence);
      if (((compteur+1)%7)==0)
         printf("\n");
      compteur++;
      i=arbre_d[i].index_suivant;
   }
   printf("\n");
   return 0;
}

/*
Compresse le fichier src.
Le résulat se trouve dans le fichier dst
Si frq est différent de NULL, il est utilsé pour lire la table des fréquences
nécessaire à la construction de l'arbre
*/
char huffman_compacter(FILE *src, FILE *dst, FILE *frq)
{
   int i, r, octet_r, bit_r, bit_count, bit_w;
   unsigned long nbre_octets;
   short index_frequence;
   short racine_arbre;
   unsigned short nbre_ascii=0;
   unsigned char code[CODE_MAX_LEN];
   unsigned char buffer[BUF_LEN];

#ifdef LINUX_COMPIL
   struct timeval tv;
   unsigned int t_debut;
   gettimeofday(&tv, NULL);
   t_debut=tv.tv_usec;
#endif

   printf("Compression en cours...\n");

   /* création ou lecture de la table des fréquences */
   if (frq==NULL)
   {
      index_frequence=huffman_calculer_frequences(src, &nbre_octets, &nbre_ascii);
   }
   else
   {
      printf("Utilisation d'un fichier de fréquences pour la création du dico\n");
      fseek(src, 0, SEEK_END);
      nbre_octets=ftell(src);
      index_frequence=huffman_lire_frequences(frq);
   }

   /*Ecriture de la taille en octets du fichier original */
   fwrite((char *)&nbre_octets, 4, 1, dst);
   /*Ecriture de la taille de la table des fréquences */
   fwrite(&nbre_ascii, 2, 1, dst);

   /*Ecriture de la table des fréquences */
   if (frq==NULL)
   {
      i=index_frequence;
      while(i!=-1)
      {
         nbre_ascii=i;
         fwrite(&nbre_ascii, 1, 1, dst);
         fwrite((char *)&arbre_d[i].frequence, 4, 1, dst);
         i=arbre_d[i].index_suivant;
      }
   }

   /* Coinstruction de l'arbre à partir de la table des fréquences */
   racine_arbre=huffman_creer_arbre(index_frequence);

   /* Allocation de mémoire pour le dictionnaire */
   if ((dico=(struct dictionnaire *)malloc(256*sizeof(struct dictionnaire)))==NULL)
   {
      /*free(arbre_d);
      free(arbre);*/
      perror("malloc");
      return -1;
   }

   /* RAZ du champs taille du dico. Si on utilise une table de fréquences
   prédéfinie pour la compression, et qu'un caractère à compresser n'est pas
   présent dans la table, alors il ne sera pas traité */
   for(i=0; i<256; i++)
      dico[i].taille=0;

   /* Création du dictionnaire à partir de l'arbre */
   huffman_creer_dictionnaire(code, racine_arbre, 0);

   /* Compression du fichier source et écriture dans le fichier cible */
   fseek(src, 0, SEEK_SET);
   code[0]=0;
   bit_w=0x80;
   /* Lecture de BUF_LEN octets dans le fichier source */
   while ((r=fread(buffer, 1, BUF_LEN, src))>0)
   {
      /* Traitement octet par octet */
      for(i=0; i<r; i++)
      {
         /* Ecriture du code correspondant au caractère dans le dictionnaire */
         octet_r=0;
         bit_r=0x80;
         /* Ecriture bit par bit */
         for(bit_count=0; bit_count<dico[buffer[i]].taille; bit_count++)
         {
            if (dico[buffer[i]].code[octet_r] & bit_r)
               code[0]|=bit_w;
            /*else
               code[0]&=~(bit_w); */
            bit_r>>=1;
            if (bit_r==0)
            {
               octet_r++;
               bit_r=0x80;
            }
            bit_w>>=1;
            if (bit_w==0)
            {
               /*printf("%3x", code[0]);*/
               fputc(code[0], dst);
               code[0]=0;
               bit_w=0x80;
            }
         }
      }
   }
   if(bit_w!=0x80)
      fputc(code[0], dst);

   free(dico);
#ifdef LINUX_COMPIL
   gettimeofday(&tv, NULL);
   printf("Compactage effectué en %u \xb5s. Taux de compression: %.2f\n", tv.tv_usec-t_debut, (float)nbre_octets/ftell(dst));
#else
   printf("Compactage terminé. Taux de compression : %.2f\n", (float)nbre_octets/ftell(dst));
#endif
   return 0;
}

/*
Decompresse le fichier src
Le résultat se trouve dans dst
*/
char huffman_decompacter(FILE *src, FILE *dst, FILE *frq)
{
   int i, j, r;
   unsigned long nbre_octets;
   unsigned char nbre_ascii, bit_r;
   short index_frequence;
   short racine_arbre;
   unsigned char buffer[BUF_LEN];

#ifdef LINUX_COMPIL
   struct timeval tv;
   unsigned int t_debut;
   gettimeofday(&tv, NULL);
   t_debut=tv.tv_usec;
#endif

   /* Lecture de la taille du fichier original */
   fread((char *)&nbre_octets, 4, 1, src);
   printf("Taille du fichier original : %d octets\n", nbre_octets);
   printf("Décompression en cours...\n");

   /* Lecture de la table des fréquences */
   if (frq==NULL)
   {
      index_frequence=huffman_lire_frequences(src);
   }
   else
   {
      fread(&nbre_ascii, 1, 1, src);
      index_frequence=huffman_lire_frequences(frq);
   }

   if (index_frequence==-1)
   {
      printf("Erreur de lecture de la table des frequences\n");
      return -1;
   }

   /* Construction de l'arbre à partir de la table des fréquences */
   racine_arbre=huffman_creer_arbre(index_frequence);

   /* Decompression du fichier source et écriture du résultat dans le fichier cible */
   j=racine_arbre;
   /* Lecture de BUF_LEN octets dans le fichier source */
   while ((r=fread(buffer, 1, BUF_LEN, src))>0)
   {
      /* Traitement octet par octet */
      for(i=0; i<r && nbre_octets>0; i++)
      {
         /* Traitement bit par bit */
         for(bit_r=0x80; bit_r!=0 && nbre_octets>0; bit_r>>=1)
         {
            if (buffer[i]&bit_r)
               j=arbre[j].branche1;
            else
               j=arbre[j].branche0;
            if ((arbre[j].branche0==-1) || (arbre[j].branche1==-1))
            {
               /*printf("%c", j);*/
               fputc((char)j, dst);
               nbre_octets--;
               j=racine_arbre;
            }
         }
      }
   }
#ifdef LINUX_COMPIL
   gettimeofday(&tv, NULL);
   printf("Décompression effectuée en %u\xb5s\n", tv.tv_usec-t_debut);
#else
   printf("Décompression terminée\n");
#endif
   return 0;
}

/*
Fonction principale
Ouvre les fichiers en lecture ou écriture et alloue un espace mémoire suffisant
pour les structures
*/

int main(int argc, char *argv[])
{
   FILE *src, *dst, *frq;

   /* Syntaxe incorrecte */
   if (argc<4)
   {
      printf("%s c/d/f source destination [frequence]\n", argv[0]);
      return -1;
   }

   /* Ouverture du fichier source en lecture */
   if ((src=fopen(argv[2], "rb"))==NULL)
   {
      perror("fopen");
      return -1;
   }

   /* Ouverture du fichier cible en écriture */
   if ((dst=fopen(argv[3], "wb"))==NULL)
   {
      perror("fopen");
      return -1;
   }

   /* Ouverture d'un éventuel fichier de liste de fréquences, en lecture */
   if (argc>4)
   {
      if ((frq=fopen(argv[4], "rb"))==NULL)
      {
         perror("fopen");
         return -1;
      }
   }
   else
      frq=NULL;

   /* Allocation mémoire pour les données diverses necessaires à la construction de l'arbre */
   if ((arbre_d=(struct arbre_data *)malloc(512*sizeof(struct arbre_data)))==NULL)
   {
      perror("malloc");
      return -1;
   }

   /* Allocation d'une zone mémoire pour l'arbre */
   if ((arbre=(struct arbre *)malloc(512*sizeof(struct arbre)))==NULL)
   {
      free(arbre_d);
      perror("malloc");
      return -1;
   }

   /* Compression ou décompression ? */
   if (*argv[1]=='c')
      huffman_compacter(src, dst, frq);
   else if (*argv[1]=='d')
      huffman_decompacter(src, dst, frq);
   else if (*argv[1]=='f')
      huffman_creer_fichier_frequences(src, dst);

   /* Libération de la mémoire */
   free(arbre_d);
   free(arbre);

   /* Fermeture des fichiers */
   fclose(src);
   fclose(dst);
   if (frq!=NULL)
      fclose(frq);
#ifdef LINUX_COMPIL
   printf("Linux rules!\n");
#endif
   return 0;
}


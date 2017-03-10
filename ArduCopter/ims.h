/*
 * ims.h
 *
 *  Created on: Feb 15, 2017
 *      Author: Daniel Monier-Reyes
 */

// Utilisation de la HAL pour pouvoir écrire sur la console de monitoring
//#include "Copter.h"

// Pour les fonctions d'écriture/lecture de fichiers
#include <fstream>
#include <iostream>

// -----------------------------------------------------------------------
// Déclaration des variables
// -----------------------------------------------------------------------

// -----------------------------------------------------------------------
// Déclaration des classes
// -----------------------------------------------------------------------

// Classe représentant une equation récurrente du second ordre
class Correcteur_2nd_Ordre_Discret
{
public:
    // Constructeur de la classe prenant en paramètre les coefficients de l'équation récurrente
    Correcteur_2nd_Ordre_Discret(double m_cxn=0,double m_cxn_1=0,double m_cxn_2=0,double m_cyn_1=0,double m_cyn_2=0)
    {
        cxn=m_cxn;
        cxn_1=m_cxn_1;
        cxn_2=m_cxn_2;
        cyn_1=m_cyn_1;
        cyn_2=m_cyn_2;

        xn=0;
        xn_1=0;
        xn_2=0;
        yn=0;
        yn_1=0;
        yn_2=0;

    }

    // Calcul d'un cycle de l'équation récurrente
    void cycle(double new_xn);

    // Récupération de la valeur de sortie de l'équation récurrente y(n)
    double getyn();

    // Réinitialisation des valeurs de l'équation récurrente
    void reset(void);


private:
    double cxn,cxn_1,cxn_2,cyn_1,cyn_2;  // Coefficients de l'équation récurrente
    double xn;                           // Valeur de l'échantillon d'entrée x(n)
    double xn_1,xn_2;                    // Valeurs successives des échantillons d'entrée x(n-1), x(n-2)
    double yn;                           // Valeur de la sortie y(n)
    double yn_1,yn_2;                    // Valeurs successives de la sortie y(n-1), y(n-2)
};

// ---------------------------------------------------------------------------
// Déclaration des fonctions
// ---------------------------------------------------------------------------

//int open_file_csv_log(void);
void write_csv_log(std::ofstream myfile);

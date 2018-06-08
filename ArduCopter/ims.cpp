/*
 * ims.cpp
 *
 *  Created on: Feb 15, 2017
 *      Author: Daniel Monier-Reyes
 */

#include "ims.h"

// Pour les fonctions d'écriture/lecture de fichiers
#include <fstream>
#include <iostream>

// ----------------------------------------------------------------------------
// Fonctions de la classe représentant une equation récurrente du second ordre
// ----------------------------------------------------------------------------

// Récupération de la valeur de sortie de l'équation récurrente y(n) du 2nd ordre
double Correcteur_2nd_Ordre_Discret::getyn()
{
    return yn;
}

// Récupération de la valeur de sortie de l'équation récurrente y(n) du 1er ordre
double Correcteur_1er_Ordre_Discret::getyn()
{
    return yn;
}

// Réinitialisation des valeurs de l'équation récurrente du 2nd ordre
void Correcteur_2nd_Ordre_Discret::reset(void)
{
    xn=0;
    xn_1=0;
    xn_2=0;
    yn=0;
    yn_1=0;
    yn_2=0;
}

// Réinitialisation des valeurs de l'équation récurrente du 1er ordre
void Correcteur_1er_Ordre_Discret::reset(void)
{
    xn=0;
    xn_1=0;
    yn=0;
    yn_1=0;
}

// Calcul d'un cycle de l'équation récurrente du 2nd ordre
void Correcteur_2nd_Ordre_Discret::cycle(double new_xn)
{
    xn=new_xn;

    yn=(cxn*xn)+(cxn_1*xn_1)+(cxn_2*xn_2)+(cyn_1*yn_1)+(cyn_2*yn_2);

    yn_2=yn_1;
    yn_1=yn;
    xn_2=xn_1;
    xn_1=xn;
}

// Calcul d'un cycle de l'équation récurrente du 1er ordre
void Correcteur_1er_Ordre_Discret::cycle(double new_xn)
{
    xn=new_xn;

    yn=(cxn*xn)+(cxn_1*xn_1)+(cyn_1*yn_1);

    yn_1=yn;
    xn_1=xn;
}

// -----------------------------------------------------------------------------
// Fonctions de lecture écriture de fichiers
// -----------------------------------------------------------------------------
/*void open_file_csv_log(void)
{
    if (!outf)
    {
        // If we couldn't open the output file stream for writing
        // Print an error and exit
        return 1;
    }
}*/

/*void write_csv_log(std::ofstream myfile)
{


    myfile.open("IMS1_CSV_LOG.dat");

    // We'll write two lines into this file
    myfile << "This is line 1" << std::endl;
    myfile << "This is line 2" << std::endl;

    myfile.close();

}
*/
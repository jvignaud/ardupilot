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
// Fonction d'écriture des fichiers log
// ----------------------------------------------------------------------------

void ecriture_log(std::ofstream *fichier, double roll, double pitch, double yaw_rate ,double target_roll_rad, double target_pitch_rad, double target_yaw_rate_rad, double target_throttle_newton, double pos_theta, double pos_phi, double pos_r, double pos_z, double moteur1, double moteur2, double moteur3, double moteur4, int16_t moteur1_pwm, int16_t moteur2_pwm, int16_t moteur3_pwm, int16_t moteur4_pwm)
{
    *fichier << roll;
    *fichier << ",";
    *fichier << pitch;
    *fichier << ",";
    *fichier << yaw_rate;
    *fichier << ",";
    *fichier << target_roll_rad;
    *fichier << ",";
    *fichier << target_pitch_rad;
    *fichier << ",";
    *fichier << target_yaw_rate_rad;
    *fichier << ",";
    *fichier << target_throttle_newton;
    *fichier << ",";
    *fichier << pos_phi;
    *fichier << ",";
    *fichier << pos_theta;
    *fichier << ",";
    *fichier << pos_r;
    *fichier << ",";
    *fichier << pos_z;
    *fichier << ",";
    *fichier << moteur1;
    *fichier << ",";
    *fichier << moteur2;
    *fichier << ",";
    *fichier << moteur3;
    *fichier << ",";
    *fichier << moteur4;
    *fichier << ",";
    *fichier << moteur1_pwm;
    *fichier << ",";
    *fichier << moteur2_pwm;
    *fichier << ",";
    *fichier << moteur3_pwm;
    *fichier << ",";
    *fichier << moteur4_pwm;
    *fichier << std::endl;
}

// ----------------------------------------------------------------------------
// Fonctions de la classe représentant une equation récurrente du premier ordre
// ----------------------------------------------------------------------------

// Récupération de la valeur de sortie de l'équation récurrente y(n) du 1er ordre
double Correcteur_1er_Ordre_Discret::getyn()
{
    return yn;
}

// Réinitialisation des valeurs de l'équation récurrente du 1er ordre
void Correcteur_1er_Ordre_Discret::reset(void)
{
    xn=0;
    xn_1=0;
    yn=0;
    yn_1=0;
}

// Calcul d'un cycle de l'équation récurrente du 1er ordre
void Correcteur_1er_Ordre_Discret::cycle(double new_xn)
{
    xn=new_xn;

    yn=(cxn*xn)+(cxn_1*xn_1)+(cyn_1*yn_1);

    yn_1=yn;
    xn_1=xn;
}

// ----------------------------------------------------------------------------
// Fonctions de la classe représentant une equation récurrente du second ordre
// ----------------------------------------------------------------------------

// Récupération de la valeur de sortie de l'équation récurrente y(n) du 2nd ordre
double Correcteur_2nd_Ordre_Discret::getyn()
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
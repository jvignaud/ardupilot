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
#include <time.h>

#define LOG_TIME 600

// -----------------------------------------------------------------------
// Déclaration des structures
// -----------------------------------------------------------------------

struct Coef_Correcteurs
{
    double xn;
    double xn_1;
    double xn_2;
    double yn_1;
    double yn_2;
};

// -----------------------------------------------------------------------
// Déclaration des classes
// -----------------------------------------------------------------------

// Classe permettant de récupérer les paramètres du drone comptenu dans un fichier
class Parametres_Drone
{
public:
    // Constructeur de la classe prenant en param�tre le nom du fichier contenant les paramètres
    Parametres_Drone(std::string nom_fichier);

    // Méthodes (fonctions)
    // Fonction permettant de regrouper les fonctions qui récupérent les données du fichier paramètre
    void set_parameters(void);
    // fonctions qui permettent de retourner les attributs de la classe "Parametres_Drone"
    float get_rotation_min(void) const;
    float get_rotation_max(void) const;
    float get_masse_arrachage(void) const;
    double get_coef_trainee(void) const;
    double get_coef_poussee(void) const;
    double get_envergure(void) const;
    struct Coef_Correcteurs get_roulis(void) const;
    struct Coef_Correcteurs get_tangage(void) const;
    struct Coef_Correcteurs get_lacet(void) const;
    struct Coef_Correcteurs get_consigne_smooth(void) const;
    int16_t get_offset_pwm(void) const;
    std::string get_fichier_log(void) const;
    float get_angle_max_roulis_tangage(void) const;
    float get_vitesse_max_lacet(void) const;
    bool get_affichage_parametres(void) const;
    bool get_affichage_consignes(void) const;
    bool get_affichage_erreur(void) const;
    bool get_affichage_AHRS(void) const;
    bool get_affichage_pwm(void) const;
    bool get_affichage_sorties_PID(void) const;
    bool get_affichage_commandes(void) const;
    bool get_test_poussee(void) const;


private:
    // fonctions qui permettent de récupérer et initialiser les attributs de la classe "Parametres_Drone"
    void set_fichier_log(void);
    void set_rotation_min(void);
    void set_rotation_max(void);
    void set_masse_arrachage(void);
    void set_coef_trainee(void);
    void set_coef_poussee(void);
    void set_envergure(void);
    void set_roulis(void);
    void set_tangage(void);
    void set_lacet(void);
    void set_consigne_smooth(void);
    void set_offset_pwm(void);
    void set_angle_max_roulis_tangage(void);
    void set_vitesse_max_lacet(void);
    void set_affichage_parametres(void);
    void set_affichage_consignes(void);
    void set_affichage_erreur(void);
    void set_affichage_AHRS(void);
    void set_affichage_pwm(void);
    void set_affichage_sorties_PID(void);
    void set_affichage_commandes(void);
    void set_test_poussee(void);

    void aller_a_la_ligne(int num_ligne);
    void aller_a_la_ligne_apres(int num_ligne_apres);

    // Attributs (variables)
    int16_t offset_pwm; // en % : valeur de pwm minimale envoyée au moteur
    std::string fichier_log;
    float rotation_min, rotation_max;
    float masse_arrachage;
    double envergure;
    double coef_trainee, coef_poussee;
    struct Coef_Correcteurs roulis,tangage,lacet,consigne_smooth;
    std::ifstream fichier;  // on ouvre le fichier en lecture
    float angle_max_roulis_tangage, vitesse_max_lacet;
    bool affichage_parametres,affichage_consignes,affichage_erreur,affichage_AHRS,affichage_pwm,affichage_sorties_PID,affichage_commandes,test_poussee;
    std::string texte;
};

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

class Correcteur_1er_Ordre_Discret
{
public:
    // Constructeur de la classe prenant en paramètre les coefficients de l'équation récurrente
    Correcteur_1er_Ordre_Discret(double m_cxn=0,double m_cxn_1=0,double m_cyn_1=0)
    {
        cxn=m_cxn;
        cxn_1=m_cxn_1;
        cyn_1=m_cyn_1;

        xn=0;
        xn_1=0;
        yn=0;
        yn_1=0;

    }

    // Calcul d'un cycle de l'équation récurrente
    void cycle(double new_xn);

    // Récupération de la valeur de sortie de l'équation récurrente y(n)
    double getyn();

    // Réinitialisation des valeurs de l'équation récurrente
    void reset(void);


private:
    double cxn,cxn_1,cyn_1;  // Coefficients de l'équation récurrente
    double xn;                          // Valeur de l'échantillon d'entrée x(n)
    double xn_1;                    	// Valeurs successives des échantillons d'entrée x(n-1)
    double yn;                          // Valeur de la sortie y(n)
    double yn_1;                    	// Valeurs successives de la sortie y(n-1)
};


// Classe représentant le drone
class Quadri
{
public:
// Constructeur de la classe
    Quadri(std::string emplacement_fichier_parametres);

    // Méthodes (fonctions)

    // Fonction de récupération des données :
private:
    // -des commandes
    void set_u_phi(void);
    void set_u_theta(void);
    void set_u_r(void);
    void set_u_z(void);

    // -des valeurs des pwm pour chaque moteurs
    void set_w1_pwm(void);
    void set_w2_pwm(void);
    void set_w3_pwm(void);
    void set_w4_pwm(void);

public:
    void set_pwm_moteurs(void); // fonction qui permet de calculer les pwm de tous les moteurs

    // -des valeurs relevées par les capteurs
    void set_angle_roulis(double m_angle_roulis);
    void set_angle_tangage(double m_angle_tangage);
    void set_vitesse_angle_lacet(double m_vitesse_angle_lacet);

    // -des valeurs pwm min et max
    void set_pwm_min(double m_pwm_min);
    void set_pwm_max(double m_pwm_max);

    // -des consignes de poussée et de la vitesse du lacet
    void set_pilot_throttle_scaled(double m_pilot_throttle_scaled);
    void set_target_yaw_rate(float m_target_yaw_rate);
    
    // Fonction qui test et initialise un fichier log
    void ouverture_fichier_log(void);

    // Fonction qui test et ferme le fichier log
    void fermeture_fichier_log(void);

    // Fonction qui reset toutes les suites récursives
    void reset_PID(void);

    // Fonction qui retourne les pwm
    double get_w1_pwm(void) const;
    double get_w2_pwm(void) const;
    double get_w3_pwm(void) const;
    double get_w4_pwm(void) const;

    float get_angle_max_roulis_tangage(void) const;
    float get_vitesse_max_lacet(void) const;

private:
    // Ecriture du fichier LOG
    void titre_log(void);
    void ecriture_log(void);

    // Vérification du non-dépassement du PWM max
    void test_pwm(void);

    // Debugger 
    void debugger(void);

// Attributs (variables)
private:
public:
    // classe qui récupére les paramètres du drone (attention à l'emplacement du fichier sur le drone)
    Parametres_Drone params;

    // wu = 25 rad/s
    // PID ROLL : y(n)=54.4335.x(n)+-107.3269.x(n-1)+52.9008.x(n-2)+1.839.y(n-1)+-0.83901.y(n-2)
    Correcteur_2nd_Ordre_Discret pid_roll;

    // PID PITCH : y(n)=60.6081.x(n)+-119.5013.x(n-1)+58.9015.x(n-2)+1.839.y(n-1)+-0.83901.y(n-2)
    Correcteur_2nd_Ordre_Discret pid_pitch;

    // PI R : y(n)=1.0955.x(n)+-1.0886.x(n-1)+1.y(n-1)
    Correcteur_1er_Ordre_Discret pi_yaw;

    // fonction smooth consigne
    Correcteur_2nd_Ordre_Discret roll_smooth,pitch_smooth,yaw_rate_smooth;

    // ofstream est utilisé pour écrire un fichier CSV nommé IMS5_CSV_LOG.dat, celui-ci contiendra toutes les informations de vol
    std::ofstream fichier_log;

    // variable vérifiant si le fichier est bien ouvert ou non
    bool fichier_log_ouvert;

    // nom du mode de vol qui apparaitra dans le fichier log
    std::string nom_programme;

    // Sortie des PIDs
    double u_theta, u_phi, u_r, u_z;

    // Commandes (moteurs en rad/s)
    double w1,w2,w3,w4;

    // Commandes (moteurs en pwm)
    int16_t w1_pwm,w2_pwm,w3_pwm,w4_pwm;

    // Valeurs de pwm min et max pour faire tourner les moteurs
    int16_t pwm_min,pwm_max;

    // Offset du pwm min en %
    int16_t offset_pwm;

    // Consigne de vitesse de lacet en centidegré/seconde
    float target_yaw_rate;

    // Consigne de poussée entre 0 et 1
    float pilot_throttle_scaled;

    // Consignes en radians et radians/seconde
    double target_roll_rad, target_pitch_rad, target_yaw_rate_rad; // Target angles en radians et radians/s
    double target_throttle_newton;                                 // Target poussée en Newton
    double target_roll_smooth, target_pitch_smooth, target_yaw_rate_smooth; // Target angles en radians et radians/s en smooth

    // Angles et vitesse récupérer par les capteurs (AHRS)
    double angle_roulis, angle_tangage, vitesse_angle_lacet;

    // Booléen permettant d'activer la fonction debugger
    bool debug;

public:
    // Index des moteurs (nécessaire pour adapter la numérotation des moteurs entre la loi de commande et Arducopter)
    const int16_t w1_index=4;                 // Représentation du drone en configuration en x
    const int16_t w2_index=2;                 //    W1   W3         4   1
    const int16_t w3_index=1;                 //       x      =>      x
    const int16_t w4_index=3;                 //    W4   W2         3   2

    // consigne en centi-degré
    float target_roll, target_pitch;
};
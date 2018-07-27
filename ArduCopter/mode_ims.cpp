/*
 * control_ims.cpp
 *
 * Créé le: 15 Février 2017
 *      Auteur : Daniel Monier-Reyes
 * 
 * Modifié  le : 9 Juillet 2018 
 *          par : Jamy Vignaud
 * 
 * Routines d'initialisation et d'appel pour le mode de vol IMS
 * Utilisation de la classe Quadri afin de simplifier la lecture du code et d'optimiser les calculs
 */


#include "Copter.h"
#include "ims.hpp"

 

// --------------------------------------------------------------------
// Déclaration des variables Globales
// --------------------------------------------------------------------

// Classe Quadri permettant de récupérer les données des capteurs, lire le fichier mis en argument (généré par un script MatLab), et de calculer les PWM
// /!\ Attention à l'emplacement du fichier sur le drone  
Quadri Drone_RPi("/home/pi/ardupilot/build/navio2/bin/ParametresDrone.conf");



// ---------------------------------------------------------------------------------------------
// ims_init - Routine d'initialisation du mode de vol IMS
// ---------------------------------------------------------------------------------------------
bool Copter::ModeIMS::init(bool ignore_checks)
{
    // Récupération des valeurs min et max pwm pour la rotation des moteurs
    Drone_RPi.set_pwm_min(motors->get_pwm_output_min());
    Drone_RPi.set_pwm_max(motors->get_pwm_output_max());

    // Si le drone est atterri et que le mode à partir duquel nous avons switché n'a pas de poussée manuel et le stick de poussée est trop haut
    if (motors->armed() && ap.land_complete && !copter.flightmode->has_manual_throttle() &&
            (get_pilot_desired_throttle(channel_throttle->get_control_in()) > get_non_takeoff_throttle())) {
        return false;
    }
    // set target altitude to zero for reporting
    pos_control->set_alt_target(0);

    return true;
}

// ---------------------------------------------------------------------------------------------
// ims_run - Routine d'appel du mode de vol IMS à exécuter à 400Hz
// ---------------------------------------------------------------------------------------------
void Copter::ModeIMS::run()
{
    // ------------------------------------------------------------------------
    // Récupération des consignes
    // ------------------------------------------------------------------------

    AP_Vehicle::MultiCopter &aparm = copter.aparm;

    // Récupération des consignes roulis et tangage (en centidegrés)
    get_pilot_desired_lean_angles(Drone_RPi.target_roll, Drone_RPi.target_pitch, aparm.angle_max, Drone_RPi.get_angle_max_roulis_tangage()*100);

    // Récupération de la consigne en lacet (en centidegrés par seconde) (67.0 est la vitesse maximale par défaut du lacet)
    // g.acro_yaw_p à voir... : vitesse lacet max
    Drone_RPi.set_target_yaw_rate(Drone_RPi.get_vitesse_max_lacet() / g.acro_yaw_p *  get_pilot_desired_yaw_rate(channel_yaw->get_control_in()));  

    // Récupération de la consigne en poussée (valeur entre 0 et 1)
    // Le choix entre la poussée exponentielle et linéaire se faire dans le fichier mis en argument dans la déclaration de la classe Quadri
    if (Drone_RPi.get_poussee_exponentielle())
    {
        // Poussée exponentielle
        Drone_RPi.set_pilot_throttle_scaled(get_pilot_desired_throttle(channel_throttle->get_control_in()));
    }
    else 
    {
        // Poussée linéaire       
        Drone_RPi.set_pilot_throttle_scaled(get_pilot_desired_throttle(channel_throttle->get_control_in(),0.5f)); 
    }  

    // Si les moteurs sont désarmés ou qu'ils ne tournent pas, ou encore, que le stick de la poussée est en bas
    if (!motors->armed() || ap.throttle_zero || !motors->get_interlock()) {
        zero_throttle_and_relax_ac();

        // Reset des PIDs pour éviter qu'ils divergent
        Drone_RPi.reset_PID();
        // Fermeture du fichier log
		Drone_RPi.fermeture_fichier_log();

        return;
    }

    else 
    {
        // Fonction qui test et initialise un fichier log
		Drone_RPi.ouverture_fichier_log();
    }

    // Réinitialisation du flag d'atterissage
    set_land_complete(false);

    // Tous les moteurs tournent lorsqu'ils sont armés
    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    // -----------------------------------------------------------------------------------------------
    // Récupération des données reçus par les capteurs
    // -----------------------------------------------------------------------------------------------

    // Récupération des angles de roulis et de tangage et de la vitesse du lacet dans la classe Quadri
	Drone_RPi.set_angle_roulis(ahrs.roll);
	Drone_RPi.set_angle_tangage(ahrs.pitch);
	Drone_RPi.set_vitesse_angle_lacet(ahrs.get_gyro().z);

    // ---------------------------------------------------------------------
    // Gestion des PWM
    // ---------------------------------------------------------------------

    // Calcul des PWM à envoyer aux moteurs
    Drone_RPi.set_pwm_moteurs();

    // ---------------------------------------------------------------------
    // Gestion Moteurs
    // ---------------------------------------------------------------------

    // Rotation des moteurs en fonction de la valeur en pwm des commandes
    motors->output_test(Drone_RPi.w1_index,Drone_RPi.get_w1_pwm());    // output_test : Fait tourner les moteurs à une valeur de PWM spécifiée : 
    motors->output_test(Drone_RPi.w2_index,Drone_RPi.get_w2_pwm());    // void output_test(uint8_t motor_seq, int16_t pwm);
    motors->output_test(Drone_RPi.w3_index,Drone_RPi.get_w3_pwm());    // motor_seq est le numéro du moteur (va de 1 au nombre de moteurs total)
    motors->output_test(Drone_RPi.w4_index,Drone_RPi.get_w4_pwm());    // pwm est la valeur pwm envoyée en sortie (normalement situé entre 1000 et 2000)
}

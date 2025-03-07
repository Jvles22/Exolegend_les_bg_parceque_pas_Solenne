#include "gladiator.h"

Gladiator* gladiator;

void reset();

double reductionAngle(double x) {
    // Ramène l'angle dans l'intervalle [-π, π]
    x = fmod(x + PI, 2 * PI);  
    if (x < 0) x += 2 * PI;     
    return x - PI;              
}

// Fonction pour orienter le robot vers le nord (0 radian)
void orienterVersNord() {
    // Obtenir la position actuelle du robot
    RobotData data = gladiator->robot->getData();
    double angleActuel = data.position.a;

    // Définir l'angle cible (0 radian pour le nord)
    double angleCible = 85 * (PI / 180); // Conversion degrés → radians

    // Trouver le chemin de rotation le plus court
    double erreur = reductionAngle(angleCible - angleActuel);

    // Rotation jusqu'à atteindre la position cible
    while (abs(erreur) > 0.05) {  
        angleActuel = gladiator->robot->getData().position.a;
        erreur = reductionAngle(angleCible - angleActuel);

        // Vitesse proportionnelle à l'erreur pour ralentir en fin de rotation
        double vitesse = std::max(0.02, 0.1 * abs(erreur));  

        // Définir la direction de rotation
        double direction = (erreur > 0) ? 1 : -1;

        // Appliquer la vitesse aux roues
        gladiator->control->setWheelSpeed(WheelAxis::RIGHT, vitesse * direction);
        gladiator->control->setWheelSpeed(WheelAxis::LEFT, -vitesse * direction);

        delay(10);  // Attendre un peu avant de réévaluer
    }

    // Arrêter complètement le robot
    gladiator->control->setWheelSpeed(WheelAxis::RIGHT, 0);
    gladiator->control->setWheelSpeed(WheelAxis::LEFT, 0);

    gladiator->log("Orientation vers le nord terminée avec précision");
}

void setup() {
    // Instanciation de l'objet gladiator
    gladiator = new Gladiator();
    // Enregistrement de la fonction de reset
    gladiator->game->onReset(&reset);
}

void reset() {
    // Initialisation des variables avant chaque partie
}

void loop() {
    if (gladiator->game->isStarted()) {  
        // Orienter le robot vers le nord dès que le match commence
        orienterVersNord();

        // Une fois orienté, ne pas répéter l'opération inutilement
        while (true) delay(100);
    }
}

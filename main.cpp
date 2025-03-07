#include "gladiator.h"

Gladiator* gladiator;
bool enSpirale = true;

void reset();

void setup() {
    gladiator = new Gladiator();
    gladiator->game->onReset(&reset);
}

void reset() {
    enSpirale = true;
}

void loop() {
    if (gladiator->game->isStarted() && enSpirale) {
        RobotData data = gladiator->robot->getData();
        Position position = data.position;

        double distanceCentre = sqrt(position.x * position.x + position.y * position.y);

        // Condition d'arrêt : si le robot est proche du centre (tolérance de 0.1 m)
        if (distanceCentre < 0.1) {
            gladiator->control->setWheelSpeed(WheelAxis::RIGHT, 0);
            gladiator->control->setWheelSpeed(WheelAxis::LEFT, 0);
            gladiator->log("Le robot est arrivé au centre !");
            enSpirale = false;
            return;
        }

        // Réduction progressive du rayon de la spirale
        double facteurDeReduction = 1.0 - (distanceCentre / 2.0); // Diminue en fonction de la distance au centre
        facteurDeReduction = std::max(0.2, facteurDeReduction); // Empêche une valeur trop basse

        // Définition des vitesses des roues pour avancer en spirale
        double vitesseMax = 0.3;  // Vitesse de la roue extérieure
        double vitesseMin = vitesseMax * facteurDeReduction; // La roue intérieure tourne moins vite

        gladiator->control->setWheelSpeed(WheelAxis::RIGHT, vitesseMax);
        gladiator->control->setWheelSpeed(WheelAxis::LEFT, vitesseMin);

        delay(100); // Petit délai pour ajustement
    }
}

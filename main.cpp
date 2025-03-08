#include "gladiator.h"
#include <cmath>
#include <vector>
#include <chrono>
#undef abs

// Constantes pour le contrôle du robot
float kw = 1.2;
float kv = 1.f;
float wlimit = 3.f;
float vlimit = 0.6;
float erreurPos = 0.07;

// Fonction pour réduire un angle dans l'intervalle [-π, π]
double reductionAngle(double x)
{
    x = fmod(x + M_PI, 2 * M_PI);
    if (x < 0)
        x += 2 * M_PI;
    return x - M_PI;
}

// Classe Vector2 pour représenter les positions
class Vector2 {
public:
    Vector2() : _x(0.), _y(0.) {}
    Vector2(float x, float y) : _x(x), _y(y) {}

    float norm2() const {
        return std::sqrt(_x * _x + _y * _y);
    }

    Vector2 operator-(const Vector2 &other) const {
        return {_x - other._x, _y - other._y};
    }

    float x() const { return _x; }
    float y() const { return _y; }
    float dot(const Vector2& other) const { return _x * other._x + _y * other._y; }
    float cross(const Vector2& other) const { return _x * other._y - _y * other._x; }
    float angle(const Vector2& m) const { return std::atan2(cross(m), dot(m)); }
    float angle() const { return std::atan2(_y, _x); }

private:
    float _x, _y;
};

Gladiator *gladiator;
int currentSize = 12; // Taille initiale du terrain
unsigned long lastShrinkTime = 0; // Temps du dernier rétrécissement

void reset() {
    // Fonction de reset
    gladiator->log("Call of reset function");
}

// Fonction pour déplacer le robot vers une position cible
void go_to(Position cons, Position pos)
{
    double consvl, consvr;
    double dx = cons.x - pos.x;
    double dy = cons.y - pos.y;
    double d = sqrt(dx * dx + dy * dy);

    if (d > erreurPos)
    {
        double rho = atan2(dy, dx);
        double consw = kw * reductionAngle(rho - pos.a);

        double consv = kv * d * cos(reductionAngle(rho - pos.a));
        consw = abs(consw) > wlimit ? (consw > 0 ? 1 : -1) * wlimit : consw;
        consv = abs(consv) > vlimit ? (consv > 0 ? 1 : -1) * vlimit : consv;

        consvl = consv - gladiator->robot->getRobotRadius() * consw; // GFA 3.6.2
        consvr = consv + gladiator->robot->getRobotRadius() * consw; // GFA 3.6.2
    }
    else
    {
        consvr = 0;
        consvl = 0;
    }

    gladiator->control->setWheelSpeed(WheelAxis::RIGHT, consvr, false); // GFA 3.2.1
    gladiator->control->setWheelSpeed(WheelAxis::LEFT, consvl, false);  // GFA 3.2.1
}

void setup() {
    // Instanciation de l'objet gladiator
    gladiator = new Gladiator();
    // Enregistrement de la fonction de reset
    gladiator->game->onReset(&reset);
}

void loop() {
    if (gladiator->game->isStarted()) {
        // Vérifier si le terrain doit rétrécir
        unsigned long currentTime = millis();
        if (currentTime - lastShrinkTime >= 18000) { // 18 secondes
            lastShrinkTime = currentTime;
            currentSize -= 2; // Réduire la taille du terrain
            gladiator->log("Terrain rétréci à %d cases", currentSize);
        }

        // Parcourir toutes les cases du terrain pour trouver des pièces
        for (int i = 0; i < currentSize; ++i) {
            for (int j = 0; j < currentSize; ++j) {
                MazeSquare *indexedSquare = gladiator->maze->getSquare(i, j);
                Coin coin = indexedSquare->coin;

                // Si une pièce est trouvée, déplacer le robot vers elle
                if (coin.value > 0) {
                    Position posCoin = coin.p;
                    Position myPosition = gladiator->robot->getData().position;
                    go_to(posCoin, myPosition);
                    gladiator->log("Déplacement vers la pièce à (%.2f, %.2f)", posCoin.x, posCoin.y);
                    return; // Sortir de la boucle après avoir trouvé une pièce
                }
            }
        }

        // Gestion des bombes
        int bombCount = gladiator->weapon->getBombCount();
        if (bombCount > 2) {
            // Dropper toutes les bombes sauf 2
            gladiator->weapon->dropBombs(bombCount - 2);
            gladiator->log("Drop bomb");
        } else if (gladiator->weapon->canDropBombs(1)) {
            // Dropper une bombe
            gladiator->weapon->dropBombs(1);
            gladiator->log("Drop bomb");
        }

        delay(100); // Boucle à 10 Hz
    }
}
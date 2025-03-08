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

void reset()
{
}

inline float moduloPi(float a) // return angle in [-pi; pi]
{
    return (a < 0.0) ? (std::fmod(a - M_PI, 2 * M_PI) + M_PI) : (std::fmod(a + M_PI, 2 * M_PI) - M_PI);
}

inline bool aim(Gladiator *gladiator, const Vector2 &target, bool showLogs)
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

    gladiator->control->setWheelSpeed(WheelAxis::LEFT, leftCommand);
    gladiator->control->setWheelSpeed(WheelAxis::RIGHT, rightCommand);

    if (showLogs || targetReached)
    {
        gladiator->log("ta %f, ca %f, ea %f, tx %f cx %f ex %f ty %f cy %f ey %f", targetAngle, posRaw.a, angleError,
                       target.x(), pos.x(), posError.x(), target.y(), pos.y(), posError.y());
    }

    return targetReached;
}

void setup() {
    // Instanciation de l'objet gladiator
    gladiator = new Gladiator();
    // enregistrement de la fonction de reset qui s'éxecute à chaque fois avant qu'une partie commence
    gladiator->game->onReset(&reset);
}




// Fonction pour détecter la position d'un coin proche du robot
Position findCoinPosition() {
    // Récupérer la case la plus proche du robot
    const MazeSquare* nearestSquare = gladiator->maze->getNearestSquare();

    // Rayon de recherche (chercher dans un carré de 3x3 cases autour du robot)
    int range = 1;  // Rechercher dans une zone de 3x3 autour du robot

    // Parcourir les cases autour du robot pour vérifier si un coin est présent
    for (int di = -range; di <= range; ++di) {
        for (int dj = -range; dj <= range; ++dj) {
            int i = nearestSquare->i + di;  // L'indice de la colonne (horizontal)
            int j = nearestSquare->j + dj;  // L'indice de la ligne (vertical)

            // Vérifier si les indices sont valides dans la grille (limites 0 <= i, j < 11)
            if (i >= 0 && j >= 0 && i < 11 && j < 11) {
                // Récupérer la case correspondante
                const MazeSquare* square = gladiator->maze->getSquare(i, j);

                // Vérifier si la case contient un coin (indépendamment d'une bombe)
                if (square && square->coin.value > 0) {
                    // Si un coin est présent sur la case, renvoyer sa position
                    return square->coin.p;
                }
            }
        }
    }

    // Si aucun coin n'est trouvé, renvoyer une position invalide (0, 0)
    return Position{0, 0, 0};
}



void loop() {
    if (gladiator->game->isStarted()) {
        // Trouver la position d'un coin proche du robot
        Position coinPos = findCoinPosition();

        // Afficher la position du coin
        if (coinPos.x != 0 || coinPos.y != 0) {
            gladiator->log("Coin trouvé à la position: (%.2f, %.2f)", coinPos.x, coinPos.y);
        } else {
            gladiator->log("Aucun coin trouvé autour.");
        }
        static unsigned i = 0;
        bool showLogs = (i % 50 == 0);

        if (aim(gladiator, coinPos, showLogs))
        {
            gladiator->log("target atteinte !");
        }
        i++;
    }
    delay(1000);  // Attendre avant de relancer la boucle
}

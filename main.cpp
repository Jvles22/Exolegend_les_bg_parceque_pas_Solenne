#include "gladiator.h"
#include <cmath>
#undef abs

// x,y représentent des coordonnées en m
// Vector{1.5,1.5} représente le point central
// Pour convertir une cordonnée de cellule (i,j) (0<=i<=13, 0<=j<=13) :
// x = i * CELL_SIZE + 0.5*CELL_SIZE
// y = j * CELL_SIZE + 0.5*CELL_SIZE
// avec CELL_SIZE = 3.0/14 (~0.214)

class Vector2
{
  public:
    Vector2() : _x(0.), _y(0.)
    {
    }
    Vector2(float x, float y) : _x(x), _y(y)
    {
    }

    float norm1() const
    {
        return std::abs(_x) + std::abs(_y);
    }
    float norm2() const
    {
        return std::sqrt(_x * _x + _y * _y);
    }
    void normalize()
    {
        _x /= norm2();
        _y /= norm2();
    }
    Vector2 normalized() const
    {
        Vector2 out = *this;
        out.normalize();
        return out;
    }

    Vector2 operator-(const Vector2 &other) const
    {
        return {_x - other._x, _y - other._y};
    }
    Vector2 operator+(const Vector2 &other) const
    {
        return {_x + other._x, _y + other._y};
    }
    Vector2 operator*(float f) const
    {
        return {_x * f, _y * f};
    }

    bool operator==(const Vector2 &other) const
    {
        return std::abs(_x - other._x) < 1e-5 && std::abs(_y - other._y) < 1e-5;
    }
    bool operator!=(const Vector2 &other) const
    {
        return !(*this == other);
    }

    float x() const
    {
        return _x;
    }
    float y() const
    {
        return _y;
    }

    float dot(const Vector2 &other) const
    {
        return _x * other._x + _y * other._y;
    }
    float cross(const Vector2 &other) const
    {
        return _x * other._y - _y * other._x;
    }
    float angle(const Vector2 &m) const
    {
        return std::atan2(cross(m), dot(m));
    }
    float angle() const
    {
        return std::atan2(_y, _x);
    }

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
    constexpr float ANGLE_REACHED_THRESHOLD = 0.1;
    constexpr float POS_REACHED_THRESHOLD = 0.05;

    auto posRaw = gladiator->robot->getData().position;
    Vector2 pos{posRaw.x, posRaw.y};

    Vector2 posError = target - pos;

    float targetAngle = posError.angle();
    float angleError = moduloPi(targetAngle - posRaw.a);

    bool targetReached = false;
    float leftCommand = 0.f;
    float rightCommand = 0.f;

    if (posError.norm2() < POS_REACHED_THRESHOLD) //
    {
        targetReached = true;
    }
    else if (std::abs(angleError) > ANGLE_REACHED_THRESHOLD)
    {
        float factor = 0.1; // Réduit le facteur pour ralentir la rotation
        if (angleError < 0)
            factor = -factor;
        rightCommand = factor;
        leftCommand = -factor;
    }
    else
    {
        float factor = 0.3; // Réduit le facteur pour ralentir le mouvement
        rightCommand = factor; //+angleError*0.1  => terme optionel, "pseudo correction angulaire";
        leftCommand = factor;  //-angleError*0.1   => terme optionel, "pseudo correction angulaire";
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

void setup()
{
    // instanciation de l'objet gladiator
    gladiator = new Gladiator();
    // enregistrement de la fonction de reset qui s'éxecute à chaque fois avant qu'une partie commence
    gladiator->game->onReset(&reset);
}

Position findCoinPosition() {
    // Récupérer la case la plus proche du robot
    const MazeSquare* nearestSquare = gladiator->maze->getNearestSquare();

    // Rayon de recherche (chercher dans un carré de 3x3 cases autour du robot)
    int range = 3;  // Rechercher dans une zone de 3x3 autour du robot

    // Initialiser la position du coin le plus proche à une valeur invalide
    Position closestCoinPosition = {-1, -1};
    int closestDistance = INT_MAX;  // Initialiser avec une distance maximale

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
                    // Calculer la distance entre le robot et le coin
                    int distance = abs(di) + abs(dj);

                    // Si cette distance est plus courte que la distance minimale actuelle, mettre à jour
                    if (distance < closestDistance) {
                        closestDistance = distance;
                        closestCoinPosition = square->coin.p;
                    }
                }
            }
        }
    }

    // Retourner la position du coin le plus proche
    return closestCoinPosition;
}

void loop()
{
    // gladiator->log(gladiator->maze);
    if (gladiator->game->isStarted())
    {
        Position pos = findCoinPosition();
        gladiator->log("coin position : %d; %d", pos.x, pos.y);
        static unsigned i = 0;
        bool showLogs = (i % 50 == 0);

        // if (aim(gladiator, {pos.x, pos.y}, showLogs))
        if (aim(gladiator, {1.5, 1.5}, showLogs))
        {
            gladiator->log("target atteinte !");
        }
        if (aim(gladiator, {0, 0}, showLogs))
        {
            gladiator->log("target atteinte !");
        }
        if (aim(gladiator, {1.5, 1.5}, showLogs))
        {
            gladiator->log("target atteinte !");
        }
        int bombCount = gladiator->weapon->getBombCount();
        gladiator->log("bombes restantes : %d", bombCount);

        // Si il reste plus de 2 bombes
        if (bombCount > 0) {
            // Dropper toutes les bombes sauf 2
            gladiator->weapon->dropBombs(bombCount);
            gladiator->log("Drop bomb");
        // Il peux dropper au moins 1 bombe
        // } else if (gladiator->weapon->canDropBombs(1)) {
        //     // Dropper une bombe
        //     gladiator->weapon->dropBombs(1);
        //     gladiator->log("Drop bomb");
        }
        i++;
    }
    delay(50); // Augmenter le délai pour ralentir la boucle principale
}

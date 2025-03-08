#include "gladiator.h"
#include <cmath>
#include <vector>
#include <chrono>
#undef abs

float kw = 1.2;
float kv = 1.f;
float wlimit = 3.f;
float vlimit = 0.6;
float erreurPos = 0.07;



double reductionAngle(double x)
{
    x = fmod(x + PI, 2 * PI);
    if (x < 0)
        x += 2 * PI;
    return x - PI;
}

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
    float dot(const Vector2& other) const { return _x*other._x + _y*other._y; }
    float cross(const Vector2& other) const { return _x*other._y - _y*other._x; }
    float angle(const Vector2& m) const { return std::atan2(cross(m), dot(m)); }
    float angle() const { return std::atan2(_y, _x); }


private:
    float _x, _y;
};
inline float moduloPi(float a) {
    return (a < 0.0) ? (std::fmod(a - M_PI, 2 * M_PI) + M_PI) : (std::fmod(a + M_PI, 2 * M_PI) - M_PI);
}

inline bool aim(Gladiator* gladiator, const Vector2& target, bool showLogs)
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
        float factor = 0.2;
        if (angleError < 0)
            factor = - factor;
        rightCommand = factor;
        leftCommand = -factor;
    }
    else {
        float factor = 0.5;
        rightCommand = factor;//+angleError*0.1  => terme optionel, "pseudo correction angulaire";
        leftCommand = factor;//-angleError*0.1   => terme optionel, "pseudo correction angulaire";
    }

    gladiator->control->setWheelSpeed(WheelAxis::LEFT, leftCommand);
    gladiator->control->setWheelSpeed(WheelAxis::RIGHT, rightCommand);

    if (showLogs || targetReached)
    {
        gladiator->log("ta %f, ca %f, ea %f, tx %f cx %f ex %f ty %f cy %f ey %f", targetAngle, posRaw.a, angleError, target.x(), pos.x(), posError.x(), target.y(), pos.y(), posError.y());
    }

    return targetReached;
}


Gladiator *gladiator;
int currentSize = 12; // Taille initiale du terrain modifiée à 8
unsigned long lastShrinkTime = 0; // Temps du dernier rétrécissement

void reset() {}


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


inline bool aim(Gladiator *gladiator, const Vector2 &target) {
    constexpr float ANGLE_REACHED_THRESHOLD = 0.1;
    constexpr float POS_REACHED_THRESHOLD = 0.05;

    auto posRaw = gladiator->robot->getData().position;
    Vector2 pos{posRaw.x, posRaw.y};

    Vector2 posError = target - pos;

    float targetAngle = std::atan2(posError.y(), posError.x());
    float angleError = moduloPi(targetAngle - posRaw.a);

    float leftCommand = 0.f;
    float rightCommand = 0.f;

    if (posError.norm2() < POS_REACHED_THRESHOLD) {
        return true; // Cible atteinte
    } else if (std::abs(angleError) > ANGLE_REACHED_THRESHOLD) {
        float factor = 0.2;
        if (angleError < 0) factor = -factor;
        rightCommand = factor;
        leftCommand = -factor;
    } else {
        float factor = 0.5;
        rightCommand = factor;
        leftCommand = factor;
    }

    gladiator->control->setWheelSpeed(WheelAxis::LEFT, leftCommand);
    gladiator->control->setWheelSpeed(WheelAxis::RIGHT, rightCommand);

    return false; // Cible non atteinte
}

void setup() {
    gladiator = new Gladiator();
    gladiator->game->onReset(&reset);
}

void loop() {
    if (gladiator->game->isStarted()) {
        static std::vector<Vector2> targets;

        // Vérifier si le terrain doit rétrécir

        // Générer toutes les positions de la matrice actuell
        
        unsigned long currentTime = millis();
        if (currentTime - lastShrinkTime >= 18000) { // 20 secondes
            lastShrinkTime = currentTime;
            currentSize -= 2;

        }
        int id=gladiator->robot->getData().id;
        if (id==63) {
        for (int i = currentSize/2; i < currentSize; ++i) {
            for (int j = 0; j < currentSize; ++j) {
                
        
        MazeSquare *indexedSquare = gladiator->maze->getSquare(i, j);
        Coin coin = indexedSquare->coin;

         if (coin.value > 0)   {
            Position posCoin = coin.p;
            Position myPosition = gladiator->robot->getData().position;
            
            go_to(posCoin,myPosition);
           } 
         }
    }
} else {
    for (int i = 0; i < currentSize/2; ++i) {
        for (int j = 0; j < currentSize; ++j) {
            
    
    MazeSquare *indexedSquare = gladiator->maze->getSquare(i, j);
    Coin coin = indexedSquare->coin;

     if (coin.value > 0)   {
        Position posCoin = coin.p;
        Position myPosition = gladiator->robot->getData().position;
        
        go_to(posCoin,myPosition);
       } 
     }
}
}
    int bombCount = gladiator->weapon->getBombCount();

    // Si il reste plus de 2 bombes
    if (bombCount > 2) {
        // Dropper toutes les bombes sauf 2
        gladiator->weapon->dropBombs(bombCount - 2);
        gladiator->log("Drop bomb");
    // Il peut dropper au moins 1 bombe
    } else if (gladiator->weapon->canDropBombs(1)) {
        // Dropper une bombe
        gladiator->weapon->dropBombs(1);
        gladiator->log("Drop bomb");
    }

    delay(100); // boucle à 100Hz
}

}
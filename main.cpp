#include "gladiator.h"
#include <cmath>
#include <algorithm>
#undef abs

class Vector2 {
  public:
    constexpr Vector2() : _x(0.f), _y(0.f) {}
    constexpr Vector2(float x, float y) : _x(x), _y(y) {}

    [[nodiscard]] float norm1() const { return std::abs(_x) + std::abs(_y); }
    [[nodiscard]] float norm2() const { return std::sqrt(_x * _x + _y * _y); }
    void normalize() {
        float n = norm2();
        if (n > 1e-5) {
            *this *= (1.0f / n);
        }
    }
    [[nodiscard]] Vector2 normalized() const {
        Vector2 out = *this;
        out.normalize();
        return out;
    }

    [[nodiscard]] Vector2 operator-(const Vector2 &other) const { return {_x - other._x, _y - other._y}; }
    [[nodiscard]] Vector2 operator+(const Vector2 &other) const { return {_x + other._x, _y + other._y}; }
    [[nodiscard]] Vector2 operator*(float f) const { return {_x * f, _y * f}; }
    Vector2 &operator*=(float f) {
        _x *= f;
        _y *= f;
        return *this;
    }
    [[nodiscard]] bool operator==(const Vector2 &other) const { return std::abs(_x - other._x) < 1e-5 && std::abs(_y - other._y) < 1e-5; }
    [[nodiscard]] bool operator!=(const Vector2 &other) const { return !(*this == other); }
    [[nodiscard]] float dot(const Vector2 &other) const { return _x * other._x + _y * other._y; }
    [[nodiscard]] float cross(const Vector2 &other) const { return _x * other._y - _y * other._x; }
    [[nodiscard]] float angle(const Vector2 &m) const { return std::atan2(cross(m), dot(m)); }
    [[nodiscard]] float angle() const { return std::atan2(_y, _x); }
    [[nodiscard]] float x() const { return _x; }
    [[nodiscard]] float y() const { return _y; }

  private:
    float _x, _y;
};

Gladiator *gladiator;

void reset() {}

inline float moduloPi(float a) {
    return std::fmod(a + M_PI, 2 * M_PI) - M_PI;
}

bool aim(Gladiator *gladiator, const Vector2 &target, bool showLogs) {
    constexpr float ANGLE_REACHED_THRESHOLD = 0.1f;
    constexpr float POS_REACHED_THRESHOLD = 0.05f;
    
    auto posRaw = gladiator->robot->getData().position;
    Vector2 pos{posRaw.x, posRaw.y};
    Vector2 posError = target - pos;

    if (posError.norm2() < POS_REACHED_THRESHOLD) {
        return true;
    }

    float targetAngle = posError.angle();
    float angleError = moduloPi(targetAngle - posRaw.a);

    float factor = (std::abs(angleError) > ANGLE_REACHED_THRESHOLD) ? 0.1f : 0.3f;
    factor = (angleError < 0) ? -factor : factor;

    gladiator->control->setWheelSpeed(WheelAxis::LEFT, -factor);
    gladiator->control->setWheelSpeed(WheelAxis::RIGHT, factor);

    if (showLogs) {
        gladiator->log("Angle Target: %f, Current: %f, Error: %f", targetAngle, posRaw.a, angleError);
    }

    return false;
}

bool aimFast(Gladiator *gladiator, const Vector2 &target) {
    auto posRaw = gladiator->robot->getData().position;
    Vector2 pos{posRaw.x, posRaw.y};
    //Vector2 direction = (target - pos).normalized();
    
    gladiator->control->setWheelSpeed(WheelAxis::LEFT, 1.0f);
    gladiator->control->setWheelSpeed(WheelAxis::RIGHT, 1.0f);
    
    return (target - pos).norm2() < 0.05f;
}

int findEnemy() {
    for (int id = 1; id <= 127; ++id) {
        if (id == 63 || id == 57) continue; // Ignorer les robots alliés
        RobotData enemy = gladiator->game->getOtherRobotData(id);
        if (enemy.id != 0 && enemy.lifes > 0) {
            return id;
        }
    }
    return -1;
}

void setup() {
    gladiator = new Gladiator();
    gladiator->game->onReset(&reset);
}

Position findCoinPosition() {
    const MazeSquare* nearestSquare = gladiator->maze->getNearestSquare();
    constexpr int range = 3;
    Position closestCoinPosition = {-1, -1};
    int closestDistance = INT_MAX;

    for (int di = -range; di <= range; ++di) {
        for (int dj = -range; dj <= range; ++dj) {
            int i = nearestSquare->i + di, j = nearestSquare->j + dj;
            if (i >= 0 && j >= 0 && i < 11 && j < 11) {
                const MazeSquare* square = gladiator->maze->getSquare(i, j);
                if (square && square->coin.value > 0) {
                    int distance = std::abs(di) + std::abs(dj);
                    if (distance < closestDistance) {
                        closestDistance = distance;
                        closestCoinPosition = square->coin.p;
                    }
                }
            }
        }
    }
    return closestCoinPosition;
}

void loop() {
    if (gladiator->game->isStarted()) {
        // 1. Attaquer les ennemis en priorité
        int enemyId = findEnemy();
        if (enemyId != -1) {
            RobotData enemy = gladiator->game->getOtherRobotData(enemyId);
            Vector2 enemyPos{enemy.position.x, enemy.position.y};
            gladiator->log("Attacking enemy at: %0.1f, %0.1f", enemy.position.x, enemy.position.y);
            if (aimFast(gladiator, enemyPos)) {
                gladiator->log("Enemy reached! Dropping bombs...");
                int bombCount = gladiator->weapon->getBombCount();
                if (bombCount > 0) {
                    gladiator->weapon->dropBombs(bombCount);
                    gladiator->log("Dropped bombs on enemy");
                }
            }
            return; // Sortir de la boucle pour continuer à attaquer
        }

        // 2. Si aucun ennemi n'est trouvé, chercher des pièces
        Position pos = findCoinPosition();
        if (pos.x != -1 && pos.y != -1) {
            gladiator->log("Coin Position: %0.1f, %0.1f", pos.x, pos.y);
            static unsigned i = 0;
            bool showLogs = (i++ % 50 == 0);

            if (aim(gladiator, {pos.x, pos.y}, showLogs)) {
                gladiator->log("Target reached!");
            }
        }

        // 3. Larguer des bombes si nécessaire
        int bombCount = gladiator->weapon->getBombCount();
        if (bombCount > 0) {
            gladiator->weapon->dropBombs(bombCount);
            gladiator->log("Dropped bombs");
        }
    }
    delay(50); // Boucle à 20 Hz
}
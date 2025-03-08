#include "gladiator.h"
#include <Arduino.h>
#undef abs  // Undefine the macro defined in Arduino.h

#include <cmath>
#include <queue>
#include <unordered_map>
#include <vector>
#include <algorithm>
#include <cstdint>

// Constantes
constexpr float CELL_SIZE = 3.0f / 14;
constexpr int RANGE = 3;
constexpr int LOG_FREQUENCY = 50;
constexpr float ANGLE_REACHED_THRESHOLD = 0.1f;
constexpr float POS_REACHED_THRESHOLD = 0.05f;
constexpr float KW = 1.2f;
constexpr float KV = 1.0f;
constexpr float WLIMIT = 3.0f;
constexpr float VLIMIT = 0.6f;
constexpr float ERREUR_POS = 0.07f;

class Vector2 {
public:
    Vector2() : _x(0.), _y(0.) {}
    Vector2(float x, float y) : _x(x), _y(y) {}

    float norm1() const { return std::abs(_x) + std::abs(_y); }
    float norm2() const { return std::sqrt(_x * _x + _y * _y); }
    void normalize() { _x /= norm2(); _y /= norm2(); }
    Vector2 normalized() const { Vector2 out = *this; out.normalize(); return out; }

    Vector2 operator-(const Vector2 &other) const { return {_x - other._x, _y - other._y}; }
    Vector2 operator+(const Vector2 &other) const { return {_x + other._x, _y + other._y}; }
    Vector2 operator*(float f) const { return {_x * f, _y * f}; }

    bool operator==(const Vector2 &other) const { return std::abs(_x - other._x) < 1e-5 && std::abs(_y - other._y) < 1e-5; }
    bool operator!=(const Vector2 &other) const { return !(*this == other); }

    float x() const { return _x; }
    float y() const { return _y; }

    float dot(const Vector2 &other) const { return _x * other._x + _y * other._y; }
    float cross(const Vector2 &other) const { return _x * other._y - _y * other._x; }
    float angle(const Vector2 &m) const { return std::atan2(cross(m), dot(m)); }
    float angle() const { return std::atan2(_y, _x); }

private:
    float _x, _y;
};

Gladiator *gladiator;

void reset() {
    gladiator->log("Call of reset function");
}

inline float moduloPi(float a) {
    return (a < 0.0) ? (std::fmod(a - M_PI, 2 * M_PI) + M_PI) : (std::fmod(a + M_PI, 2 * M_PI) - M_PI);
}

inline bool aim(Gladiator *gladiator, const Vector2 &target, bool showLogs) {
    auto posRaw = gladiator->robot->getData().position;
    Vector2 pos{posRaw.x, posRaw.y};
    Vector2 posError = target - pos;
    float targetAngle = posError.angle();
    float angleError = moduloPi(targetAngle - posRaw.a);
    bool targetReached = posError.norm2() < POS_REACHED_THRESHOLD;
    float leftCommand = 0.f;
    float rightCommand = 0.f;

    if (!targetReached) {
        if (std::abs(angleError) > ANGLE_REACHED_THRESHOLD) {
            float factor = angleError < 0 ? -0.2f : 0.2f;
            rightCommand = factor;
            leftCommand = -factor;
        } else {
            float factor = 0.5f;
            rightCommand = factor;
            leftCommand = factor;
        }
    }

    gladiator->control->setWheelSpeed(WheelAxis::LEFT, leftCommand);
    gladiator->control->setWheelSpeed(WheelAxis::RIGHT, rightCommand);

    if (showLogs || targetReached) {
        gladiator->log("ta %f, ca %f, ea %f, tx %f cx %f ex %f ty %f cy %f ey %f", targetAngle, posRaw.a, angleError,
                       target.x(), pos.x(), posError.x(), target.y(), pos.y(), posError.y());
    }

    return targetReached;
}

Position findClosestBombPosition() {
    const MazeSquare* nearestSquare = gladiator->maze->getNearestSquare();
    if (!nearestSquare) return {-1, -1};

    Position closestBombPosition = {-1, -1};
    int closestDistance = INT_MAX;

    // Check surrounding squares for bombs
    for (int di = -RANGE; di <= RANGE; ++di) {
        for (int dj = -RANGE; dj <= RANGE; ++dj) {
            int i = nearestSquare->i + di;
            int j = nearestSquare->j + dj;

            if (i >= 0 && j >= 0 && i < 11 && j < 11) {
                const MazeSquare* square = gladiator->maze->getSquare(i, j);
                if (square && square->coin.value == 0) {  // No coin here, check for bombs
                    int distance = abs(di) + abs(dj);
                    if (square->isBomb && distance < closestDistance) {  // Check if there's a bomb
                        closestDistance = distance;
                        closestBombPosition = square->coin.p;
                    }
                }
            }
        }
    }

    return closestBombPosition;
}

bool isSquareFree(Position pos) {
    const MazeSquare* square = gladiator->maze->getSquare(pos.x, pos.y);
    return (square && square->possession == 0 && !square->isBomb);  // Ensure the square is not occupied and has no bomb
}

void loop() {
    if (gladiator->game->isStarted()) {
        static unsigned i = 0;
        bool showLogs = (i % LOG_FREQUENCY == 0);

        Position bombPos = findClosestBombPosition();

        if (bombPos.x != -1 && bombPos.y != -1) {
            // Move to the closest bomb if it's not already collected
            if (aim(gladiator, {static_cast<float>(bombPos.x), static_cast<float>(bombPos.y)}, showLogs)) {
                // Check if the position is free to drop a bomb
                if (isSquareFree(bombPos)) {
                    gladiator->weapon->dropBombs(1);
                    gladiator->log("Bomb dropped at (%.2f, %.2f)", bombPos.x, bombPos.y);  // Use %.2f to format float
                } else {
                    gladiator->log("Square occupied, moving to next bomb.");
                }
            }
        }

        i++;
    }

    delay(10);
}

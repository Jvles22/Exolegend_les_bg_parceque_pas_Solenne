#include "gladiator.h"
#include <cmath>
#include <queue>
#include <unordered_map>
#include <vector>
#include <algorithm>
#include <cstdint>

// Undefine the abs macro
#undef abs

// Constantes
constexpr float CELL_SIZE = 3.0f / 14;
constexpr int RANGE = 3;
constexpr int LOG_FREQUENCY = 50;
constexpr float ANGLE_REACHED_THRESHOLD = 0.05f;
constexpr float POS_REACHED_THRESHOLD = 0.05f;
constexpr float KW = 1.2f;
constexpr float KV = 1.0f;
constexpr float WLIMIT = 3.0f;
constexpr float VLIMIT = 0.6f;
constexpr float ERREUR_POS = 0.07f;


struct CustomPosition {
    float i;
    float j;
};


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

    gladiator->log("Target: (%f, %f), Current: (%f, %f), Error: (%f, %f)",
                   target.x(), target.y(), pos.x(), pos.y(), posError.x(), posError.y());
    gladiator->log("Target Angle: %f, Current Angle: %f, Angle Error: %f",
                   targetAngle, posRaw.a, angleError);
    gladiator->log("Angle Reached Threshold: %f, Abs Angle Error: %f",
                   ANGLE_REACHED_THRESHOLD, std::abs(angleError));

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

    gladiator->log("Left Command: %f, Right Command: %f", leftCommand, rightCommand);

    gladiator->control->setWheelSpeed(WheelAxis::LEFT, leftCommand);
    gladiator->control->setWheelSpeed(WheelAxis::RIGHT, rightCommand);

    if (showLogs || targetReached) {
        gladiator->log("ta %f, ca %f, ea %f, tx %f cx %f ex %f ty %f cy %f ey %f", targetAngle, posRaw.a, angleError,
                       target.x(), pos.x(), posError.x(), target.y(), pos.y(), posError.y());
    }

    return targetReached;
}


void setup() {
    gladiator = new Gladiator();
    gladiator->game->onReset(&reset);
}

Position findCoinPosition() {
    const MazeSquare* nearestSquare = gladiator->maze->getNearestSquare();
    if (!nearestSquare) return {-1, -1};

    Position closestCoinPosition = {-1, -1};
    int closestDistance = INT_MAX;

    for (int di = -RANGE; di <= RANGE; ++di) {
        for (int dj = -RANGE; dj <= RANGE; ++dj) {
            int i = nearestSquare->i + di;
            int j = nearestSquare->j + dj;

            if (i >= 0 && j >= 0 && i < 11 && j < 11) {
                const MazeSquare* square = gladiator->maze->getSquare(i, j);
                if (square && square->coin.value > 0) {
                    int distance = abs(di) + abs(dj);
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

std::vector<MazeSquare*> bfs(MazeSquare* start, MazeSquare* goal) {
    std::queue<MazeSquare*> q;
    std::unordered_map<MazeSquare*, MazeSquare*> parent;
    q.push(start);
    parent[start] = nullptr;

    while (!q.empty()) {
        MazeSquare* current = q.front(); q.pop();
        if (current == goal) break;
        
        // Vérifier chaque voisin en s'assurant qu'il n'y a pas de mur entre les cases
        // if (current->northSquare && parent.find(current->northSquare) == parent.end() && !current->hasNorthWall) {
        //     parent[current->northSquare] = current;
        //     q.push(current->northSquare);
        // }
        // if (current->southSquare && parent.find(current->southSquare) == parent.end() && !current->hasSouthWall) {
        //     parent[current->southSquare] = current;
        //     q.push(current->southSquare);
        // }
        // if (current->eastSquare && parent.find(current->eastSquare) == parent.end() && !current->hasEastWall) {
        //     parent[current->eastSquare] = current;
        //     q.push(current->eastSquare);
        // }
        // if (current->westSquare && parent.find(current->westSquare) == parent.end() && !current->hasWestWall) {
        //     parent[current->westSquare] = current;
        //     q.push(current->westSquare);
        // }
    }

    std::vector<MazeSquare*> path;
    for (MazeSquare* at = goal; at; at = parent[at])
        path.push_back(at);
    std::reverse(path.begin(), path.end());
    return path;
}
CustomPosition xyToij(float X, float Y, float A) {
    CustomPosition robotPos;
    
    // Applique la matrice de rotation inverse pour passer du repère cartésien au repère du robot
    robotPos.i = X * cos(A) + Y * sin(A);
    robotPos.j = -X * sin(A) + Y * cos(A);
    
    return robotPos;
}
void loop() {
    if (gladiator->game->isStarted()) {
        Position pos1 = findCoinPosition();
        // Récupérez la position actuelle du robot
        auto robotPosition = gladiator->robot->getData().position;
        // Utilisez la position du robot pour A
        CustomPosition pos = xyToij(pos1.x, pos1.y, robotPosition.a);

        // Le reste du code reste inchangé
        static unsigned i = 0;
        bool showLogs = (i % LOG_FREQUENCY == 0);

        gladiator->log("Coin Position: (%0.01f, %0.01f)", pos.i, pos.j); // Utilisez pos.i et pos.j


        if (pos.i != -1 && pos.j != -1) {
            MazeSquare* start = gladiator->maze->getNearestSquare();
            MazeSquare* goal = gladiator->maze->getSquare(pos.i, pos.j);
            gladiator->log("Start Square: (%d, %d)", start->i, start->j);
            gladiator->log("Goal Square: (%d, %d)", goal->i, goal->j);
            std::vector<MazeSquare*> path = bfs(start, goal);

            gladiator->log("Path Size: %zu", path.size());
            for (size_t j = 0; j < path.size(); ++j) {
                MazeSquare* square = path[j];
                gladiator->log("Path Element %zu: Square (%d, %d)", j, square->i, square->j);
            }
            static int pathIndex = 0;
        if (pathIndex < static_cast<int>(path.size())) {
            MazeSquare* nextSquare = path[pathIndex];
            // Convertir en coordonnées du monde
            float worldX = nextSquare->i * CELL_SIZE + CELL_SIZE/2;
            float worldY = nextSquare->j * CELL_SIZE + CELL_SIZE/2;
            gladiator->log("Next Square: (%0.01f, %0.01f)", worldX, worldY);
            gladiator->log("Type of worldX: %s", typeid(worldX).name());
            if (aim(gladiator, {worldX, worldY}, showLogs)) {
                pathIndex++; // Avancer au prochain point
            }
        }
        }

        int bombCount = gladiator->weapon->getBombCount();
        if (bombCount > 0) {
            // Vérifier les cases autour du robot pour déposer des bombes de manière stratégique
            const MazeSquare* nearestSquare = gladiator->maze->getNearestSquare();
            for (int di = -1; di <= 1; ++di) {
                for (int dj = -1; dj <= 1; ++dj) {
                    int i = nearestSquare->i + di;
                    int j = nearestSquare->j + dj;

                    if (i >= 0 && j >= 0 && i < 11 && j < 11) {
                        const MazeSquare* square = gladiator->maze->getSquare(i, j);
                        if (square && (square->possession == 2 || square->coin.value > 0)) {
                            // Déposer une bombe si la case appartient à l'équipe adverse ou contient une pièce
                            gladiator->weapon->dropBombs(1);
                            gladiator->log("Drop bomb at strategic location (%d, %d)", i, j);
                            bombCount--;
                            if (bombCount == 0) break;
                        }
                    }
                }
                if (bombCount == 0) break;
            }
        }

        i++;
    }
    delay(10);
    delay(10);
}
#include "gladiator.h"

Gladiator *gladiator;

float kw = 2.0;
float kv = 1.5;
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

void go_to(Position cons, Position pos)
{
    double dx = cons.x - pos.x;
    double dy = cons.y - pos.y;
    double d = sqrt(dx * dx + dy * dy);

    double rho = atan2(dy, dx);
    double angle_diff = reductionAngle(rho - pos.a);

    if (fabs(angle_diff) > 0.1) // Se tourner vers la cible d'abord
    {
        double consw = kw * angle_diff;
        consw = abs(consw) > wlimit ? (consw > 0 ? 1 : -1) * wlimit : consw;
        gladiator->control->setWheelSpeed(WheelAxis::RIGHT, -consw, false);
        gladiator->control->setWheelSpeed(WheelAxis::LEFT, consw, false);
    }
    else if (d > erreurPos) // Une fois orienté, aller tout droit
    {
        double consv = kv * d;
        consv = abs(consv) > vlimit ? (consv > 0 ? 1 : -1) * vlimit : consv;
        gladiator->control->setWheelSpeed(WheelAxis::RIGHT, consv, false);
        gladiator->control->setWheelSpeed(WheelAxis::LEFT, consv, false);
    }
    else
    {
        gladiator->control->setWheelSpeed(WheelAxis::RIGHT, 0, false);
        gladiator->control->setWheelSpeed(WheelAxis::LEFT, 0, false);
    }
}

Position findCoinPosition()
{
    const MazeSquare *nearestSquare = gladiator->maze->getNearestSquare();
    int range = 3;

    for (int di = -range; di <= range; ++di)
    {
        for (int dj = -range; dj <= range; ++dj)
        {
            int i = nearestSquare->i + di;
            int j = nearestSquare->j + dj;

            if (i >= 0 && j >= 0 && i < 11 && j < 11)
            {
                const MazeSquare *square = gladiator->maze->getSquare(i, j);
                if (square && square->coin.value > 0)
                {
                    return square->coin.p;
                }
            }
        }
    }
    return Position{0, 0, 0};
}

void reset()
{
    gladiator->log("Call of reset function");
}

void setup()
{
    gladiator = new Gladiator();
    gladiator->game->onReset(&reset);
}

void loop()
{
    if (gladiator->game->isStarted())
    {
        Position myPosition = gladiator->robot->getData().position;
        Position coinPos = findCoinPosition();

        if (coinPos.x != 0 || coinPos.y != 0)
        {
            go_to(coinPos, myPosition);
        }
        
        if (gladiator->weapon->canDropBombs(1))
        {
            gladiator->weapon->dropBombs(1);
            gladiator->log("Drop bomb");
        }
    }
    delay(10);
}
#include "gladiator.h"
Gladiator *gladiator;

float kw = 1.2;
float kv = 2.f;
float wlimit = 3.f;
float vlimit = 0.6;
float erreurPos = 0.07;
float casesize = 0.25;
double reductionAngle(double x)
{
    x = fmod(x + PI, 2 * PI);
    if (x < 0)
        x += 2 * PI;
    return x - PI;
}
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

void reset()
{
    // fonction de reset:
    // initialisation de toutes vos variables avant le début d'un match
    gladiator->log("Call of reset function"); // GFA 4.5.1
}

void setup()
{
    // instanciation de l'objet gladiator
    gladiator = new Gladiator();
    // enregistrement de la fonction de reset qui s'éxecute à chaque fois avant qu'une partie commence
    gladiator->game->onReset(&reset); // GFA 4.4.1
}

void loop()
{
    if (gladiator->game->isStarted())
    { // tester si un match à déjà commencer
        // code de votre stratégie
        Position myPosition = gladiator->robot->getData().position;
        // Position goal{0.25, 0.25, 0};
        // for(int i = 0; i < 5; i++){
        Position goal{myPosition.x, myPosition.y, 0};
        go_to(goal, myPosition);
        //     go_to(goal, myPosition);
        gladiator->log("Position : %f %f %f", myPosition.x, myPosition.y, myPosition.a); // GFA 4.5.1
        // }
        const MazeSquare *nearestSquare = gladiator->maze->getNearestSquare(); // GFA 4.7.2
        // vérifier s'il a un mur au dessus
        if (nearestSquare->northSquare != NULL)
        {
            MazeSquare *topSquare = nearestSquare->northSquare;
            // calculons les coordonnées du centre de cette case
            float squareSize = gladiator->maze->getSquareSize(); // largeur d'une case GFA 4.7.4
            Position centerCoor;
            // pour calculer les coordonnées x et y il faut récupérer les index i et j de la case
            centerCoor.x = (topSquare->i + 0.25) * squareSize;
            centerCoor.y = (topSquare->j + 0.25) * squareSize;
            // Position goal{centerCoor.x, centerCoor.y, 0};
            go_to(centerCoor,myPosition);
        }

        
        // vérifier s'il a un mur en dessous
        else if (nearestSquare->southSquare != NULL){
            MazeSquare *downSquare = nearestSquare->southSquare;
            // calculons les coordonnées du centre de cette case
            float squareSize = gladiator->maze->getSquareSize(); // largeur d'une case GFA 4.7.4
            Position centerCoor;
            // pour calculer les coordonnées x et y il faut récupérer les index i et j de la case
            centerCoor.x = (downSquare->i + 0.25) * squareSize;
            centerCoor.y = (downSquare->j + 0.25) * squareSize;
            // Position goal{centerCoor.x, centerCoor.y, 0};
            go_to(centerCoor,myPosition);

        }
        
        // vérifier s'il a un mur à gauche
        else if (nearestSquare->westSquare != NULL){
            MazeSquare *leftSquare = nearestSquare->westSquare;
            // calculons les coordonnées du centre de cette case
            float squareSize = gladiator->maze->getSquareSize(); // largeur d'une case GFA 4.7.4
            Position centerCoor;
            // pour calculer les coordonnées x et y il faut récupérer les index i et j de la case
            centerCoor.x = (leftSquare->i + 0.25) * squareSize;
            centerCoor.y = (leftSquare->j + 0.25) * squareSize;
            // Position goal{centerCoor.x, centerCoor.y, 0};
            go_to(centerCoor,myPosition);

        }
        
        // vérifier s'il a un mur à droite
        else if (nearestSquare->eastSquare != NULL){
            MazeSquare *rightSquare = nearestSquare->eastSquare;
            // calculons les coordonnées du centre de cette case
            float squareSize = gladiator->maze->getSquareSize(); // largeur d'une case GFA 4.7.4
            Position centerCoor;
            // pour calculer les coordonnées x et y il faut récupérer les index i et j de la case
            centerCoor.x = (rightSquare->i + 0.25) * squareSize;
            centerCoor.y = (rightSquare->j + 0.25) * squareSize;
            // Position goal{centerCoor.x, centerCoor.y, 0};
            go_to(centerCoor,myPosition);

        }

        // vérifier s'il a un mur à droite
        // else if (nearestSquare->eastSquare == NULL)
        // {
        //     // SI le pointeur pointant vers la case droite est nul, cela veut dire qu'il y a un mur qui empêche de passer
        //     gladiator->log("Il y a un mur à droite de la case où se trouve le robot"); // GFA 4.5.1
        // }
        

    }
    delay(10);
}

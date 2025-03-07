
#ifndef local_maze_hpp
#define local_maze_hpp

#include "gladiator.h"

#define start_max_size 12

struct MazeCell{
    uint8_t i; //ligne
    uint8_t j; //colonne

    MazeCell* Nord;  //pointeur vers la case au nord
    MazeCell* South; //pointeur vers la case au sud
    MazeCell* East; //pointeur vers la case à l'est
    MazeCell* West; //pointeur vers la case à l'ouest

    bool WallNord; //true if there is a wall to the north
    bool WallSouth; //true if there is a wall to the south
    bool WallEast; //true if there is a wall to the east
    bool WallWest; //true if there is a wall to the west

    uint8_t color; //0 if the cell is our color, 1 if it's neutral, 2 if it's the opponent's color
    bool Rocket; //true if there is a rocket in the cell

    MazeCell(uint8_t i, uint8_t j, MazeCell* Nord, MazeCell* South, MazeCell* East, MazeCell* West, bool WallNord, bool WallSouth, bool WallEast, bool WallWest, uint8_t color, bool Rocket);
};


struct LocalMaze{
    uint8_t size; //size of the maze (size x size)
    MazeCell *FirstCell; //pointeur vers la première case du labyrinthe
    Gladiator *gladiator; //pointeur vers le gladiator
    MazeCell* table[start_max_size][start_max_size]; //tableau de pointeurs vers les cases du labyrinthe
    LocalMaze(uint8_t size, Gladiator *gladiator); //constructeur
    void UpdateMaze(uint8_t max_dim); //met à jour le labyrinthe
    void add_wall_dead();
    MazeCell *getSquare(uint8_t i, uint8_t j); //retourne un pointeur vers la case (i, j)
};

#endif
#include <iostream>
#include <string>
#include <vector>
#include <queue>
#include <iostream>
#include <vector>
#include <algorithm>
#include "API.h"

using namespace std;
#define MAZE_SIZE 16
#define MAX_DISTNACE 16 * 16
int max_queue_size = 0;
class Heading
{
public:
    static const int NORTH = 0;
    static const int EAST = 1;
    static const int SOUTH = 2;
    static const int WEST = 3;
};

extern int manhattan_distance[MAZE_SIZE][MAZE_SIZE];
extern int vertical_walls[MAZE_SIZE][MAZE_SIZE - 1];
extern int horizontal_walls[MAZE_SIZE - 1][MAZE_SIZE];

struct Mouse
{
    short int x;
    short int y;
    short int heading;
};
struct Cell
{
    short int x;
    short int y;
};

void init();
void traverse();
Cell getMinCell(vector<Cell> adj_cells);
vector<Cell> get_accessable_cells(short int x, short int y);
void change_deriction(Cell current_cell, Cell min_cell);
void update_walls();
void flood_fill(short int x, short int y);
queue<Cell> flood_fill_queue;
Mouse mouse = {0, 0, Heading::NORTH};


int distance(short int x, short int y) {
    return manhattan_distance[x][y];
}
int distance (Cell cell) {
    return manhattan_distance[cell.x][cell.y];
}
int main(int argc, char *argv[])
{

    init();
    traverse();
}

void flood_fill(short int x, short int y)
{
    Cell current_cell = {x, y};
    if (distance(x, y) == 0)
    {
        return;
    }

    flood_fill_queue.push(current_cell);
    while (!flood_fill_queue.empty())
    {

        Cell considered_cell = flood_fill_queue.front();
        flood_fill_queue.pop();
        vector<Cell> accessaable_cells = get_accessable_cells(considered_cell.x, considered_cell.y);
        Cell min_element = getMinCell(accessaable_cells);

        if (distance(considered_cell) - 1 != distance(min_element))
        {
            if (flood_fill_queue.size() > max_queue_size)
            {
                max_queue_size = flood_fill_queue.size();
            }
            int min_neighbour = distance(min_element);
            API::setText(considered_cell.x, considered_cell.y, to_string(min_neighbour + 1));
            API::setColor(mouse.x, mouse.y, 'o');
            manhattan_distance[considered_cell.x][considered_cell.y] = min_neighbour + 1;
            for (int i = 0; i < accessaable_cells.size(); i++)
            {

                flood_fill_queue.push(accessaable_cells[i]);
            }
        }
    }
}
void update_walls()
{
    int x = mouse.x;
    int y = mouse.y;
    int heading = mouse.heading;
    if (heading == Heading::NORTH)
    {
        if (x > 0)
        {
            if (API::wallLeft())
            {
                vertical_walls[x - 1][y] = API::wallLeft();
                API::setWall(x, y, 'w');
            }
        }
        if (x < 15)
        {
            if (API::wallRight())
            {
                vertical_walls[x][y] = API::wallRight();
                API::setWall(x, y, 'e');
            }
        }
        if (y < 15)
        {
            if (API::wallFront())
            {
                horizontal_walls[x][y] = API::wallFront();
                API::setWall(x, y, 'n');
            }
        }
    }
    else if (heading == Heading::EAST)
    {
        if (x < 15)
        {
            if (API::wallFront())
            {
                vertical_walls[x][y] = API::wallFront();
                API::setWall(x, y, 'e');
            }
        }
        if (y > 0)
        {
            if (API::wallRight())
            {
                horizontal_walls[x][y - 1] = API::wallRight();
                API::setWall(x, y, 's');
            }
        }
        if (y < 15)
        {
            if (API::wallLeft())
            {
                horizontal_walls[x][y] = API::wallLeft();
                API::setWall(x, y, 'n');
            }
        }
    }
    else if (heading == Heading::SOUTH)
    {
        if (x > 0)
        {
            if (API::wallRight())
            {
                vertical_walls[x - 1][y] = API::wallRight();
                API::setWall(x, y, 'w');
            }
        }
        if (x < 15)
        {
            if (API::wallLeft())
            {
                vertical_walls[x][y] = API::wallLeft();
                API::setWall(x, y, 'e');
            }
        }
        if (y > 0)
        {
            if (API::wallFront())
            {
                horizontal_walls[x][y - 1] = API::wallFront();
                API::setWall(x, y, 's');
            }
        }
    }
    else if (heading == Heading::WEST)
    {
        if (x > 0)
        {
            if (API::wallFront())
            {
                vertical_walls[x - 1][y] = API::wallFront();
                API::setWall(x, y, 'w');
            }
        }
        if (y > 0)
        {
            if (API::wallLeft())
            {
                horizontal_walls[x][y - 1] = API::wallLeft();
                API::setWall(x, y, 's');
            }
        }
        if (y < 15)
        {
            if (API::wallRight())
            {
                horizontal_walls[x][y] = API::wallRight();
                API::setWall(x, y, 'n');
            }
        }
    }
}
void init()
{
    API::setText(0, 0, "Start");
    for (int i = 0; i < MAZE_SIZE; i++)
    {
        for (int j = 0; j < MAZE_SIZE; j++)
        {
            API::setText(i, j, to_string(manhattan_distance[i][j]));
        }
    }
}

void traverse()
{
    while (true)
    {
        update_walls();
        
        Cell current_cell = {mouse.x, mouse.y};
        API::setColor(mouse.x, mouse.y, 'o');
        vector<Cell> accessable_cells = get_accessable_cells(mouse.x, mouse.y);
        Cell min_accessable_cell = getMinCell(accessable_cells);

        if (distance(current_cell) > distance(min_accessable_cell))
        {
            change_deriction(current_cell, min_accessable_cell);
            API::moveForward();
            mouse.x = min_accessable_cell.x;
            mouse.y = min_accessable_cell.y;
        }
        else if (manhattan_distance[mouse.x][mouse.y] == 0)
        {
            cerr << "Max queue size: " << max_queue_size << endl;
            return;
        }
        else
        {
            flood_fill(mouse.x, mouse.y);
        }
    }
}

vector<Cell> get_accessable_cells(short int x, short int y)
{
    vector<Cell> accessable_cells;
    if (x > 0 && !vertical_walls[x - 1][y])
    {
        Cell left_cell;
        left_cell.x = x - 1;
        left_cell.y = y;
        accessable_cells.push_back(left_cell);
    }
    if (x < 15 && !vertical_walls[x][y])
    {
        Cell right_cell;
        right_cell.x = x + 1;
        right_cell.y = y;
        accessable_cells.push_back(right_cell);
    }
    if (y > 0 && !horizontal_walls[x][y - 1])
    {
        Cell bottom_cell;
        bottom_cell.x = x;
        bottom_cell.y = y - 1;
        accessable_cells.push_back(bottom_cell);
    }
    if (y < 15 && !horizontal_walls[x][y]) // y = 11, x = 5
    {
        Cell top_cell;
        top_cell.x = x;
        top_cell.y = y + 1;
        accessable_cells.push_back(top_cell);
    }
    return accessable_cells;
}

Cell getMinCell(vector<Cell> cells)
{

    Cell min_cell = cells[0];
    for (int i = 1; i < cells.size(); i++)
    {
        if (distance(cells[i]) < distance(min_cell))
        {
            min_cell = cells[i];
        }
    }
    return min_cell;
}

void change_deriction(Cell current_cell, Cell min_cell)
{
    if (current_cell.x == min_cell.x)
    {
        if (current_cell.y < min_cell.y) // SHOULD GO TO NORTH
        {
            if (mouse.heading == Heading::EAST)
            {
                API::turnLeft();
            }
            else if (mouse.heading == Heading::SOUTH)
            {
                API::turnLeft();
                API::turnLeft();
            }
            else if (mouse.heading == Heading::WEST)
            {
                API::turnRight();
            }
            mouse.heading = Heading::NORTH;
        }
        else if (current_cell.y > min_cell.y) // SHOULD GO TO SOUTH
        {
            if (mouse.heading == Heading::NORTH)
            {
                API::turnLeft();
                API::turnLeft();
            }
            else if (mouse.heading == Heading::EAST)
            {
                API::turnRight();
            }
            else if (mouse.heading == Heading::WEST)
            {
                API::turnLeft();
            }
            mouse.heading = Heading::SOUTH;
        }
    }
    else if (current_cell.y == min_cell.y)
    {
        if (current_cell.x < min_cell.x) // SHOULD GO TO EAST
        {
            if (mouse.heading == Heading::NORTH)
            {
                API::turnRight();
            }
            else if (mouse.heading == Heading::SOUTH)
            {
                API::turnLeft();
            }
            else if (mouse.heading == Heading::WEST)
            {
                API::turnLeft();
                API::turnLeft();
            }
            mouse.heading = Heading::EAST;
        }
        else if (current_cell.x > min_cell.x) // SHOULD GO TO WEST
        {
            if (mouse.heading == Heading::NORTH)
            {
                API::turnLeft();
            }
            else if (mouse.heading == Heading::SOUTH)
            {
                API::turnRight();
            }
            else if (mouse.heading == Heading::EAST)
            {
                API::turnLeft();
                API::turnLeft();
            }
            mouse.heading = Heading::WEST;
        }
    }
}

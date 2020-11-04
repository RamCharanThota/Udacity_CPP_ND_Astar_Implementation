#include <iostream>
#include <vector>
#include <fstream>
#include <string>
#include <sstream>
#include <cmath>
#include <algorithm> //for sorting
using std::abs;
using std::cout;
using std::ifstream;
using std::istringstream;
using std::sort;
using std::string;
using std::vector;

// directional deltas
const int delta[4][2]{{-1, 0}, {0, -1}, {1, 0}, {0, 1}};

enum class State
{
    kEmpty,
    kObstacle,
    kClosed,
    kPath
};

// function to Parse Line and store int elements
vector<State> ParseLine(string &Line)
{
    vector<State> State_vec;
    int n;
    char c;
    istringstream mystream(Line);
    while (mystream >> n >> c && c == ',')
    {
        if (n == 0)
        {
            State_vec.push_back(State::kEmpty);
        }
        else
        {
            State_vec.push_back(State::kObstacle);
        }
    };
    return State_vec;
}

//convert state to formated string
string CellString(State &cell)
{
    switch (cell)
    {
    case State::kObstacle:
        return "⛰️  ";
    case State::kPath:
        return "🚗 ";
    default:
        return "0  ";
    }
}

//function prints the board elements to the console
void PrintBoard(const vector<vector<State>> &b)
{
    for (vector<State> each_row_of_board : b)
    { // loop through each row of board
        for (State ele_of_each_row : each_row_of_board)
        { // loop through each element of row
            cout << CellString(ele_of_each_row) << " ";
        }
        cout << std::endl;
    }
}
// function to read the Board file
vector<vector<State>> ReadBoardFile(const string &path)
{
    vector<vector<State>> ele_board;
    ifstream board_file(path);
    if (board_file)
    {
        string row;
        while (getline(board_file, row))
        {
            vector<State> row_board = ParseLine(row);
            ele_board.push_back(row_board);
        }
    }
    return ele_board;
}

// Heuristic function
int Heuristic(int x1, int y1, int x2, int y2)
{
    // return manhantan distance between two points
    return abs(x2 - x1) + abs(y2 - y1);
}

//AddtoOpen function adds nodes to the open vector
void AddToOpen(int x, int y, int g, int h, vector<vector<State>> &grid_state, vector<vector<int>> &open_node_list)
{
    vector<int> new_node = {x, y, g, h};
    open_node_list.push_back(new_node); // added the node to open_node list
    grid_state[x][y] = State::kClosed;
}

//function to compare f=(g+h) values of two nodes
bool Compare(vector<int> node1, vector<int> node2)
{
    int f_val_node1 = node1[2] + node1[3];
    int f_val_node2 = node2[2] + node2[3];

    if (f_val_node1 > f_val_node2)
    {
        return true;
    }

    return false;
}

// to sort the open_node

void CellSort(vector<vector<int>> *v)
{
    sort(v->begin(), v->end(), Compare);
}

// checkValidCell function to check for obstacles and closed cell

bool CheckValidCell(int x, int y, vector<vector<State>> &grid)
{
    bool on_grid_x = (x >= 0 && x < grid.size());
    bool on_grid_y = (y >= 0 && y < grid[0].size());
    if (on_grid_x && on_grid_y && grid[x][y] == State::kEmpty)
    {
        return true;
    }

    return false;
}

//ExpandNeighbout to explore neightbour nodes
void ExpandNeighbours(vector<int> &cur_node, vector<vector<int>> &open_v, vector<vector<State>> &grid, int goal[2])
{
    //get the current node data
    int x_cur_node = cur_node[0];
    int y_cur_node = cur_node[1];
    int g_cur_node = cur_node[2];
    //int h_cur_node=cur_node[3];

    for (int i = 0; i < 4; i++)
    {
        int x_neig = x_cur_node + delta[i][0];
        int y_neig = y_cur_node + delta[i][1];
        if (CheckValidCell(x_neig, y_neig, grid))
        {
            int g_neig = g_cur_node + 1;
            int h_neig = Heuristic(x_neig, y_neig, goal[0], goal[1]);
            AddToOpen(x_neig, y_neig, g_neig, h_neig, grid, open_v);
        }
    }
}

// A* search function
vector<vector<State>> Search(vector<vector<State>> grid, int start[2], int goal[2])
{
    // write the logic here
    vector<vector<int>> open_n; // empty vector to add open node
    int x_start = start[0];
    int y_start = start[1];
    int x_goal = goal[0];
    int y_goal = goal[1];
    int g = 0;
    int h = Heuristic(x_start, y_start, x_goal, y_goal);
    AddToOpen(x_start, y_start, g, h, grid, open_n);

    while (open_n.size() > 0)
    {
        CellSort(&open_n);
        auto current_node = open_n.back();
        open_n.pop_back();
        x_start = current_node[0];
        y_start = current_node[1];
        grid[x_start][y_start] = State::kPath;
        if (x_start == x_goal && y_start == y_goal)
        {
            return grid;
        }
        ExpandNeighbours(current_node, open_n, grid, goal);
    }

    cout << "No path found!"
         << "\n";
    return std::vector<vector<State>>{};
}

int main()
{
    int init[2]{0, 0};
    int goal[2]{4, 5};
    vector<vector<State>> board;
    board = ReadBoardFile("1.board");
    vector<vector<State>> solution = Search(board, init, goal);
    PrintBoard(solution);
}

/**
 * @file AStar2.cpp
 * @brief Implementation of the A* pathfinding algorithm.
 * 
 * This file contains the implementation of the A* pathfinding algorithm, including
 * the creation of the grid, calculation of heuristic costs, retrieval of neighboring
 * nodes, and printing of the grid and path. The A* algorithm is used to find the
 * shortest path from a start node to a goal node in a grid.
 * 
 * The following classes and functions are defined in this file:
 * - Node: Represents a node in the grid with coordinates, weight, g cost, h cost, and parent.
 * - NodeHash: Hash function for Node pointers used in unordered sets.
 * - NodeEqual: Equality comparison function for Node pointers used in unordered sets.
 * - CompareNodes: Comparison function for Node pointers used in priority queues.
 * - createGrid: Creates a grid of nodes with random weights.
 * - printGrid: Prints the grid with the weights of each node.
 * - calculateH: Calculates the heuristic cost from a node to the goal node.
 * - calculateGAndH: Calculates the g and h costs for a node.
 * - getNeighbors: Retrieves the neighboring nodes of a given node in a grid.
 * - addNeighborsToOpenNodes: Adds the neighbors of the current node to the open nodes priority queue.
 * - printPath: Prints the path from the start node to the goal node.
 * - printGridWithPath: Prints the grid showing the path from the start node to the goal node.
 * - aStar: Implements the A* algorithm to find the shortest path from start to goal.
 * 
 * The main function initializes the grid, sets the start and goal nodes, and runs the A* algorithm.
 * 
 * @note The grid creation using Perlin noise is not implemented and is left as a TODO.
 * 
 * @see AStar2.h for the class and function declarations.
 */

#include <iostream>
#include <vector>
#include <queue>
#include <unordered_set>
#include <cmath>
#include <ctime>
#include "AStar2.h"

Node::Node(int x, int y, int weight, Node* parent)
    : x(x), y(y), weight(weight), g(0), h(0), parent(parent) {}

bool Node::operator==(const Node& other) const {
    return x == other.x && y == other.y;
}
// NodeHash and NodeEqual are now defined in the header file
std::size_t NodeHash::operator()(const Node* node) const {
    return std::hash<int>()(node->x) ^ std::hash<int>()(node->y);
}

bool NodeEqual::operator()(const Node* lhs, const Node* rhs) const {
    return lhs->x == rhs->x && lhs->y == rhs->y;
}

bool CompareNodes::operator()(const Node* lhs, const Node* rhs) const {
    return (lhs->g + lhs->h) > (rhs->g + rhs->h);
}

/**
 * @brief Creates a grid of nodes with random weights.
 * 
 * This function generates a 2D grid of nodes with specified width and height.
 * Each node in the grid is assigned a random weight between the specified minimum
 * and maximum weights.
 * 
 * @param width The width of the grid.
 * @param height The height of the grid.
 * @return std::vector<std::vector<Node>> A 2D vector representing the grid of nodes.
 */
std::vector<std::vector<Node>> createGrid(int width, int height) {
    std::vector<std::vector<Node>> grid;
    int minWeight = 0;
    int maxWeight = 9;

    for (int y = 0; y < height; ++y) {
        std::vector<Node> row;
        for (int x = 0; x < width; ++x) {
            int randomWeight = minWeight + ( std::rand() % ( maxWeight - minWeight + 1 ) );
            row.emplace_back(x, y, randomWeight);
        }
        grid.push_back(row);
    }
    return grid;
}

// TODO implement the grid creation using perlin noise for more interesting weights
// #include <noise/noise.h> required
// std::vector<std::vector<Node>> createGridWithPerlinNoise(int width, int height) {
//     std::vector<std::vector<Node>> grid;
//     noise::module::Perlin perlinModule;

//     for (int y = 0; y < height; ++y) {
//         std::vector<Node> row;
//         for (int x = 0; x < width; ++x) {
//             double noiseValue = perlinModule.GetValue(x * 0.1, y * 0.1, 0.0);
//             int weight = static_cast<int>((noiseValue + 1.0) * 5); // Normalize and scale to 0-10
//             row.emplace_back(x, y, weight);
//         }
//         grid.push_back(row);
//     }
//     return grid;
// }

/** @brief Prints the grid with the weights of each node.
 * 
 * This function takes a 2D grid of nodes and prints the weights of each node
 * in the grid. Each row of the grid is printed on a new line.
 * 
 * @param grid A 2D vector of nodes representing the grid.
 */
void printGrid(const std::vector<std::vector<Node>>& grid) {
    for (const auto& row : grid) {
        for (const auto& node : row) {
            std::cout << node.weight << " ";
        }
        std::cout << std::endl;
    }
}

/** @brief Calculates the heuristic cost from a node to the goal node.
 * 
 * This function calculates the heuristic cost (h) from a given node to the goal node
 * based on the chosen heuristic type. The heuristic can be either Manhattan distance
 * or Euclidean distance.
 * 
 * @param node The node from which to calculate the heuristic cost.
 * @param goal The goal node.
 * @param heuristic The heuristic type to be used (Manhattan or Euclidean).
 * @return int The calculated heuristic cost.
 */
int calculateH(const Node& node, const Node& goal, HeuristicType heuristic) {
    switch (heuristic) {        
        case EUCLIDEAN:
            // Using Euclidean distance as heuristic
            return static_cast<int>(std::sqrt(std::pow(node.x - goal.x, 2) + std::pow(node.y - goal.y, 2)));
        case MANHATTAN:
            [[fallthrough]];
        default:            
            // Using Manhattan distance as heuristic when none are provided
            return abs(node.x - goal.x) + abs(node.y - goal.y);
    }
}

/**
 * @brief Calculates the g and h costs for a node.
 * 
 * Sets the g cost and calculates the h cost for a node based on the goal node
 * and the chosen heuristic.
 * 
 * @param node The node for which to calculate the costs.
 * @param goal The goal node.
 * @param gCost The g cost to be set for the node.
 * @param heuristic The heuristic type to be used (Manhattan or Euclidean).
 */
void calculateGAndH(Node& node, const Node& goal, int gCost, HeuristicType heuristic) {
    node.g = gCost;
    node.h = calculateH(node, goal, heuristic);
}

/**
 * @brief Retrieves the neighboring nodes of a given node in a grid.
 * 
 * This function takes a node and a grid of nodes, and returns a vector of pointers
 * to the neighboring nodes (left, right, up, and down) of the given node.
 * 
 * @param node A pointer to the node for which neighbors are to be found.
 * @param grid A 2D vector of nodes representing the grid.
 * @return std::vector<Node*> A vector of pointers to the neighboring nodes.
 */
std::vector<Node*> getNeighbors(Node* node, std::vector<std::vector<Node>>& grid) {
    std::vector<Node*> neighbors;
    int x = node->x;
    int y = node->y;

    if (x > 0) neighbors.push_back(&grid[y][x - 1]); // Left
    if (x < grid[0].size() - 1) neighbors.push_back(&grid[y][x + 1]); // Right
    if (y > 0) neighbors.push_back(&grid[y - 1][x]); // Up
    if (y < grid.size() - 1) neighbors.push_back(&grid[y + 1][x]); // Down

    return neighbors;
}

/**
 * @brief Adds the neighbors of the current node to the open nodes priority queue.
 * 
 * This function retrieves the neighbors of the current node from the grid and adds them
 * to the open nodes priority queue if they are not already in the closed nodes set. It
 * calculates the g and h costs for each neighbor and sets the current node as their parent.
 * 
 * @param currentNode A pointer to the current node being processed.
 * @param grid A 2D vector of nodes representing the grid.
 * @param goal A reference to the goal node.
 * @param heuristic The heuristic type to be used (Manhattan or Euclidean).
 */
void addNeighborsToOpenNodes(Node* currentNode, std::vector<std::vector<Node>>& grid, const Node& goal, HeuristicType heuristic) {
    auto neighbors = getNeighbors(currentNode, grid);

    for (auto neighbor : neighbors) {
        if (closedNodes.find(neighbor) == closedNodes.end()) {
            int g = neighbor->weight + currentNode->g;
            calculateGAndH(*neighbor, goal, g, heuristic);
            neighbor->parent = currentNode;
            openNodes.push(neighbor);
        }
    }
}

/**
 * @brief Prints the path from the start node to the goal node.
 * 
 * This function recursively prints the path from the start node to the goal node
 * by following the parent pointers of each node. It prints the coordinates (x, y)
 * and the weight of each node in the path.
 * 
 * @param node A pointer to the current node in the path.
 */
void printPath(Node* node) {
    if (node == nullptr) return;
    printPath(node->parent);
    std::cout << "(" << node->x << "," << node->y << "," << node->weight << ") ";
}

/**
 * @brief Prints the grid with the path from the start node to the goal node.
 * 
 * This function takes a 2D grid of nodes and a goal node, and prints the grid
 * with the path from the start node to the goal node marked with asterisks (*).
 * 
 * @param grid A 2D vector of nodes representing the grid.
 * @param goal A pointer to the goal node.
 */
void printGridWithPath(const std::vector<std::vector<Node>>& grid, Node* goal) {
    std::unordered_set<Node*, NodeHash, NodeEqual> pathNodes;
    Node* currentNode = goal;

    printPath(currentNode);
    std::cout << std::endl;

    while (currentNode != nullptr) {
        pathNodes.insert(currentNode);
        currentNode = currentNode->parent;
    }

    for (const auto& row : grid) {
        for (const auto& node : row) {
            if (pathNodes.find(const_cast<Node*>(&node)) != pathNodes.end()) {
                std::cout << "* ";
            } else {
                std::cout << node.weight << " ";
            }
        }
        std::cout << std::endl;
    }
}

/**
 * @brief Implements the A* algorithm to find the shortest path from start to goal.
 * 
 * This function uses the A* search algorithm to find the shortest path from the start node
 * to the goal node in a given grid. It uses a priority queue to explore nodes with the lowest
 * cost (f) first and a set to keep track of visited nodes. The heuristic function can be chosen
 * between Manhattan and Euclidean distances.
 * 
 * @param grid A 2D vector of nodes representing the grid.
 * @param start A reference to the start node.
 * @param goal A reference to the goal node.
 * @param heuristic The heuristic type to be used (Manhattan or Euclidean).
 */
void aStar(std::vector<std::vector<Node>>& grid, Node& start, Node& goal, HeuristicType heuristic) {
    openNodes.push(&start);

    while (!openNodes.empty()) {
        Node* currentNode = openNodes.top();
        openNodes.pop();

        if (*currentNode == goal) {
            std::cout << "Path found: ";
            printGridWithPath(grid, currentNode);
            return;
        }

        closedNodes.insert(currentNode);
        addNeighborsToOpenNodes(currentNode, grid, goal, heuristic);
    }

    std::cout << "No path found!" << std::endl;
}

int main() {
    int width, height;
    int heuristicChoice;

    std::cout << "Enter grid width: ";
    std::cin >> width;
    std::cout << "Enter grid height: ";
    std::cin >> height;
    std::cout << "Choose heuristic (0 for Manhattan, 1 for Euclidean): ";
    std::cin >> heuristicChoice;

    // Seed the random number generator with the current time
    std::srand(static_cast<unsigned>(time(0)));
    auto grid = createGrid(width, height);
    printGrid(grid);

    // Create start and goal nodes at the corners of the grid (bottom right and top left)
    Node start(0, 0, 0);
    Node goal(width - 1, height - 1, 0);

    // Run the A* algorithm with the chosen heuristic
    aStar(grid, start, goal, static_cast<HeuristicType>(heuristicChoice));

    return 0;
}

/* TODO:
 * - Implement grid creation using Perlin noise for more interesting weights
 * - Determine if openNodes already contains a neighbor node then determine the lower f cost
 * - Add a check for unreachable goal node
 * - Add GUI visualization of the grid and path
 */
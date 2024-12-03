#ifndef ASTAR2_H
#define ASTAR2_H

#include <iostream>
#include <vector>
#include <queue>
#include <unordered_set>
#include <cmath>
#include <ctime>

// Enumeration for heuristic types used in the A* algorithm
enum HeuristicType {
    MANHATTAN,  
    EUCLIDEAN   
};

// Class representing a node in the A* algorithm
class Node {
public:
    int x, y;       // Coordinates of the node
    int weight;     // Weight of the node
    int g;          // Cost from the start node to this node
    int h;          // Heuristic cost from this node to the goal
    Node* parent;   // Pointer to the parent node

    // Constructor to initialize a node with coordinates, weight, and optional parent
    Node(int x, int y, int weight, Node* parent = nullptr);

    // Equality operator to compare two nodes based on their coordinates
    bool operator==(const Node& other) const;
};


// Struct to define a hash function for Node pointers
struct NodeHash {
    std::size_t operator()(const Node* node) const;
};

// Struct to define an equality function for Node pointers
struct NodeEqual {
    bool operator()(const Node* lhs, const Node* rhs) const;
};

// Struct to define a comparison function for Node pointers
struct CompareNodes {
    bool operator()(const Node* lhs, const Node* rhs) const;
};

// openNodes is a priority queue of Node pointers, sorted by the sum of g and h (f)
std::priority_queue<Node*, std::vector<Node*>, CompareNodes> openNodes;
// closedNodes is an unordered set of Node pointers
std::unordered_set<Node*, NodeHash, NodeEqual> closedNodes;

#endif // ASTAR2_H
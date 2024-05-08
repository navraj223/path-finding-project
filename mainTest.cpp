#include <iostream>
#include <fstream>
#include <limits>
#include <string>
#include <thread>
#include "Color_Code.h"
#include <algorithm>
#include <stdexcept>
#include <vector>
#include <iostream>
#include <vector>
#include <random>
#include <algorithm>
#include <chrono>
using namespace std;
#define INF numeric_limits<int>::max()


class Graph {
public:
    int V;  // Number of vertices
    vector<vector<int > > adj;  // Adjacency list

    // Constructor initializes an empty graph with V vertices
    Graph(int V) : V(V), adj(V) {}

    // Adds an edge from vertex u to vertex v
    void addEdge(int u, int v) {
        adj[u].push_back(v);
        adj[v].push_back(u);  // For undirected graph
    }

    // Generates a deterministic graph with a specific level of connectivity
    void generateDeterministicGraph(int percentage) {
        int numberOfEdges = (V * (V - 1) / 2) * percentage / 100;  // Calculate total number of edges based on percentage
        int edgesPerVertex = (V - 1) * percentage / 100;  // Edges per vertex

        for (int i = 0; i < V; i++) {
            for (int j = 1; j <= edgesPerVertex; j++) {
                int neighbor = (i + j) % V;
                if (neighbor != i && find(adj[i].begin(), adj[i].end(), neighbor) == adj[i].end()) {
                    addEdge(i, neighbor);
                }
            }
        }
    }
};


class Point {
public:
    int x, y;
    Point(int x = -1, int y = -1) : x(x), y(y) {}

    bool operator==(const Point& other) const {
        return x == other.x && y == other.y;
    }
};


template<typename T, typename PriorityType>
class PriorityQueue {
private:
    vector<pair<T, PriorityType > > heap;

    void heapifyDown(int idx) {
        int left = 2 * idx + 1;
        int right = 2 * idx + 2;
        int smallest = idx;

        if (left < heap.size() && heap[left].second < heap[smallest].second) {
            smallest = left;
        }
        if (right < heap.size() && heap[right].second < heap[smallest].second) {
            smallest = right;
        }
        if (smallest != idx) {
            swap(heap[idx], heap[smallest]);
            heapifyDown(smallest);
        }
    }

    void heapifyUp(int idx) {
        while (idx != 0 && heap[(idx - 1) / 2].second > heap[idx].second) {
            swap(heap[idx], heap[(idx - 1) / 2]);
            idx = (idx - 1) / 2;
        }
    }

public:
    PriorityQueue() {}

    void push(T item, PriorityType priority) {
        heap.emplace_back(item, priority);
        heapifyUp(heap.size() - 1);
    }

    void pop() {
        if (heap.empty()) throw out_of_range("Heap is empty");
        heap[0] = heap.back();
        heap.pop_back();
        heapifyDown(0);
    }

    pair<T, PriorityType> top() const {
        if (heap.empty()) throw out_of_range("Heap is empty");
        return heap[0];
    }

    bool empty() const {
        return heap.empty();
    }
};



class Matrix {
public:
    int **data;
    int rows, cols;

    // Default constructor
    Matrix() : rows(0), cols(0), data(nullptr) {}

    // Constructor with dimensions
    Matrix(int n, int m) : rows(n), cols(m), data(new int*[n]) {
        for (int i = 0; i < n; ++i) {
            data[i] = new int[m]; // Allocate and initialize with zeroes
        }
    }

    // Copy constructor
    Matrix(const Matrix& other) : rows(other.rows), cols(other.cols), data(new int*[other.rows]) {
        for (int i = 0; i < rows; ++i) {
            data[i] = new int[cols];
            std::copy(other.data[i], other.data[i] + cols, data[i]);
        }
    }

    // Assignment operator
    Matrix& operator=(const Matrix& other) {
        if (this != &other) {
            for (int i = 0; i < rows; ++i) {
                delete[] data[i];
            }
            delete[] data;

            rows = other.rows;
            cols = other.cols;
            data = new int*[rows];
            for (int i = 0; i < rows; ++i) {
                data[i] = new int[cols];
                std::copy(other.data[i], other.data[i] + cols, data[i]);
            }
        }
        return *this;
    }

    // Destructor
    ~Matrix() {
        for (int i = 0; i < rows; ++i) {
            delete[] data[i];
        }
        delete[] data;
    }

    // Load data from a file
    void loadFromFile(const std::string& filename) {
        std::ifstream file(filename);
        if (!file) {
            throw std::runtime_error("Failed to open file: " + filename);
        }

        file >> rows >> cols;
        data = new int*[rows];
        for (int i = 0; i < rows; ++i) {
            data[i] = new int[cols];
            for (int j = 0; j < cols; ++j) {
                file >> data[i][j];
            }
        }
        file.close();
    }

    // Display matrix data
    void display() const {
        for (int i = 0; i < rows; ++i) {
            for (int j = 0; j < cols; ++j) {
                std::cout << data[i][j] << " ";
            }
            std::cout << std::endl;
        }
    }
};


class Map {
public:
    Matrix* terrain;
    Point source, destination;

~Map() {
    if (terrain != nullptr && ownsTerrain) {
        delete terrain;
        terrain = nullptr; // Prevent dangling pointer
    }
}
bool ownsTerrain = false; // Add this member to the Map class.

    void loadTerrain(const string& filename) {
        if (terrain != nullptr) {
            delete terrain;
        }
        terrain = new Matrix();
        terrain->loadFromFile(filename);
    }

    void displayMap() const {
        if (terrain != nullptr) {
            terrain->display();
        } else {
            cout << "No terrain loaded." << endl;
        }
    }

    void findPath();
    int getTerrainCost(int terrainType);
    void markPathOnTerrain(int** dist);
    void clearScreen();

};




int Map::getTerrainCost(int terrainType) {
    switch(terrainType) {
        case 0: return 1; // Prairies
        case 1: return 3; // Mountains
        case 2: return 2; // Forests
        case 3: return 1; // Roads
        case -1: return 1; // Path (if applicable)
        default: return INF; // Inaccessible terrain
    }
}

void Map::markPathOnTerrain(int** dist) {
    Point p = destination;
    while (!(p == source)) {
        terrain->data[p.x][p.y] = -1; // Mark as part of the path

        int min_dist = dist[p.x][p.y];
        Point next_step = p;
        int dx[] = {-1, 1, 0, 0};
        int dy[] = {0, 0, -1, 1};

        for (int i = 0; i < 4; ++i) {
            int nx = p.x + dx[i], ny = p.y + dy[i];
            if (nx >= 0 && nx < terrain->rows && ny >= 0 && ny < terrain->cols) {
                if (dist[nx][ny] < min_dist) {
                    min_dist = dist[nx][ny];
                    next_step = Point(nx, ny);
                }
            }
        }

        p = next_step; // Move to the next step in the path
        if (p == source) {
            terrain->data[p.x][p.y] = -1; // Also mark the source as part of the path
        }
    }
}

void Map::clearScreen() {
    #ifdef _WIN32
        system("cls");
    #else
        system("clear");
    #endif
}

void Map::findPath() {
    if (terrain == nullptr || source.x < 0 || source.y < 0 || destination.x >= terrain->rows || destination.y >= terrain->cols) {
        std::cout << "Invalid source or destination." << std::endl;
        return; // Ensure that the source and destination are within bounds and initialized
    }

    // Allocate distance matrix
    int** dist = new int*[terrain->rows];
    for (int i = 0; i < terrain->rows; ++i) {
        dist[i] = new int[terrain->cols];
        std::fill(dist[i], dist[i] + terrain->cols, INF);
    }

    // Priority queue for managing frontier
    PriorityQueue<Point, int> pq;
    pq.push(source, 0);
    dist[source.x][source.y] = 0;

    // Directions for moving in the grid
    int dx[] = {-1, 1, 0, 0};
    int dy[] = {0, 0, -1, 1};

    while (!pq.empty()) {
        Point p = pq.top().first;
        pq.pop();

        for (int i = 0; i < 4; ++i) {
            int nx = p.x + dx[i], ny = p.y + dy[i];
            if (nx >= 0 && nx < terrain->rows && ny >= 0 && ny < terrain->cols && terrain->data[nx][ny] != 1) {  // Check bounds and ensure cell is passable
                int cost = getTerrainCost(terrain->data[nx][ny]);
                if (dist[nx][ny] > dist[p.x][p.y] + cost) {
                    dist[nx][ny] = dist[p.x][p.y] + cost;
                    pq.push(Point(nx, ny), dist[nx][ny]);
                }
            }
        }
    }

    // Optionally mark the path on the terrain for visualization (call to markPathOnTerrain(dist) here if needed)
    //markPathOnTerrain(dist);

    // Clean up distance matrix to prevent memory leaks
    for (int i = 0; i < terrain->rows; ++i) {
        delete[] dist[i];
    }
    delete[] dist;
}



void adjustConnectivity(Matrix& matrix, int numVertices, double connectivityPercentage) {
    std::random_device rd; // Obtain a random number from hardware
    std::mt19937 eng(rd()); // Seed the generator

    int numTotalPossible = numVertices * numVertices; // Since the map is a square
    int numPassable = static_cast<int>(numTotalPossible * connectivityPercentage / 100.0);

    std::vector<std::pair<int, int > > positions;
    for (int i = 0; i < numVertices; i++) {
        for (int j = 0; j < numVertices; j++) {
            positions.emplace_back(i, j);
        }
    }

    std::shuffle(positions.begin(), positions.end(), eng);

    // Set all cells to barriers initially
    for (int i = 0; i < numVertices; i++) {
        for (int j = 0; j < numVertices; j++) {
            matrix.data[i][j] = 1; // Assuming '1' is a barrier
        }
    }

    // Set passable terrain based on connectivity percentage
    for (int i = 0; i < numPassable; i++) {
        auto& pos = positions[i];
        matrix.data[pos.first][pos.second] = 0; // '0' represents passable terrain
    }
}




int main() {
    //edit line below to change # of verticies 
    const int numVertices = 1000; // 10x10 matrix
    //edit the number of values below to change connectivity 
    vector<double> connectivityLevels;
    connectivityLevels.push_back(5.0);
    connectivityLevels.push_back(25.0);
    connectivityLevels.push_back(50.0);
    connectivityLevels.push_back(75.0);
    connectivityLevels.push_back(100.0);

    cout << "Testing for 100 verticies: \n";
    for (double connectivity : connectivityLevels) {
        Matrix* map = new Matrix(numVertices, numVertices);
        adjustConnectivity(*map, numVertices, connectivity);

        Map pathFindingMap;
        pathFindingMap.terrain = map;
        pathFindingMap.ownsTerrain = true;
        pathFindingMap.source = Point(0, 0);  // Initialize source
        pathFindingMap.destination = Point(numVertices - 1, numVertices - 1);  // Initialize destination

        auto start = std::chrono::high_resolution_clock::now();
        pathFindingMap.findPath();
        auto end = std::chrono::high_resolution_clock::now();

        std::chrono::duration<double, std::milli> elapsed = end - start;
        std::cout << "Time taken at " << connectivity << "% connectivity: " << elapsed.count() << " ms" << std::endl;
    }

    

    return 0;
}



#include <iostream>
#include <fstream>
#include <limits>
#include <string>
#include <thread>
#include "Color_Code.h"
#include <algorithm>
#include <stdexcept>
using namespace std;
#define INF numeric_limits<int>::max()  // Define the maximum integer value used for representing infinite distances.

// Class representing a point in two-dimensional space.
class Point {
public:
    int x, y;  // X and Y coordinates of the point.
    Point(int x = -1, int y = -1) : x(x), y(y) {}
    bool operator==(const Point& other) const {
        return x == other.x && y == other.y;
    }
};

// Custom "priority queue" class template for elements of type T with priorities of type PriorityType.
template<typename T, typename PriorityType>
class PriorityQueue {
private:
    pair<T, PriorityType>* heap;
    int capacity;
    int size;

    // Function to expand the capacity of the queue when it is full.
    void expandCapacity() {
        PriorityType newCapacity = capacity * 2 + 1;
        auto newHeap = new pair<T, PriorityType>[newCapacity];
        for (int i = 0; i < size; ++i) {
            newHeap[i] = heap[i];
        }
        delete[] heap;
        heap = newHeap;
        capacity = newCapacity;
    }

    // Function to reorder the heap downwards starting from index idx to maintain the heap property.
    void heapifyDown(int idx) {
        int left = 2 * idx + 1;
        int right = 2 * idx + 2;
        int smallest = idx;

        if (left < size && heap[left].second < heap[smallest].second) {
            smallest = left;
        }
        if (right < size && heap[right].second < heap[smallest].second) {
            smallest = right;
        }
        if (smallest != idx) {
            swap(heap[idx], heap[smallest]);
            heapifyDown(smallest);
        }
    }

    // Function to reorder the heap upwards starting from index idx to maintain the heap property.
    void heapifyUp(int idx) {
        while (idx != 0 && heap[(idx - 1) / 2].second > heap[idx].second) {
            swap(heap[idx], heap[(idx - 1) / 2]);
            idx = (idx - 1) / 2;
        }
    }

public:
    // Constructor to initialize an empty priority queue.
    PriorityQueue() : heap(nullptr), capacity(0), size(0) {}
    ~PriorityQueue() {
        delete[] heap;
    }

    // Method to add an element 'item' with priority 'priority' into the queue.
    void push(T item, PriorityType priority) {
        if (size == capacity) {
            expandCapacity();
        }
        heap[size] = make_pair(item, priority);
        heapifyUp(size);
        size++;
    }
    
    // Method to remove the element with the highest priority from the queue.
    void pop() {
        if (size == 0) throw out_of_range("Heap is empty");
        heap[0] = heap[--size];
        heapifyDown(0); 
    }
    // Method to return the element with the highest priority from the queue.
    pair<T, PriorityType> top() const {
        if (size == 0) throw out_of_range("Heap is empty");
        return heap[0];
    }
    // Method to check if the queue is empty.
    bool empty() const {
        return size == 0;
    }
};
// Defines a matrix structure for managing 2D grid data.
class Matrix {
public:
    int **data;
    int rows, cols;

    // Constructor initializes a matrix of specified size or empty if not specified.
    Matrix(int n = 0, int m = 0) : rows(n), cols(m), data(nullptr) {
        if (rows > 0 && cols > 0) {
            data = new int*[rows];
            for (int i = 0; i < rows; ++i) {
                data[i] = new int[cols];
            }
        }
    }
    
    // Destructor to free the allocated memory for the matrix.
    ~Matrix() {
        for (int i = 0; i < rows; ++i) {
            delete[] data[i];
        }
        delete[] data;
    }
    
    // Loads matrix data from a file.
    void loadFromFile(const string& filename);
    
     // Displays the matrix as a visually formatted grid to the console.
    void display() const;
};

// Loads matrix data from a specified file, replacing any existing data.
void Matrix::loadFromFile(const string& filename) {
    ifstream file(filename);
    if (!file.is_open()) {
        cerr << "Failed to open file: " << filename << endl;
        return;
    }

    for (int i = 0; i < rows; ++i) {
        delete[] data[i];
    }
    delete[] data;

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

// Displays the matrix with each element formatted based on its value (e.g., as terrain types).
void Matrix::display() const {
    for (int i = 0; i < rows; ++i) {
        for (int repeatRow = 0; repeatRow < 2; ++repeatRow) {
            for (int k = 0; k < cols; ++k) {
                if (data[i][k] == 0) {
                    cout << ESC "42;30m  " RESET;
                } else if (data[i][k] == 1) {
                    cout << ESC "47;30m^^" RESET;
                } else if (data[i][k] == 2) {
                    cout << ESC "42;30mTT" RESET;
                } else if (data[i][k] == 3) {
                    cout << ESC "43;30m##" RESET;
                } else if (data[i][k] == -1) {
                    cout << ESC "44;37m**" RESET;
                } else {
                    cout << "   ";
                }
            }
            cout << endl;
        }
    }
}

// Defines a map for pathfinding and visualization purposes.
class Map {
public:
    Matrix* terrain; // Pointer to the matrix representing the terrain.
    Point source, destination; // Start and end points for pathfinding.

    // Constructor initializes the map with no terrain and default source and destination points.
    Map() : terrain(nullptr) {}

    // Destructor cleans up allocated terrain.
    ~Map() { delete terrain; }

    // Loads terrain data from a file into the matrix.
    void loadTerrain(const string& filename) {
        if (terrain != nullptr) {
            delete terrain;
        }
        terrain = new Matrix();
        terrain->loadFromFile(filename);
    }
    // Function to set the source and destination points for pathfinding based on user input.
    void setSourceDestination(int sx, int sy, int dx, int dy) {
        source.x = sx;
        source.y = sy;
        destination.x = dx;
        destination.y = dy;
    }

    void displayMap() const {
        if (terrain != nullptr) {
            terrain->display();
        } else {
            cout << "No terrain loaded." << endl;
        }
    }
    // Declares methods relavent to pathfinding.
    void findPath();
    int getTerrainCost(int terrainType);
    void markPathOnTerrain(int** dist);
    void clearScreen();
};

// Returns the cost of traversing a specific terrain type.
int Map::getTerrainCost(int terrainType) {
    switch(terrainType) {
        case 0: return 1;
        case 1: return 3;
        case 2: return 2;
        case 3: return 1;
        case -1: return 1;
        default: return INF;
    }
}

// Marks the path on the terrain matrix based on distances computed.
void Map::markPathOnTerrain(int** dist) {
    Point p = destination;
    while (!(p == source)) {
        terrain->data[p.x][p.y] = -1;

        int min_dist = dist[p.x][p.y];
        Point next_step = p;
        int dx[] = {-1, 1, 0, 0};
        int dy[] = {0, 0, -1, 1};

        for (int i = 0; i < 4; ++i) { // Evaluates all possible moves to find the next step with the lowest distance.
            int nx = p.x + dx[i], ny = p.y + dy[i];
            if (nx >= 0 && nx < terrain->rows && ny >= 0 && ny < terrain->cols) {
                if (dist[nx][ny] < min_dist) {
                    min_dist = dist[nx][ny];
                    next_step = Point(nx, ny);
                }
            }
        }

        p = next_step;
        if (p == source) {
            terrain->data[p.x][p.y] = -1;
        }
    }
}

// Clears the console screen based on the operating system.
void Map::clearScreen() {
    #ifdef _WIN32
        system("cls");
    #else
        system("clear");
    #endif
}

// Finds the shortest path from the source to the destination using the custom "priority queue" while following the logic of Dijkstra's Algorithm.
void Map::findPath() {
    if (terrain == nullptr) return;

    int** dist = new int*[terrain->rows];
    for (int i = 0; i < terrain->rows; ++i) {
        dist[i] = new int[terrain->cols];
        std::fill(dist[i], dist[i] + terrain->cols, INF);
    }

    PriorityQueue<Point, int> pq; // Custom "Priority queue" to manage the exploration front.
    pq.push(source, 0);
    dist[source.x][source.y] = 0;

    int dx[] = {-1, 1, 0, 0};
    int dy[] = {0, 0, -1, 1};

    while (!pq.empty()) { // Explore the terrain until there are no more points to process.
        Point p = pq.top().first;
        pq.pop();

        for (int i = 0; i < 4; ++i) { // Check all adjacent points.
            int nx = p.x + dx[i], ny = p.y + dy[i];
            if (nx >= 0 && nx < terrain->rows && ny >= 0 && ny < terrain->cols) {
                int cost = getTerrainCost(terrain->data[nx][ny]);
                if (dist[nx][ny] > dist[p.x][p.y] + cost) {
                    dist[nx][ny] = dist[p.x][p.y] + cost;
                    pq.push(Point(nx, ny), dist[nx][ny]);
                }
            }
        }
    }

    markPathOnTerrain(dist); // After finding paths, mark the final path on the map.

    for (int i = 0; i < terrain->rows; ++i) {
        delete[] dist[i];
    }
    delete[] dist;
}

// The main function: the entry point of the program.
int main() {
    cout << "Map Visualization and Pathfinding Program\n";

    string mapFilename;
    int sx, sy, dx, dy;

    // Display available maps
    cout << "Available maps: Map0.txt (5x5), Map1.txt (10x10), Map2.txt (10x10), Map3.txt (10x10), Map4.txt (10x10), Map5.txt (10x10)\n";
    cout << "Enter the map file name: ";
    cin >> mapFilename;

    // Check if the selected file is valid
    ifstream testFile(mapFilename);
    if (!testFile.good()) {
        cerr << "Error: File not found or unable to open!\n";
        return 1;
    }
    testFile.close();

    // Get source and destination from the user
    cout << "Enter source coordinates (x y): ";
    cin >> sx >> sy;
    cout << "Enter destination coordinates (x y): ";
    cin >> dx >> dy;

    Map map;
    map.loadTerrain(mapFilename); // Load the map data from the specified file
    map.setSourceDestination(sx, sy, dx, dy); // Set the source and destination
    map.findPath(); // Perform pathfinding
    map.displayMap(); // Display the map with the path

    return 0;
}
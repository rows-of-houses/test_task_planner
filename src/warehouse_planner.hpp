#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <vector>

#define X_START 0
#define Y_START 0
#define X_END 100
#define Y_END 100

#define POINT_TIME 10.0
#define SPEED 2.0

template <typename coord_type, typename fine_type>
class WaypointTemplate {
private:
    coord_type x, y;
    fine_type fine;

public:
    WaypointTemplate() {};

    WaypointTemplate(coord_type x_, coord_type y_, fine_type fine_)
        : x(x_)
        , y(y_)
        , fine(fine_)
    {
    }

    coord_type getX() const { return x; }

    coord_type getY() const { return y; }

    fine_type getFine() const { return fine; }

    double getProcessingTime() const { return POINT_TIME; }

    static double getTravelTime(WaypointTemplate p1, WaypointTemplate p2);

    friend std::istream& operator>>(std::istream& in,
        WaypointTemplate& waypoint)
    {
        in >> waypoint.x >> waypoint.y >> waypoint.fine;
        return in;
    }
};

using Waypoint = WaypointTemplate<int, int>;

class Graph {
private:
    struct Edge {
        int parent, child;
        double weight;
    };

    struct Vertex {
        Waypoint wp;
    };

    std::vector<Edge> edges;
    std::vector<Vertex> vertices;

    double getEdgeWeight(Waypoint parent_waypoint, Waypoint child_waypoint) const;

public:
    Graph(std::vector<Waypoint> waypoints);

    int getNumVertices() const { return static_cast<int>(vertices.size()); }

    int getNumEdges() const { return static_cast<int>(edges.size()); }

    std::vector<Vertex> getVertices() const { return vertices; }

    std::vector<Edge> getEdges() const { return edges; }
};

class SingleSourceShortestPath {
public:
    virtual double getStartCost(const Graph& graph) = 0;

    virtual double findShortestPathCost(const Graph& graph, int start_index,
        int end_index)
        = 0;

    // virtual std::vector<int> findShortestPath(const Graph& graph, int start_index,
    //     int end_index) {};
};

class BellmanFordAlgorithm : SingleSourceShortestPath {
private:
    double getStartCost(const Graph& graph) override;

public:
    double findShortestPathCost(const Graph& graph, int start_index, int end_index) override;
};

double FindMinCost(const std::vector<Waypoint>& waypoints);
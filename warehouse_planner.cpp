#include <warehouse_planner.hpp>

template <typename coord_type, typename fine_type>
double WaypointTemplate<coord_type, fine_type>::getTravelTime(WaypointTemplate<coord_type, fine_type> p1, WaypointTemplate<coord_type, fine_type> p2)
{
    return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y)) / SPEED;
}

Graph::Graph(std::vector<Waypoint> waypoints)

{
    int num_vertices = waypoints.size();

    for (const auto& waypoint : waypoints) {
        struct Vertex v = { .wp = waypoint };
        vertices.push_back(v);
    }

    for (int i = 0; i < num_vertices; ++i) {
        for (int j = i + 1; j < num_vertices; ++j) {
            auto weight = getEdgeWeight(waypoints[i], waypoints[j]);
            struct Edge e = { .parent = i, .child = j, .weight = weight };
            edges.push_back(e);
        }
    }
};

double Graph::getEdgeWeight(Waypoint parent_waypoint, Waypoint child_waypoint) const
{
    return (Waypoint::getTravelTime(parent_waypoint, child_waypoint) - child_waypoint.getFine() + child_waypoint.getProcessingTime());
}

double BellmanFordAlgorithm::getStartCost(const Graph& graph)
{
    double start_cost = 0.0;
    for (auto vertex : graph.getVertices()) {
        start_cost += vertex.wp.getFine();
    }
    return start_cost;
}

double BellmanFordAlgorithm::findShortestPathCost(const Graph& graph, int start_index, int end_index)
{
    std::vector<double> cost_array(graph.getNumVertices(),
        std::numeric_limits<double>::max());

    cost_array.at(start_index) = getStartCost(graph);

    for (int i = 0; i < graph.getNumVertices() - 1; ++i) {
        for (auto edge : graph.getEdges()) {
            if (cost_array[edge.parent] != std::numeric_limits<double>::max() && cost_array[edge.child] > cost_array[edge.parent] + edge.weight) {
                cost_array[edge.child] = cost_array[edge.parent] + edge.weight;
            }
        }
    }
    return cost_array.at(end_index);
}

double FindMinCost(const std::vector<Waypoint>& waypoints)
{
    auto graph = Graph(waypoints);
    auto bf = BellmanFordAlgorithm();
    auto min_cost = bf.findShortestPathCost(graph, 0, waypoints.size() - 1);
    return min_cost;
}

int main()
{
    std::cout << std::fixed;
    std::cout << std::setprecision(3);

    while (true) {
        int n;
        std::cin >> n;
        if (n == 0) {
            break;
        }

        std::vector<Waypoint> waypoints(n + 2);
        waypoints[0] = Waypoint(X_START, Y_START, 0);
        waypoints[n + 1] = Waypoint(X_END, Y_END, 0);
        for (int i = 1; i < n + 1; ++i) {
            std::cin >> waypoints[i];
        }

        auto min_cost = FindMinCost(waypoints);
        std::cout << min_cost << std::endl;
    }
    return 0;
}
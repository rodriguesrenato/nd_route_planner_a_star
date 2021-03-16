#include "route_planner.h"
#include <algorithm>
using std::cout;
// Iinitialize Route Planner and finde the closest node from the user start and end coordinates.
RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y) : m_Model(model)
{
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);

    // Debug nodes values
    std::cout << "\n Closest start_node: " << start_node->x << "," << start_node->y << "\t| user input start: " << start_x << "," << start_y << std::endl;
    std::cout << "Closest end_node: " << end_node->x << "," << end_node->y << "\t| user input end: " << end_x << "," << end_y << std::endl;
}

// Calculate the distance from the given node to the end_node
float RoutePlanner::CalculateHValue(const RouteModel::Node *node)
{
    return node->distance(*end_node);
}

// Find the neighbors of given node and populate it's neighbors list.
// Iterate over each neigbour: set current node as parent,
// calculate g and h, mark as visited and at last, added it in open_list.
void RoutePlanner::AddNeighbors(RouteModel::Node *current_node)
{
    current_node->FindNeighbors();
    for (RouteModel::Node *n : current_node->neighbors)
    {
        n->parent = current_node;
        n->h_value = CalculateHValue(n);
        n->g_value += current_node->g_value + current_node->distance(*n);
        n->visited = true;
        open_list.push_back(n);
    }
}

// A comparison functor for the NextNode() sort function
bool Compare(const RouteModel::Node *n1, const RouteModel::Node *n2)
{
    return (n1->g_value + n1->h_value) > (n2->g_value + n2->h_value);
}

// Sort the open_list, get the node of lowest g+h value, remove it from open_list and return it.
RouteModel::Node *RoutePlanner::NextNode()
{
    std::sort(open_list.begin(), open_list.end(), Compare);
    auto next_node = open_list.back();
    open_list.pop_back();
    return next_node;
}

// Construct the path from end_node to start_node by creating a vector following node's parent attribute
std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node)
{
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    // Sum up the distance of the current node to its parent node
    // Add current node the the path vector and set it's parent as the current node for next loop iteration.
    while (current_node != start_node)
    {
        distance += current_node->distance(*current_node->parent);
        path_found.push_back(*current_node);
        current_node = current_node->parent;
    }

    // Add start node to the path found
    path_found.push_back(*start_node);

    // Reverse vector order
    std::reverse(path_found.begin(), path_found.end());

    // Multiply the distance by the scale of the map to get meters.
    distance *= m_Model.MetricScale();
    return path_found;
}

// Perform an A* Search to find the path between start_node and end_node
void RoutePlanner::AStarSearch()
{
    // Mark start_node as current_node, add it to open_list and mark it as visited
    RouteModel::Node *current_node = start_node;
    start_node->visited = true;
    open_list.push_back(current_node);

    // Iterate over the lowest g+h value of the nodes in open_list,
    // check if it's the end node to construct the final path,
    // otherwise add it's neighbors to the open_list
    do
    {
        current_node = NextNode();
        if (current_node == end_node)
        {
            m_Model.path = ConstructFinalPath(current_node);
            std::cout << "\nPath found!" << std::endl;
            return;
        }
        AddNeighbors(current_node);
    } while (open_list.size() > 0);

    // If open_list gets empty before fine the end_node, the final path couldn't be found
    std::cout << "\nCan't find a path!" << std::endl;
    m_Model.path = std::vector<RouteModel::Node>{};
}
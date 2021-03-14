#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y) : m_Model(model)
{
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);
}

float RoutePlanner::CalculateHValue(RouteModel::Node const *node)
{
    return node->distance(*end_node);
}

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node)
{
    current_node->FindNeighbors();
    for (RouteModel::Node *n : current_node->neighbors)
    {
        if (!n->visited)
        {
            n->parent = current_node;
            n->h_value = CalculateHValue(n);
            n->g_value += current_node->g_value + current_node->distance(*n);
            n->visited = true;
            open_list.push_back(n);
        }
        //what if the sencodary path is shorter than the one already visited?? TODOTODO
    }
}

bool Compare(const RouteModel::Node *n1, const RouteModel::Node *n2)
{
    return (n1->g_value + n1->h_value) > (n2->g_value + n2->h_value);
}

RouteModel::Node *RoutePlanner::NextNode()
{
    std::sort(open_list.begin(), open_list.end(), Compare);
    auto next_node = open_list.back();
    open_list.pop_back();
    return next_node;
}

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node)
{
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

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


void RoutePlanner::AStarSearch()
{
    RouteModel::Node *current_node = start_node;
    start_node->visited = true;
    do
    {
        AddNeighbors(current_node);
        current_node = NextNode();
        if (current_node == end_node)
        {
            m_Model.path = ConstructFinalPath(current_node);
            std::cout << "\nPath found!" << std::endl;
            return;
        }
    } while (open_list.size() > 0);

    std::cout << "\nCan't find a path!" << std::endl;
    m_Model.path = std::vector<RouteModel::Node>{};
}
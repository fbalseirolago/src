#include "route_planner.h"
#include <algorithm>
#include <iostream>
RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    this->start_node = &m_Model.FindClosestNode(start_x, start_y);
    this->end_node = &m_Model.FindClosestNode(end_x, end_y);
}

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*(this->end_node));
}

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    // Fill out the neighbors attribute in the current node
    current_node->FindNeighbors();

    // Iterate over the neighbors
    for (auto &node : current_node->neighbors)
    {
        node->parent = current_node;
        node->h_value = this->CalculateHValue(node);
        node->g_value = current_node->g_value + node->distance(*current_node);

        if(!(node->visited))
        {
            node->visited = true;
            this->open_list.emplace_back(node);
        }
    }
}

static bool compare(RouteModel::Node *node1, RouteModel::Node *node2)
{
    float dist1 = node1->g_value + node1->h_value;
    float dist2 = node2->g_value + node2->h_value;

    return (dist1 > dist2);
}

RouteModel::Node *RoutePlanner::NextNode() {
    // Sort the open list to have the lowest value at the end of the list
    sort(this->open_list.begin(), this->open_list.end(), compare);

    // Create a pointer to the node in the list with the lowest sum.
    RouteModel::Node *next = (this->open_list).back();

    // Remove the last element
    (this->open_list).pop_back();

    return next;
}


std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;
    while(current_node->parent != nullptr)
    {
        path_found.emplace_back(*current_node);
        distance += current_node->distance(*(current_node->parent));
        current_node = current_node->parent;
    }

    path_found.emplace_back(*current_node);

    // Reverse the order of the path
    std::reverse(path_found.begin(), path_found.end());

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}


// TODO 7: Write the A* Search algorithm here.
// Tips:
// - Use the AddNeighbors method to add all of the neighbors of the current node to the open_list.
// - Use the NextNode() method to sort the open_list and return the next node.
// - When the search has reached the end_node, use the ConstructFinalPath method to return the final path that was found.
// - Store the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    this->start_node->visited = true;
    this->start_node->h_value = this->CalculateHValue(this->start_node);
    this->start_node->g_value = 0;
    this->AddNeighbors(this->start_node);

     while((this->open_list).size() > 0)
    {
        current_node = this->NextNode();
        this->AddNeighbors(current_node);
        if (current_node == end_node)
        {
            this->m_Model.path = this->ConstructFinalPath(current_node);
            break;
        }

    }

}
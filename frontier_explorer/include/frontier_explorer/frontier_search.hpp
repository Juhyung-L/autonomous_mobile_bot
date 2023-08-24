#ifndef FRONTIER_EXPLORER__FRONTIER_SEARCH_HPP_
#define FRONTIER_EXPLORER__FRONTIER_SEARCH_HPP_

#include <vector>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "nav2_util/lifecycle_node.hpp"

namespace frontier_explorer
{

static constexpr unsigned char NO_INFORMATION = 255;
static constexpr unsigned char LETHAL_OBSTACLE = 254;
static constexpr unsigned char INSCRIBED_INFLATED_OBSTACLE = 253;
static constexpr unsigned char MAX_NON_OBSTACLE = 252;
static constexpr unsigned char FREE_SPACE = 0;

struct Frontiers
{
    double min_distance;
    double cost;
    geometry_msgs::msg::Point centriod;
    std::vector<geometry_msgs::msg::Point> points;
};

class FrontierSearch
{
public:
    FrontierSearch(const std::shared_ptr<nav2_util::LifecycleNode>& node);
    std::vector<Frontiers> searchFrontiers(const geometry_msgs::msg::Pose& start_pose);
    void updateMap(const nav2_msgs::msg::Costmap::SharedPtr& costmap);
    bool goalOnBlacklist(const geometry_msgs::msg::Point& goal);
    void visualizeFrontiers(const std::vector<Frontiers>& frontiers);
    void addToBlacklist(const geometry_msgs::msg::Point& frontiers);

    rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub; 
private:
    unsigned char* map_data;
    unsigned int size_x, size_y;
    float resolution;
    geometry_msgs::msg::Pose map_origin;
    std::string frame_id;
    std::shared_ptr<nav2_util::LifecycleNode> node;

    double distance_weight;
    double size_weight;

    std::vector<geometry_msgs::msg::Point> frontiers_black_list;
    size_t prev_marker_count;

    bool isNewFrontier(unsigned int idx);
    Frontiers buildNewFrontiers(unsigned int start_idx, std::vector<bool>& visited,
                                                const geometry_msgs::msg::Pose& robot_pose);
    double frontierCost(const Frontiers& frontier);
    
    unsigned int cellsToIndex(unsigned int map_x, unsigned int map_y)
    {
        return map_y * size_x + map_x;
    }

    void indexToCells(unsigned int index, unsigned int& mx, unsigned int& my) const
    {
        my = index / size_x;
        mx = index - (my * size_x);
    }

    bool worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my) const
    {
        // coordinate is outside the map
        if (wx < map_origin.position.x || wy < map_origin.position.y)
        {
            return false;
        }
        // convert from world coordinate to map index
        mx = static_cast<unsigned int>((wx - map_origin.position.x ) / resolution);
        my = static_cast<unsigned int>((wy - map_origin.position.y) / resolution);

        // check if map indices are inside the map
        if (mx < size_x && my < size_y)
        {
            return true;
        }
        return false;
    }

    void mapToWorld(unsigned int mx, unsigned int my, double& wx, double& wy) const
    {
        wx = map_origin.position.x + (mx + 0.5) * resolution;
        wy = map_origin.position.y + (my + 0.5) * resolution;
    }

    std::vector<unsigned int> nhood4(unsigned int idx)
    {
        // get 4-connected neighbourhood indexes, check for edge of map
        std::vector<unsigned int> out;

        if (idx > size_x * size_y - 1) {
            RCLCPP_WARN(rclcpp::get_logger("autonomous_explorer_node"), "Evaluating nhood for offmap point");
            return out;
        }

        // index is not at the left edge of the map
        // if idx % size_x == 0 then the idx is at the left edge of the map
        if (idx % size_x > 0) {
            out.push_back(idx - 1); // add cell to the left
        }
        // index is not at the right edge of the map
        // if idx % size_x == size_x - 1 then the idx is at the right edge of the map
        // cuz if size_x = 10 and idx = 29 (right edge of third column since index starts from 0) then
        // 29 % 10 = 9 == size_x - 1
        if (idx % size_x < size_x - 1) {
            out.push_back(idx + 1); // add cell to the right
        }
        // idx is not in the first row (no cells above)
        // if idx = 5 and size_x = 10 then that idx is the 6th cell, which is in the first row (no cell above)
        if (idx >= size_x) {
            out.push_back(idx - size_x); // add upper cell
        }
        // idx is not in the last row (no cells below)
        // size_x * (size_y - 1) = idx of the right most cell in the second last row
        // if idx exceeds this number, it is in the last row (no cells below it)
        if (idx < size_x * (size_y - 1)) {
            out.push_back(idx + size_x); // add lower cell
        }
        //
        //     X
        //   X O X
        //     X
        //
        // O - input cell
        // X - potential cells to add to out
        // out of X, only add cells that are within the map
        return out;
    }

    std::vector<unsigned int> nhood8(unsigned int idx)
    {
        // get 8-connected neighbourhood indexes, check for edge of map
        std::vector<unsigned int> out = nhood4(idx);

        if (idx > size_x * size_y - 1) {
            return out;
        }

        if (idx % size_x > 0 && idx >= size_x) {
            out.push_back(idx - 1 - size_x); // add upper left cell
        }
        if (idx % size_x > 0 && idx < size_x * (size_y - 1)) {
            out.push_back(idx - 1 + size_x); // add lower left cell
        }
        if (idx % size_x < size_x - 1 && idx >= size_x) {
            out.push_back(idx + 1 - size_x); // add upper right cell
        }
        if (idx % size_x < size_x - 1 && idx < size_x * (size_y - 1)) {
            out.push_back(idx + 1 + size_x); // add lower right cell
        }
        //
        //   X   X
        //     O
        //   X   X
        //
        // O - input cell
        // X - potential cells to add to out
        // out of X, only add cells that are within the map
        return out;
    }
};

} // namespace frontier_search
#endif
#ifndef FRONTIER_SEARCH_H
#define FRONTIER_SEARCH_H

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

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
    FrontierSearch(rclcpp::Node& node);
    std::vector<Frontiers> searchFrontiers(const geometry_msgs::msg::Pose& startPose);
    void updateMap(const nav2_msgs::msg::Costmap::SharedPtr& costmap);
    bool goalOnBlacklist(const geometry_msgs::msg::Point& goal);
    void visualizeFrontiers(const std::vector<Frontiers>& frontiers);
    void addToBlacklist(const geometry_msgs::msg::Point& frontiers);
private:
    unsigned char* mapData;
    unsigned int sizeX, sizeY;
    float resolution;
    geometry_msgs::msg::Pose mapOrigin;
    std::string frameId;
    rclcpp::Node& node;

    double distanceWeight;
    double sizeWeight;

    std::vector<geometry_msgs::msg::Point> frontiersBlacklist;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markerPub;
    size_t prevMarkerCount;

    bool isNewFrontier(unsigned int idx);
    Frontiers buildNewFrontiers(unsigned int startIdx, std::vector<bool>& visited,
                                                const geometry_msgs::msg::Pose& robotPose);
    double frontierCost(const Frontiers& frontier);
    
    unsigned int cellsToIndex(unsigned int mapX, unsigned int mapY)
    {
        return mapY * sizeX + mapX;
    }

    void indexToCells(unsigned int index, unsigned int& mx, unsigned int& my) const
    {
        my = index / sizeX;
        mx = index - (my * sizeX);
    }

    bool worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my) const
    {
        // coordinate is outside the map
        if (wx < mapOrigin.position.x || wy < mapOrigin.position.y)
        {
            return false;
        }
        // convert from world coordinate to map index
        mx = static_cast<unsigned int>((wx - mapOrigin.position.x ) / resolution);
        my = static_cast<unsigned int>((wy - mapOrigin.position.y) / resolution);

        // check if map indices are inside the map
        if (mx < sizeX && my < sizeY)
        {
            return true;
        }
        return false;
    }

    void mapToWorld(unsigned int mx, unsigned int my, double& wx, double& wy) const
    {
        wx = mapOrigin.position.x + (mx + 0.5) * resolution;
        wy = mapOrigin.position.y + (my + 0.5) * resolution;
    }

    std::vector<unsigned int> nhood4(unsigned int idx)
    {
        // get 4-connected neighbourhood indexes, check for edge of map
        std::vector<unsigned int> out;

        if (idx > sizeX * sizeY - 1) {
            RCLCPP_WARN(rclcpp::get_logger("autonomous_explorer_node"), "Evaluating nhood for offmap point");
            return out;
        }

        // index is not at the left edge of the map
        // if idx % sizeX == 0 then the idx is at the left edge of the map
        if (idx % sizeX > 0) {
            out.push_back(idx - 1); // add cell to the left
        }
        // index is not at the right edge of the map
        // if idx % sizeX == size_x - 1 then the idx is at the right edge of the map
        // cuz if sizeX = 10 and idx = 29 (right edge of third column since index starts from 0) then
        // 29 % 10 = 9 == sizeX - 1
        if (idx % sizeX < sizeX - 1) {
            out.push_back(idx + 1); // add cell to the right
        }
        // idx is not in the first row (no cells above)
        // if idx = 5 and sizeX = 10 then that idx is the 6th cell, which is in the first row (no cell above)
        if (idx >= sizeX) {
            out.push_back(idx - sizeX); // add upper cell
        }
        // idx is not in the last row (no cells below)
        // size_x * (size_y - 1) = idx of the right most cell in the second last row
        // if idx exceeds this number, it is in the last row (no cells below it)
        if (idx < sizeX * (sizeY - 1)) {
            out.push_back(idx + sizeX); // add lower cell
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

        if (idx > sizeX * sizeY - 1) {
            return out;
        }

        if (idx % sizeX > 0 && idx >= sizeX) {
            out.push_back(idx - 1 - sizeX); // add upper left cell
        }
        if (idx % sizeX > 0 && idx < sizeX * (sizeY - 1)) {
            out.push_back(idx - 1 + sizeX); // add lower left cell
        }
        if (idx % sizeX < sizeX - 1 && idx >= sizeX) {
            out.push_back(idx + 1 - sizeX); // add upper right cell
        }
        if (idx % sizeX < sizeX - 1 && idx < sizeX * (sizeY - 1)) {
            out.push_back(idx + 1 + sizeX); // add lower right cell
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
#endif
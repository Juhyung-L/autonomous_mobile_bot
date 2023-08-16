#include <queue>
#include <numeric>

#include "visualization_msgs/msg/marker.hpp"

#include "frontier_explorer/frontier_search.hpp"

namespace frontier_explorer
{

static double distanceFormula(double x1, double y1, double x2, double y2)
{
    return sqrt(pow(x2-x1,2) + pow(y2-y1,2));
}

FrontierSearch::FrontierSearch(rclcpp::Node& node) : node(node)
{
    // publisher to publish frontiers (both points and centroids) for debug
    markerPub = node.create_publisher<visualization_msgs::msg::MarkerArray>("froniter_markers", 10);
    // importance of the distance to the closest frontier point when calculating the Frontiers' cost
    node.declare_parameter<double>("distance_weight", 1.0);
    // importance of the number of frontier points when calculating the Frontiers' cost
    node.declare_parameter<double>("size_weight", 1.0);

    node.get_parameter("distance_weight", distanceWeight);
    node.get_parameter("size_weight", sizeWeight);
}

std::vector<Frontiers> FrontierSearch::searchFrontiers(const geometry_msgs::msg::Pose& startPose)
{
    // turn the pose into int
    // input it into getIndex to get the index 
    std::vector<Frontiers> frontiersList;

    unsigned int mx, my;
    if (!worldToMap(startPose.position.x, startPose.position.y, mx, my))
    {
        RCLCPP_WARN(node.get_logger(), "Robot is outside of map");
        return frontiersList;
    }
    unsigned int startIndex = cellsToIndex(mx, my);

    std::vector<bool> visited(sizeX * sizeY, false);
    std::queue<unsigned int> bfs;

    // proceed if the cell under the robot has known occupancy
    // and the occupancy is less than MAX_NON_OBSTACLE
    if (mapData[startIndex] <= MAX_NON_OBSTACLE)
    {
        bfs.push(startIndex);
        visited[startIndex] = true;
    }
    else
    {
        // TODO: make the robot move forward until a cell 
        // with less than or equal to MAX_NON_OBSTACLE cell value
        return frontiersList; // return empty frontier list
    }

    // use bfs to traverse through cells less than MAX_NON_OBSTACLE to find a frontier point
    // if a frontier point is found, use bfs(again) to find all connected frontiers using buildNewFrontiers
    // buildNewFrontiers returns a Frontiers struct, which contains a vector of frontier points
    // Essentially, the Frontiers object represents an island of frontier points
    // the robot is made to traverse towards the centroid of the island
    while (!bfs.empty())
    {
        unsigned int idx = bfs.front();
        bfs.pop();

        for (unsigned int& nbr : nhood4(idx))
        {
            if (!visited[nbr] && mapData[nbr] <= MAX_NON_OBSTACLE)
            {
                if (isNewFrontier(nbr))
                {
                    frontiersList.push_back(buildNewFrontiers(nbr, visited, startPose));
                }
                bfs.push(nbr);
            }
            visited[nbr] = true;
        }
    }
    
    // calculate the cost for each
    for (auto& f : frontiersList)
    {
        f.cost = frontierCost(f);
    }

    return frontiersList;
}

double FrontierSearch::frontierCost(const Frontiers& frontier)
{
    return (distanceWeight * frontier.min_distance * resolution) -
           (sizeWeight * frontier.points.size() * resolution);
}

bool FrontierSearch::isNewFrontier(unsigned int idx)
{
    // if any of its neighboring cells is unknown, it is a frontier cell
    for (unsigned int& nbr : nhood8(idx))
    {
        if (mapData[nbr] == NO_INFORMATION)
        {
            return true;
        }
    }
    return false;
}

Frontiers FrontierSearch::buildNewFrontiers(unsigned int startIdx, std::vector<bool>& visited,
                                            const geometry_msgs::msg::Pose& robotPose)
{
    // get all connected frontiers and return the Frontiers struct
    // by traversing through cells less than MAX_NON_OBSTACLE
    Frontiers frontiers;
    frontiers.min_distance = std::numeric_limits<double>::infinity();

    std::queue<unsigned int> bfs;
    bfs.push(startIdx);
    visited[startIdx] = true;

    while (!bfs.empty())
    {
        unsigned int idx = bfs.front();
        bfs.pop();

        // change map index into map coordinates and store it in frontiers
        unsigned int mx, my;
        double wx, wy;
        indexToCells(idx, mx, my);
        mapToWorld(mx, my, wx, wy);
        geometry_msgs::msg::Point p;
        p.x = wx;
        p.y = wy;
        frontiers.points.push_back(p);

        // calculating centroid of frontiers
        frontiers.centriod.x += wx;
        frontiers.centriod.y += wy;

        // getting minimum distance from robot pose
        double distance = distanceFormula(robotPose.position.x, robotPose.position.y, wx, wy);
        if (frontiers.min_distance > distance)
        {
            frontiers.min_distance = distance;
        }

        for (unsigned int& nbr : nhood8(idx))
        {
            if (!visited[nbr] && isNewFrontier(nbr) && mapData[nbr] <= MAX_NON_OBSTACLE)
            {
                bfs.push(nbr);
            }
            visited[nbr] = true;
        }
    }
    frontiers.centriod.x /= frontiers.points.size();
    frontiers.centriod.y /= frontiers.points.size();

    return frontiers;
}

bool FrontierSearch::goalOnBlacklist(const geometry_msgs::msg::Point& goal)
    {
        constexpr static size_t tolerace = 5;

        // check if a goal is on the blacklist for goals that we're pursuing
        for (auto& f : frontiersBlacklist) {
            double dx = fabs(goal.x - f.x);
            double dy = fabs(goal.y - f.y);
            // go thru every point in frontier_blacklist
            // see if goal is close enough to a blacklisted frontier point
            if (dx < tolerace * resolution && 
                dy < tolerace * resolution)
            {
                return true;
            }
            
        }
        return false;
    }

void FrontierSearch::updateMap(const nav2_msgs::msg::Costmap::SharedPtr& costmap)
{
    mapData = costmap->data.data(); // get a pointer to the first element in the vector
    sizeX = costmap->metadata.size_x;
    sizeY = costmap->metadata.size_y;
    resolution = costmap->metadata.resolution;
    frameId = costmap->header.frame_id;
    mapOrigin = costmap->metadata.origin;
}

void FrontierSearch::addToBlacklist(const geometry_msgs::msg::Point& centroid)
{
    frontiersBlacklist.push_back(centroid);
    RCLCPP_INFO(node.get_logger(), "Blacklist size: %ld", frontiersBlacklist.size());
}

// input will always be vector<Frontiers> that is sorted based on Frontiers.cost
void FrontierSearch::visualizeFrontiers(const std::vector<Frontiers>& frontiersList)
{
    auto lowestCost = 
    std::find_if_not(frontiersList.begin(), frontiersList.end(),
                     [this](const Frontiers& f) {return goalOnBlacklist(f.centriod);
                     });
    
    auto highestCost = 
    std::find_if_not(frontiersList.end(), frontiersList.begin(),
                     [this](const Frontiers& f) {return goalOnBlacklist(f.centriod);
                     });

    if (lowestCost == frontiersList.end())
    {
        return; // either frontiersList is empty or all of its frontiers are in the blacklist
    }

    double range = highestCost->cost - lowestCost->cost;
    if (range == 0.0)
    {
        range = 0.0001; // prevent division by 0
    }

    visualization_msgs::msg::MarkerArray markersMsg;
    std::vector<visualization_msgs::msg::Marker>& markers = markersMsg.markers;
    visualization_msgs::msg::Marker m;

    // m is the marker message to send
    m.header.frame_id = frameId;
    m.header.stamp = node.now();
    m.ns = "frontiers";
    m.color.a = 1.0;

    // marker lives forever
    m.lifetime = rclcpp::Duration::from_seconds(0);
    m.frame_locked = false;
    // turn this on if the /map frame is not a fixed frame
    // if false, the markers will not move even if the map moves
    // ex. if the map rotates or translates, the markers will stay in place
    // if true, the markers move following the movement of its frame, which is /map
    // since the /map frame is fixed, the markers will never move
    // so set it false to save resources

    m.action = visualization_msgs::msg::Marker::ADD;
    size_t id = 0; // intialize id to 0
    RCLCPP_INFO(node.get_logger(), "Visualizing %ld frontiers", frontiersList.size());
    for (auto& frontiers : frontiersList) 
    {
        // set the color of the marker
        if (goalOnBlacklist(frontiers.centriod)) 
        {
            m.color.r = 1.0; // if frontiers is in blacklist, make it red
            m.color.g = 0.0;
            m.color.b = 0.0;
        }
        else
        {
            // if not in blacklist,
            // close frontiers = greener
            // faraway frontiers = bluer
            m.color.r = 0.0;
            m.color.g = 1.0 - ((frontiers.cost - lowestCost->cost) / range);
            m.color.b = (frontiers.cost - lowestCost->cost) / range;
        }

        // make point marker for individual frontier points
        m.type = visualization_msgs::msg::Marker::POINTS;
        m.id = int(id); // give new id to every marker
        m.pose.position.x = 0.0; // reset the origin of the points
        m.pose.position.y = 0.0;
        m.points = frontiers.points;
        m.scale.x = 0.05;
        m.scale.y = 0.05;
        m.scale.z = 0.05;

        markers.push_back(m);
        ++id;

        // make sphere marker for centroid
        m.type = visualization_msgs::msg::Marker::SPHERE;
        m.id = int(id); // give new id to every marker
        m.pose.position.x = frontiers.centriod.x;
        m.pose.position.y = frontiers.centriod.y;
        m.scale.x = 0.2;
        m.scale.y = 0.2;
        m.scale.z = 0.2;

        markers.push_back(m);
        ++id;
    }
    size_t currentMarkerCount = markers.size();

    // delete previous markers, which are now unused
    // so iterating from current id (id was incremented in the loop above)
    // to last_marker_count deletes all previous markers??
    m.action = visualization_msgs::msg::Marker::DELETE;
    for (; id < prevMarkerCount; ++id) {
        m.id = int(id);
        markers.push_back(m);
    }

    prevMarkerCount = currentMarkerCount; // store marker size
    markerPub->publish(markersMsg);
}

} // namespace frontier_search

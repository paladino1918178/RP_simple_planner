#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <queue>
#include <vector>
#include <cmath>
#include <unordered_map>
#include <functional>
#include <algorithm>

struct AStarNode
{
    int x, y;
    double g, h;
    AStarNode* parent;
    double f() const { return g + h; }
};

struct NodeHash
{
    size_t operator()(const std::pair<int,int>& p) const {
        return std::hash<int>()(p.first) ^ (std::hash<int>()(p.second) << 1);
    }
};

class SimplePlannerNode : public rclcpp::Node
{
public:
    SimplePlannerNode() : Node("simple_planner_node")
    {
        RCLCPP_INFO(this->get_logger(), "SimplePlannerNode pronto. Usa RViz per impostare start e goal.");

        publisher_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);
        subscription_map_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 10, std::bind(&SimplePlannerNode::map_callback, this, std::placeholders::_1)
        );
        subscription_start_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose", 10, std::bind(&SimplePlannerNode::start_callback, this, std::placeholders::_1)
        );
        subscription_goal_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10, std::bind(&SimplePlannerNode::goal_callback, this, std::placeholders::_1)
        );
    }

private:
    // Callback map
    void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        map_ = *msg;
        map_received_ = true;
        try_publish_path();
    }

    // Callback start dinamico
    void start_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        start_ = { msg->pose.pose.position.x, msg->pose.pose.position.y };
        start_set_ = true;
        RCLCPP_INFO(this->get_logger(), "Nuovo START: (%.2f, %.2f)", start_.first, start_.second);
        try_publish_path();
    }

    // Callback goal dinamico
    void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        goal_ = { msg->pose.position.x, msg->pose.position.y };
        goal_set_ = true;
        RCLCPP_INFO(this->get_logger(), "Nuovo GOAL: (%.2f, %.2f)", goal_.first, goal_.second);
        try_publish_path();
    }

    // Controlla se start, goal e map sono pronti
    void try_publish_path()
    {
        if(!map_received_ || !start_set_ || !goal_set_)
            return;

        auto points = astar();
        if(points.empty()) {
            RCLCPP_WARN(this->get_logger(), "Nessun percorso trovato!");
            return;
        }

        nav_msgs::msg::Path path_msg;
        path_msg.header.frame_id = "map";
        path_msg.header.stamp = this->now();

        RCLCPP_INFO(this->get_logger(), "Percorso calcolato:");

        for(auto &p : points)
        {
            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = "map";
            pose.pose.position.x = p.first;
            pose.pose.position.y = p.second;
            pose.pose.position.z = 0.0;
            pose.pose.orientation.w = 1.0;
            path_msg.poses.push_back(pose);

            RCLCPP_INFO(this->get_logger(), "  (%.2f, %.2f)", p.first, p.second);
        }

        publisher_->publish(path_msg);
        RCLCPP_INFO(this->get_logger(), "Percorso A* pubblicato con %zu punti.", path_msg.poses.size());
    }

    // Funzione A*
    bool isFree(int x, int y)
    {
        if(x < 0 || y < 0 || x >= (int)map_.info.width || y >= (int)map_.info.height)
            return false;
        int idx = y * map_.info.width + x;
        return map_.data[idx] == 0;
    }

    std::vector<std::pair<double,double>> astar()
    {
        std::vector<std::pair<double,double>> path;
        if(!map_received_) return path;

        int sx = (int)((start_.first - map_.info.origin.position.x) / map_.info.resolution);
        int sy = (int)((start_.second - map_.info.origin.position.y) / map_.info.resolution);
        int gx = (int)((goal_.first - map_.info.origin.position.x) / map_.info.resolution);
        int gy = (int)((goal_.second - map_.info.origin.position.y) / map_.info.resolution);

        std::priority_queue<
            std::pair<double, AStarNode*>,
            std::vector<std::pair<double, AStarNode*>>,
            std::greater<>
        > open_set;

        std::unordered_map<std::pair<int,int>, AStarNode*, NodeHash> all_nodes;

        AStarNode* start_node = new AStarNode{sx, sy, 0.0, std::hypot(gx-sx, gy-sy), nullptr};
        open_set.push({start_node->f(), start_node});
        all_nodes[{sx,sy}] = start_node;

        std::vector<std::pair<int,int>> directions = {
            {1,0},{-1,0},{0,1},{0,-1},{1,1},{1,-1},{-1,1},{-1,-1}
        };

        AStarNode* goal_node = nullptr;

        while(!open_set.empty())
        {
            AStarNode* current = open_set.top().second;
            open_set.pop();

            if(current->x == gx && current->y == gy)
            {
                goal_node = current;
                break;
            }

            for(auto &d : directions)
            {
                int nx = current->x + d.first;
                int ny = current->y + d.second;
                if(!isFree(nx,ny)) continue;

                double ng = current->g + std::hypot(d.first,d.second);
                auto it = all_nodes.find({nx,ny});
                if(it == all_nodes.end() || ng < it->second->g)
                {
                    AStarNode* neighbor;
                    if(it == all_nodes.end())
                    {
                        neighbor = new AStarNode{nx, ny, ng, std::hypot(gx-nx, gy-ny), current};
                        all_nodes[{nx,ny}] = neighbor;
                    }
                    else
                    {
                        neighbor = it->second;
                        neighbor->g = ng;
                        neighbor->parent = current;
                    }
                    open_set.push({neighbor->f(), neighbor});
                }
            }
        }

        if(goal_node)
        {
            AStarNode* n = goal_node;
            while(n)
            {
                double wx = n->x * map_.info.resolution + map_.info.origin.position.x + map_.info.resolution/2.0;
                double wy = n->y * map_.info.resolution + map_.info.origin.position.y + map_.info.resolution/2.0;
                path.push_back({wx, wy});
                n = n->parent;
            }
            std::reverse(path.begin(), path.end());
        }

        for(auto &p : all_nodes) delete p.second;

        return path;
    }

    // Variabili
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscription_map_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subscription_start_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_goal_;

    nav_msgs::msg::OccupancyGrid map_;
    bool map_received_ = false;
    bool start_set_ = false;
    bool goal_set_ = false;

    std::pair<double,double> start_;
    std::pair<double,double> goal_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimplePlannerNode>());
    rclcpp::shutdown();
    return 0;
}


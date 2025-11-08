#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include <queue>
#include <vector>
#include <cmath>
#include <unordered_map>
#include <unordered_set>
#include <functional>
#include <algorithm>

// nodo A*
struct AStarNode {
  int x, y;
  double g, h;
  AStarNode* parent;
  double f() const { return g + h; }
};

// hash pair
struct PairHash { 
  size_t operator()(const std::pair<int,int>& p) const {
    return std::hash<int>()(p.first) ^ (std::hash<int>()(p.second) << 1);
  }
};

// nodo planner
class SimplePlannerNode : public rclcpp::Node 
{
public:
  SimplePlannerNode() : Node("simple_planner_node")
  {
    RCLCPP_INFO(this->get_logger(), "SimplePlannerNode pronto. Usa RViz per impostare start e goal.");

    // Parametri
    allow_diagonal_   = this->declare_parameter("allow_diagonal", true);  // diagonali on/off
    corner_cutting_   = this->declare_parameter("corner_cutting", false); // vieta spigoli
    wall_penalty_w_   = this->declare_parameter("wall_penalty_w", 0.75);  // peso muri
    replan_on_map_    = this->declare_parameter("replan_on_map", true);   // ricalcolo mappa

    publisher_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);
    
    subscription_map_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", 10, std::bind(&SimplePlannerNode::map_callback, this, std::placeholders::_1));
    
    subscription_start_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose", 10, std::bind(&SimplePlannerNode::start_callback, this, std::placeholders::_1));
    
    subscription_goal_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", 10, std::bind(&SimplePlannerNode::goal_callback, this, std::placeholders::_1));
  }

private:
  // helpers mappa 
  // dentro mappa?
  inline bool inBounds(int x, int y) const { 
    return x >= 0 && y >= 0 &&
           x < static_cast<int>(map_.info.width) &&
           y < static_cast<int>(map_.info.height);
  }

  // indice lineare
  inline int index(int x, int y) const {
    return y * static_cast<int>(map_.info.width) + x;
  }

  // cella occupata
  inline bool isOccupied(int x, int y) const {
    if (!inBounds(x,y)) return true;
    return map_.data[index(x,y)] >= 100;
  }
  
  // cella libera
  inline bool isFree(int x, int y) const {
    if (!inBounds(x,y)) return false;
    return map_.data[index(x,y)] == 0;
  }

  // check attraversamento
  inline bool traversable(int x, int y, int nx, int ny) const { 
    if (!isFree(nx,ny)) return false;
    int dx = nx - x, dy = ny - y;
    if (!corner_cutting_ && dx != 0 && dy != 0) {
      if (!isFree(x, ny)) return false;
      if (!isFree(nx, y)) return false;
    }
    return true;
  }

  // penalità muri
  inline double wallPenalty(int cx, int cy) const {
    static const int nn[8][2] = {{1,0},{-1,0},{0,1},{0,-1},{1,1},{1,-1},{-1,1},{-1,-1}};
    int occ = 0;
    for (auto &d : nn) {
      int nx = cx + d[0], ny = cy + d[1];
      if (inBounds(nx,ny) && map_.data[index(nx,ny)] >= 100) ++occ;
    }
    return wall_penalty_w_ * (static_cast<double>(occ) / 8.0);
  }

  // Conversioni
  // map->grid
  inline std::pair<int,int> toGrid(double wx, double wy) const { 
    int gx = static_cast<int>((wx - map_.info.origin.position.x) / map_.info.resolution);
    int gy = static_cast<int>((wy - map_.info.origin.position.y) / map_.info.resolution);
    return {gx, gy};
  }

  // grid->world
  inline std::pair<double,double> toWorld(int gx, int gy) const { 
    double wx = gx * map_.info.resolution + map_.info.origin.position.x + map_.info.resolution/2.0;
    double wy = gy * map_.info.resolution + map_.info.origin.position.y + map_.info.resolution/2.0;
    return {wx, wy};
  }

  // callbacks
  // callback mappa
  void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) 
  {
    map_ = *msg;
    map_received_ = true;
    if (replan_on_map_) {
      if (!path_still_valid_on_current_map()) try_publish_path();
    }
  }

  // callback start
  void start_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    start_ = { msg->pose.pose.position.x, msg->pose.pose.position.y };
    start_set_ = true;
    RCLCPP_INFO(this->get_logger(), "Nuovo START: (%.2f, %.2f)", start_.first, start_.second);
    try_publish_path();
  }

  // callback goal
  void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    goal_ = { msg->pose.position.x, msg->pose.position.y };
    goal_set_ = true;
    RCLCPP_INFO(this->get_logger(), "Nuovo GOAL: (%.2f, %.2f)", goal_.first, goal_.second);
    try_publish_path();
  }

  // pianifica e pubblica
  void try_publish_path()
  {
    if(!map_received_ || !start_set_ || !goal_set_) return;

    auto sg = toGrid(start_.first, start_.second);
    auto gg = toGrid(goal_.first, goal_.second);

    if (!inBounds(sg.first, sg.second) || !inBounds(gg.first, gg.second)) {
      RCLCPP_WARN(this->get_logger(), "Start o goal fuori mappa.");
      return;
    }
    if (isOccupied(sg.first, sg.second) || isOccupied(gg.first, gg.second)) {
      RCLCPP_WARN(this->get_logger(), "Start o goal su cella occupata.");
      return;
    }

    if (!last_path_grid_.empty() && path_still_valid_on_current_map()) return;

    auto points = astar(sg.first, sg.second, gg.first, gg.second);
    if(points.empty()) {
      RCLCPP_WARN(this->get_logger(), "Nessun percorso trovato!");
      last_path_grid_.clear();
      publish_empty_path();
      return;
    }

    nav_msgs::msg::Path path_msg;
    path_msg.header.frame_id = "map";
    path_msg.header.stamp = this->now();

    last_path_grid_.clear();
    for(auto &p : points) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = "map";
      pose.pose.position.x = p.first;
      pose.pose.position.y = p.second;
      pose.pose.position.z = 0.0;
      pose.pose.orientation.w = 1.0;
      path_msg.poses.push_back(pose);

      auto g = toGrid(p.first, p.second);
      last_path_grid_.push_back(g);
    }

    publisher_->publish(path_msg);
    RCLCPP_INFO(this->get_logger(), "Path con A* trovato con %zu punti.", path_msg.poses.size());
  }

  // A* 
  std::vector<std::pair<double,double>> astar(int sx, int sy, int gx, int gy)
  {
    std::vector<std::pair<double,double>> path;
    static const std::vector<std::pair<int,int>> dir4 {{1,0},{-1,0},{0,1},{0,-1}};
    static const std::vector<std::pair<int,int>> dir8 {{1,0},{-1,0},{0,1},{0,-1},{1,1},{1,-1},{-1,1},{-1,-1}};
    auto &dirs = allow_diagonal_ ? dir8 : dir4;

    auto hfun = [&](int x, int y){ return std::hypot(gx - x, gy - y); };
    
    // Tie-breaker leggero per favorire linee più dritte: f' = f + eps*h
    const double eps = 1e-3;

    using QItem = std::pair<double, AStarNode*>;
    std::priority_queue<QItem, std::vector<QItem>, std::greater<QItem>> open_set;
    std::unordered_map<std::pair<int,int>, AStarNode*, PairHash> all_nodes;
    std::unordered_set<std::pair<int,int>, PairHash> closed;

    AStarNode* start_node = new AStarNode{sx, sy, 0.0, hfun(sx,sy), nullptr};
    open_set.push({start_node->f() + eps*start_node->h, start_node});
    all_nodes[{sx,sy}] = start_node;

    AStarNode* goal_node = nullptr;

    while(!open_set.empty()) {
      AStarNode* current = open_set.top().second;
      open_set.pop();

      auto key = std::make_pair(current->x, current->y);
      if (closed.find(key) != closed.end()) continue;
      closed.insert(key);

      if(current->x == gx && current->y == gy) {
        goal_node = current;
        break;
      }

      for (auto &d : dirs) {
        int nx = current->x + d.first;
        int ny = current->y + d.second;
        if (!inBounds(nx,ny)) continue;
        if (!traversable(current->x, current->y, nx, ny)) continue;
        
        // costo di passo: distanza + penalità di vicinanza ai muri
        double step = std::hypot(d.first, d.second);
        double ng = current->g + step + wallPenalty(nx, ny);

        auto nkey = std::make_pair(nx,ny);
        auto it = all_nodes.find(nkey);
        if (it == all_nodes.end()) {
          AStarNode* neighbor = new AStarNode{nx, ny, ng, hfun(nx,ny), current};
          all_nodes[nkey] = neighbor;
          open_set.push({neighbor->f() + eps*neighbor->h, neighbor});
        } else if (ng < it->second->g) {
          AStarNode* neighbor = it->second;
          neighbor->g = ng;
          neighbor->parent = current;
          open_set.push({neighbor->f() + eps*neighbor->h, neighbor}); // reinserisco con migliore f
        }
      }
    }

    if (goal_node) {
      AStarNode* n = goal_node;
      while(n) {
        auto w = toWorld(n->x, n->y);
        path.push_back(w);
        n = n->parent;
      }
      std::reverse(path.begin(), path.end());
    }

    for(auto &p : all_nodes) delete p.second;
    return path;
  }

  // variabili
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

  // parametri
  bool allow_diagonal_{true};
  bool corner_cutting_{false};
  double wall_penalty_w_{0.75};
  bool replan_on_map_{true};

  std::vector<std::pair<int,int>> last_path_grid_;
  
  // svuota path
  void publish_empty_path() {
    nav_msgs::msg::Path empty;
    empty.header.frame_id = "map";
    empty.header.stamp = this->now();
    publisher_->publish(empty);
  }

  // path valida?
  bool path_still_valid_on_current_map() const { 
    if (last_path_grid_.empty()) return false;
    for (auto &pg : last_path_grid_) {
      int x = pg.first, y = pg.second;
      if (!inBounds(x,y) || isOccupied(x,y)) return false;
    }
    return true;
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimplePlannerNode>());
  rclcpp::shutdown();
  return 0;
}

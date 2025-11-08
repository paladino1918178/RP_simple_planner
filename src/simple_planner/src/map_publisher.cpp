#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <vector>
#include <utility>
#include <algorithm>
#include <random>
#include <chrono>

using namespace std::chrono_literals;

struct Door {
  int x1, y1;
  int x2, y2;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("dynamic_map_publisher");

  auto pub = node->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", rclcpp::QoS(10));

  // Parametri mappa
  const int    W = 32;                 // larghezza celle
  const int    H = 32;                 // altezza celle
  const double RES = 0.25;             // metri/cella
  const double ORIG_X = -4.0, ORIG_Y = -4.0;
  const int WALL_THICK = 2;            // spessore muri interni
  const int ROOM_FREE  = 6;            // lato libero stanza (celle)
  const int PERIM = 1;                 // spessore muro perimetrale
  const int WALL_X1[3] = {7, 15, 23};  // colonne dei muri verticali (prima colonna del "doppio muro")
  const int WALL_X2[3] = {8, 16, 24};  // seconde colonne
  const int WALL_Y1[3] = {7, 15, 23};  // righe dei muri orizzontali
  const int WALL_Y2[3] = {8, 16, 24};

  // Parametri dinamicità delle porte
  const int   NUM_OPEN_DOORS = 18;      // quante "porte" restano aperte contemporaneamente
  const double TOGGLE_SEC     = 2.0;   // ogni quanti secondi rimescoliamo le porte aperte

  // Mappa statica con muri e stanze
  nav_msgs::msg::OccupancyGrid base_map;
  base_map.header.frame_id = "map";
  base_map.info.width  = W;
  base_map.info.height = H;
  base_map.info.resolution = RES;
  base_map.info.origin.position.x = ORIG_X;
  base_map.info.origin.position.y = ORIG_Y;
  base_map.info.origin.orientation.w = 1.0;
  base_map.data.assign(W * H, 0);

  auto idx = [W](int x, int y) { return y * W + x; };
  auto inb = [W, H](int x, int y) { return x >= 0 && x < W && y >= 0 && y < H; };
  auto set_occ = [&](int x, int y, int v){ if (inb(x,y)) base_map.data[idx(x,y)] = v; };

  // Perimetro (spessore 1 cella)
  for (int x = 0; x < W; ++x) { set_occ(x, 0, 100); set_occ(x, H-1, 100); }
  for (int y = 0; y < H; ++y) { set_occ(0, y, 100); set_occ(W-1, y, 100); }

  // Muri interni verticali (spessore 2)
  for (int k = 0; k < 3; ++k) {
    for (int y = 1; y < H-1; ++y) {
      set_occ(WALL_X1[k], y, 100);
      set_occ(WALL_X2[k], y, 100);
    }
  }
  // Muri interni orizzontali (spessore 2)
  for (int k = 0; k < 3; ++k) {
    for (int x = 1; x < W-1; ++x) {
      set_occ(x, WALL_Y1[k], 100);
      set_occ(x, WALL_Y2[k], 100);
    }
  }

  // Catalogo delle "porte" possibili 
  std::vector<int> room_band_centers = { PERIM + ROOM_FREE/2,                 // 1..6 -> centro ~3/4
                                         PERIM + ROOM_FREE + WALL_THICK + ROOM_FREE/2,      // 9..14 -> 11/12
                                         PERIM + 2*(ROOM_FREE + WALL_THICK) + ROOM_FREE/2,  // 17..22 -> 19/20
                                         PERIM + 3*(ROOM_FREE + WALL_THICK) + ROOM_FREE/2 };// 25..30 -> 27/28

  std::vector<Door> all_doors;

  // Porte sui muri verticali (collegano stanze sinistra-destra)
  for (int k = 0; k < 3; ++k) {
    for (int cy : room_band_centers) {
      if ( (cy == WALL_Y1[0]) || (cy == WALL_Y2[0]) ||
           (cy == WALL_Y1[1]) || (cy == WALL_Y2[1]) ||
           (cy == WALL_Y1[2]) || (cy == WALL_Y2[2]) ) continue;
      all_doors.push_back(Door{WALL_X1[k], cy, WALL_X2[k], cy});
    }
  }

  // Porte sui muri orizzontali (collegano stanze sopra-sotto)
  for (int k = 0; k < 3; ++k) {
    for (int cx : room_band_centers) {
      if ( (cx == WALL_X1[0]) || (cx == WALL_X2[0]) ||
           (cx == WALL_X1[1]) || (cx == WALL_X2[1]) ||
           (cx == WALL_X1[2]) || (cx == WALL_X2[2]) ) continue;
      all_doors.push_back(Door{cx, WALL_Y1[k], cx, WALL_Y2[k]});
    }
  }
  // all_doors contiene ~24 porte candidate (12 verticali + 12 orizzontali)

  // Stato delle porte aperte (dinamiche)
  std::mt19937 rng(static_cast<unsigned>(std::chrono::steady_clock::now().time_since_epoch().count()));
  auto pick_open_set = [&](int k, std::vector<int> &out_indices) {
    out_indices.clear();
    std::vector<int> pool(all_doors.size());
    for (size_t i = 0; i < all_doors.size(); ++i) pool[i] = static_cast<int>(i);
    std::shuffle(pool.begin(), pool.end(), rng);
    if (k > static_cast<int>(pool.size())) k = static_cast<int>(pool.size());
    out_indices.insert(out_indices.end(), pool.begin(), pool.begin() + k);
  };

  std::vector<int> open_door_indices;
  pick_open_set(NUM_OPEN_DOORS, open_door_indices);
  rclcpp::Time last_toggle = node->get_clock()->now();

  // Loop di pubblicazione 
  rclcpp::Rate rate(5.0); // 5 Hz
  while (rclcpp::ok()) {
    // ricalcolo set di porte aperte ogni TOGGLE_SEC
    rclcpp::Time now = node->get_clock()->now();
    if ((now - last_toggle).seconds() >= TOGGLE_SEC) {
      pick_open_set(NUM_OPEN_DOORS, open_door_indices);
      last_toggle = now;
      RCLCPP_INFO(node->get_logger(), "Aggiornate le porte aperte (%zu su %zu disponibili).",
                  open_door_indices.size(), all_doors.size());
    }

    nav_msgs::msg::OccupancyGrid map = base_map;
    for (int idxDoor : open_door_indices) {
      const Door &d = all_doors[idxDoor];
      if (inb(d.x1, d.y1)) map.data[idx(d.x1, d.y1)] = 0;
      if (inb(d.x2, d.y2)) map.data[idx(d.x2, d.y2)] = 0;
    }

    map.header.stamp = now;
    pub->publish(map);

    rclcpp::spin_some(node);
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}

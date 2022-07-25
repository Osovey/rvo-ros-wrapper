#ifndef ROSMAP_HPP
#define ROSMAP_HPP

#include <vector>
#include <set>

#include <boost/scoped_array.hpp>
#include <boost/scoped_ptr.hpp>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#include <nav_msgs/OccupancyGrid.h>
#include <player_map/map.h>

namespace rvo {
typedef std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f> > PointVector;

double pathLength(const rvo::PointVector &path);

class OccupancyMap {
public:
  OccupancyMap(map_t *map);
  ~OccupancyMap();

  static OccupancyMap* FromMapServer(const char *srv_name);

  void setMap(const nav_msgs::OccupancyGrid &grid);
  void updateCSpace(double max_occ_dist);

  double minX() { return MAP_WXGX(map_, 0); }
  double minY() { return MAP_WYGY(map_, 0); }
  double maxX() { return MAP_WXGX(map_, map_->size_x); }
  double maxY() { return MAP_WYGY(map_, map_->size_y); }

  const map_cell_t* getCell(double x, double y) {
    return map_get_cell(map_, x, y, 0);
  }

  bool lineOfSight(double x1, double y1, double x2, double y2,
                   double max_occ_dist = 0.0) const;
  PointVector astar(double x1, double y1, double x2, double y2,
                    double max_occ_dist = 0.0);

  // TODO: Unify these two APIs

  // Get a list of endpoints
  const PointVector& prepareShortestPaths(double x, double y,
                                          double max_occ_dist,
                                          double min_dist, double max_dist);
  // Get the path whose endpoint is ind from last call to prepareShortestPaths()
  PointVector buildShortestPath(int ind);

  // Calculate single source shortest paths to all endpoints
  // Returns a vector, element at index i is true if path from dests[i] exists
  // Use buildShortestPath to construct path; index matches point in dests
  void prepareAllShortestPaths(double x, double y, double max_occ_dist);
  // Get shortest path
  PointVector shortestPath(double x, double y);

private:
  struct Node {
    Node() {}
    Node(const std::pair<int, int> &c, float d, float h) :
      coord(c), true_dist(d), heuristic(h) { }
    std::pair<int, int> coord;
    float true_dist;
    float heuristic;
  };

  struct NodeCompare {
    bool operator()(const Node &lnode, const Node &rnode) {
      return make_pair(lnode.heuristic, lnode.coord) <
        make_pair(rnode.heuristic, rnode.coord);
    }
  };

  void initializeSearch(double startx, double starty);
  bool nextNode(double max_occ_dist, Node *curr_node);
  void addNeighbors(const Node &node, double max_occ_dist);
  void buildPath(int i, int j, PointVector *path);

  map_t *map_;
  int ncells_;
  int starti_, startj_;
  int stopi_, stopj_;
  boost::scoped_array<float> costs_;
  boost::scoped_array<int> prev_i_;
  boost::scoped_array<int> prev_j_;
  // Priority queue mapping cost to index
  boost::scoped_ptr<std::set<Node, NodeCompare> > Q_;
  PointVector endpoints_;
};

}
#endif

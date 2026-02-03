#include "global_planner/tta_planner.hpp"

#include <algorithm>
#include <iostream>

bool TTAPlanner::computeCenterline(
  std::vector<BoundaryPoint> & left_boundary,
  std::vector<BoundaryPoint> & right_boundary,
  std::vector<BoundaryPoint> & centerline_out
)
{
  centerline_out.clear();

  size_t n = std::min(left_boundary.size(), right_boundary.size());
  for (size_t i = 0; i < n; ++i) {
    BoundaryPoint mid_point{
      (left_boundary[i].x + right_boundary[i].x) / 2.0,
      (left_boundary[i].y + right_boundary[i].y) / 2.0
    };
    centerline_out.push_back(mid_point);

    if (i < 5) {
      std::cout << "i=" << i
                << "\nleft=(" << left_boundary[i].x << "," << left_boundary[i].y << ")"
                << "\nright=(" << right_boundary[i].x << "," << right_boundary[i].y << ")"
                << "\nmid=(" << mid_point.x << "," << mid_point.y << ")"
                << std::endl;
    }
  }

  return !centerline_out.empty();
}

/*   
    Need to implement the following steps:
    
    preprocessBounderies();
    left_chain = orderBoundary(left_boundary);
    right_chain = orderBoundary(right_boundary);
    pairs = pairBoundaryPoints(left_chain, right_chain);
    centerline = computeMidpoints(pairs);
    ordered = oderLoop(centerline);
    centerline_out = smoot(ordered);
*/


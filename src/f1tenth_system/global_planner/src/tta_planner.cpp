#include "global_planner/tta_planner.hpp"

#include <algorithm>
#include <iostream>

bool TTAPlanner::computeCenterline(
  const std::vector<BoundaryPoint>& left_boundary,
  const std::vector<BoundaryPoint>& right_boundary,
  std::vector<BoundaryPoint>& centerline_out
)

{
  centerline_out.clear();

  size_t n = std::min(left_boundary.size(), right_boundary.size());
  for (size_t i = 0; i < n; ++i) 
  {
    BoundaryPoint mid_point
    {
      (left_boundary[i].x + right_boundary[i].x) / 2.0,
      (left_boundary[i].y + right_boundary[i].y) / 2.0
    };
    centerline_out.push_back(mid_point);

    if (i < 5) 
    {
      std::cout << "i=" << i
                << "\nleft=(" << left_boundary[i].x << "," << left_boundary[i].y << ")"
                << "\nright=(" << right_boundary[i].x << "," << right_boundary[i].y << ")"
                << "\nmid=(" << mid_point.x << "," << mid_point.y << ")"
                << std::endl;
    }
  }

  return !centerline_out.empty();
}

static double dist2(const BoundaryPoint& p1, const BoundaryPoint& p2) 
{
  return (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y);
}


std::vector<BoundaryPoint> TTAPlanner::preprocessBounderies(const std::vector<BoundaryPoint>& boundary) 
{
  if(boundary.size() < 2) 
  {
    std::cerr << "Boundary has too few points to preprocess." << std::endl;
    return boundary;
  }
  std ::vector<BoundaryPoint> out;
  out.reserve(boundary.size());
  for (const auto& p : boundary) 
  {
    bool is_duplicate = false;
    for (const auto& q: out)
    {
      if (p.x == q.x && p.y == q.y)
      {
        duplicate = true;
        break;
      }
      if(!duplicate) 
      {
        out.push_back(p);
      }
    }
  }
  return out;
}

std::vector<BoundaryPoint> TTAPlanner::orderBoundary(const std::vector<BoundaryPoint>& boundary) 
{
  if (boudary.size() < 2) 
  {
    std::cerr << "Boundary has too few points to order." << std::endl;
    return boundary;
  }
  std::vector<BoundaryPoint> ordered;
  ordered.reserve(boundary.size());

  std::vector<bool> used(boundary.size(), false);

  size_t current = 0;
  used[current] = true;
  ordered.push_back(boundary[current]);

  for(size_t i = 1; i < boundary.size(); ++i) 
  {
    size_t best = static_cast<size_t>(-1);
    double best_dist = std::numeric_limits<double>::max();
    for(size_t j = 0; j < boundary.size(); ++j)
    {
      if (used[j]) 
      {
        continue;
      }
      double d2 = dist2(boundary[current], boundary[j]);
      if (d2 < best_dist) 
      {
        best_d2 = d2;
        best = j;
      }
    }

    if(best == static_cast<size_t>(-1)) 
    {
      std::cerr << "Failed to find next point in orderBoundary." << std::endl;
      break;
    }
    used[best] = true;
    ordered.push_back(boundary[best]);
    current = best;
  }
  return ordered;
}
 

/*
pairBoundaryPoints(const std::vector<BoundaryPoint>& left_chain, const std::vector<BoundaryPoint>& right_chain) 
{
  size_t n = std::min(left_chain.size(), right_chain.size());
  std::vector<std::pair<BoundaryPoint, BoundaryPoint>> pairs;
  for (size_t i = 0; i < n; ++i) 
  {
    pairs.emplace_back(left_chain[i], right_chain[i]);
  }
  return pairs;
  }

  computeMidpoints(const std::vector<std::pair<BoundaryPoint, BoundaryPoint>>& pairs) 
  {
  std::vector<BoundaryPoint> midpoints;
  for (const auto& pair : pairs) {
    BoundaryPoint midpoint{
      (pair.first.x + pair.second.x) / 2.0,
      (pair.first.y + pair.second.y) / 2.0
    };
    midpoints.push_back(midpoint);
  }
  return midpoints;
}
*/

std::vector<BoundaryPoint> TTAPlanner::pair<BoundaryPoint, BoundaryPoint> TTAPlanner::pairBoundaryPoints(
  const std::vector<BoundaryPoint>& left_chain,
  const std::vector<BoundaryPoint>& right_chain
) 
{
  std::vector<std::pair<BoundaryPoint, BoundaryPoint>> pairs;
  i (left_chain.empty() || right_chain.empty()) 
  {
    std::cerr << "One of the chains is empty in pairBoundaryPoints." << std::endl;
    return pairs;
  }
  pairs.reserve(left_chain.size());
  const double max_pair_dist2 = 25.0; //max distance squared to consider a valid pair
}




orderLoop(const std::vector<BoundaryPoint>& centerline) 
{
  size_t centerlineSize = centerline.size();
  if (centerlineSize < 2) {
    std::cerr << "Centerline has too few points to order." << std::endl;
    return centerline;
  }
  for(int i = 0; i < centerlineSize -1; ++i)
  {
    for(int j = i + 1; j < centerlineSize; ++j)
    {
      if (centerline[i].x > centerline[j].x || 
          (centerline[i].x == centerline[j].x && centerline[i].y > centerline[j].y)) {
        std::cerr << "Swapping points at index " << i << " and " << j << " for ordering." << std::endl;
        std::swap(centerline[i], centerline[j]);
      }
    }
  } 
  return centerline;
}

smooth(const std::vector<BoundaryPoint>& ordered) 
{
  std::vector<BoundaryPoint> ordered_centerline = ordered;
  size_t n = ordered_centerline.size();
  if (n < 3) 
  {
    std::cerr << "Not enough points to smooth." << std::endl;
    return ordered_centerline;
  }
  for (size_t i = 1; i < n - 1; ++i) 
  {
    ordered_centerline[i].x = (ordered_centerline[i - 1].x + ordered_centerline[i].x + ordered_centerline[i + 1].x) / 3.0;
    ordered_centerline[i].y = (ordered_centerline[i - 1].y + ordered_centerline[i].y + ordered_centerline[i + 1].y) / 3.0;
  }
  return ordered_centerline;
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

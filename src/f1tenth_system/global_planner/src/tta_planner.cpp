#include "global_planner/tta_planner.hpp"

#include <algorithm>
#include <iostream>

#include <limits>

bool TTAPlanner::computeCenterline(
  const std::vector<BoundaryPoint> & left_boundary,
  const std::vector<BoundaryPoint> & right_boundary,
  std::vector<BoundaryPoint> & centerline_out
)
{
  auto L = preprocessBoundaries(left_boundary);
  auto R = preprocessBoundaries(right_boundary);

  auto Lc = orderBoundary(L);
  auto Rc = orderBoundary(R);

  auto pairs = pairBoundaryPoints(Lc, Rc);
  auto mid = computeMidpoints(pairs);

  auto mid_ordered = orderBoundary(mid);
  auto mid_filtered = removeLargeJumps(mid_ordered);
  auto mid_smooth = smooth(mid_filtered);
  centerline_out = orderLoop(mid_smooth);

  return !centerline_out.empty();
}

static double dist2(const BoundaryPoint & p1, const BoundaryPoint & p2)
{
  return (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y);
}


std::vector<BoundaryPoint> TTAPlanner::preprocessBoundaries(
  const std::vector<BoundaryPoint> & boundary)
{
  if (boundary.size() < 2) {
    std::cerr << "Boundary has too few points to preprocess." << std::endl;
    return boundary;
  }

  std::vector<BoundaryPoint> out;
  out.reserve(boundary.size());

  for (const auto & p : boundary) {
    bool duplicate = false;

    for (const auto & q : out) {
      if (p.x == q.x && p.y == q.y) {
        duplicate = true;
        break;
      }
    }

    if (!duplicate) {
      out.push_back(p);
    }
  }

  return out;
}

std::vector<BoundaryPoint> TTAPlanner::orderBoundary(
  const std::vector<BoundaryPoint> & boundary)
{
  if (boundary.size() < 2) {return boundary;}

  std::vector<BoundaryPoint> ordered;
  ordered.reserve(boundary.size());

  std::vector<bool> used(boundary.size(), false);

  // Start at index 0 for baseline (later: start near vehicle pose)
  size_t current = 0;
  used[current] = true;
  ordered.push_back(boundary[current]);

  for (size_t k = 1; k < boundary.size(); ++k) {
    size_t best = static_cast<size_t>(-1);
    double best_d2 = std::numeric_limits<double>::infinity();

    for (size_t i = 0; i < boundary.size(); ++i) {
      if (used[i]) {continue;}
      double d2 = dist2(boundary[current], boundary[i]);
      if (d2 < best_d2) {
        best_d2 = d2;
        best = i;
      }
    }

    if (best == static_cast<size_t>(-1)) {
      break;                                     // should not happen
    }
    used[best] = true;
    ordered.push_back(boundary[best]);
    current = best;
  }

  return ordered;
}


std::vector<std::pair<BoundaryPoint, BoundaryPoint>> TTAPlanner::pairBoundaryPoints(
  const std::vector<BoundaryPoint> & left_chain,
  const std::vector<BoundaryPoint> & right_chain)
{
  std::vector<std::pair<BoundaryPoint, BoundaryPoint>> pairs;
  if (left_chain.empty() || right_chain.empty()) {
    std::cerr << "One of the chains is empty in pairBoundaryPoints." << std::endl;
    return pairs;
  }
  pairs.reserve(left_chain.size());
  const double max_pair_dist2 = 9.0;

  for (const auto & left_pt : left_chain) {
    size_t best = static_cast<size_t>(-1);
    double best_d2 = std::numeric_limits<double>::infinity();

    for (size_t j = 0; j < right_chain.size(); ++j) {
      double d2 = dist2(left_pt, right_chain[j]);
      if (d2 < best_d2) {
        best_d2 = d2;
        best = j;
      }
    }

    if (best != static_cast<size_t>(-1) && best_d2 <= max_pair_dist2) {
      pairs.emplace_back(left_pt, right_chain[best]);
    }
  }
  return pairs;

}

std::vector<BoundaryPoint> TTAPlanner::orderLoop(const std::vector<BoundaryPoint> & centerline)
{
  if (centerline.size() < 2) {return centerline;}

  std::vector<BoundaryPoint> out = centerline;
  const double close_dist2 = 1.0; // distance squared to consider points as neighbors
  if (dist2(out.front(), out.back()) < close_dist2) {
    out.back() = out.front(); // close the loop
  } else {
    out.push_back(out.front()); // close the loop by adding the first point at the end
  }
  return out;
}


std::vector<BoundaryPoint> TTAPlanner::smooth(const std::vector<BoundaryPoint> & ordered)
{
  if (ordered.size() < 3) {return ordered;}

  std::vector<BoundaryPoint> out = ordered;

  for (size_t i = 1; i + 1 < ordered.size(); ++i) {
    out[i].x = (ordered[i - 1].x + ordered[i].x + ordered[i + 1].x) / 3.0;
    out[i].y = (ordered[i - 1].y + ordered[i].y + ordered[i + 1].y) / 3.0;
  }
  return out;
}

std::vector<BoundaryPoint> TTAPlanner::computeMidpoints(
  const std::vector<std::pair<BoundaryPoint, BoundaryPoint>> & pairs)
{
  std::vector<BoundaryPoint> mids;
  mids.reserve(pairs.size());

  for (const auto & pr : pairs) {
    const auto & left = pr.first;
    const auto & right = pr.second;

    BoundaryPoint mid = left;  // copy left so any extra fields are carried over
    mid.x = 0.5 * (left.x + right.x);
    mid.y = 0.5 * (left.y + right.y);

    mids.push_back(mid);
  }

  return mids;
}

std::vector<BoundaryPoint> TTAPlanner::removeLargeJumps(
  const std::vector<BoundaryPoint> & ordered_points)
{
  if (ordered_points.size() < 3) {
    return ordered_points;
  }

  std::vector<BoundaryPoint> filtered;
  filtered.reserve(ordered_points.size());

  filtered.push_back(ordered_points[0]);

  const double max_jump_dist2 = 4.0;  
  // 4.0 = 2m squared
  // adjust later if needed

  for (size_t i = 1; i < ordered_points.size(); ++i) {
    double d2 = dist2(filtered.back(), ordered_points[i]);

    if (d2 <= max_jump_dist2) {
      filtered.push_back(ordered_points[i]);
    }
  }

  return filtered;
}

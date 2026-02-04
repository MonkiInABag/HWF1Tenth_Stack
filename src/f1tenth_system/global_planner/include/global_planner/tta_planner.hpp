#pragma once
#include <vector>

struct BoundaryPoint
{
  double x;
  double y;
};

class TTAPlanner
{
public:
  bool computeCenterline(
    const std::vector<BoundaryPoint> & left_boundary,
    const std::vector<BoundaryPoint> & right_boundary,
    std::vector<BoundaryPoint> & centerline
  );
};

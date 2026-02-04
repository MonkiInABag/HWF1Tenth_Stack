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


private:
  std::vector<BoundaryPoint> preprocessBoundaries(
    const std::vector<BoundaryPoint> & boundary
  );

  std::vector<BoundaryPoint> orderBoundary(
    const std::vector<BoundaryPoint> & boundary
  );

  std::vector<std::pair<BoundaryPoint, BoundaryPoint>> pairBoundaryPoints(
    const std::vector<BoundaryPoint> & left_chain,
    const std::vector<BoundaryPoint> & right_chain
  );

  std::vector<BoundaryPoint> computeMidpoints(
    const std::vector<std::pair<BoundaryPoint, BoundaryPoint>> & pairs
  );

  std::vector<BoundaryPoint> orderLoop(
    const std::vector<BoundaryPoint> & centerline
  );

  std::vector<BoundaryPoint> smooth(
    const std::vector<BoundaryPoint> & ordered_centerline
  );

};





/*
preprocessBounderies();
    left_chain = orderBoundary(left_boundary);
    right_chain = orderBoundary(right_boundary);
    pairs = pairBoundaryPoints(left_chain, right_chain);
    centerline = computeMidpoints(pairs);
    ordered = oderLoop(centerline);
    centerline_out = smoot(ordered);

*/
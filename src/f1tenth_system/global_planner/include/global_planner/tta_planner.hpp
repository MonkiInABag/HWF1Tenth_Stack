
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
        std::vector<BoundaryPoint>& left_boundary,
        std::vector<BoundaryPoint>& right_boundary,
        std::vector<BoundaryPoint>& centerline
    );
};
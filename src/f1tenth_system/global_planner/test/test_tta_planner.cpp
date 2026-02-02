#include <gtest/gtest.h>
#include "global_planner/tta_planner.hpp"

TEST(TTAPlanner, ComputeCenterline)
{
  TTAPlanner planner;

  std::vector<BoundaryPoint> left_boundary = {
    {0.0, 0.0},
    {0.0, 2.0},
    {0.0, 4.0}
  };

  std::vector<BoundaryPoint> right_boundary = {
    {2.0, 0.0},
    {2.0, 2.0},
    {2.0, 4.0}
  };

  std::vector<BoundaryPoint> centerline;

  ASSERT_TRUE(planner.computeCenterline(left_boundary, right_boundary, centerline));
  ASSERT_EQ(centerline.size(), 3);

  EXPECT_DOUBLE_EQ(centerline[0].x, 1.0);
  EXPECT_DOUBLE_EQ(centerline[0].y, 0.0);

  EXPECT_DOUBLE_EQ(centerline[1].x, 1.0);
  EXPECT_DOUBLE_EQ(centerline[1].y, 2.0);

  EXPECT_DOUBLE_EQ(centerline[2].x, 1.0);
  EXPECT_DOUBLE_EQ(centerline[2].y, 4.0);
}

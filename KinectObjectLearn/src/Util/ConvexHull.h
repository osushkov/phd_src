
#ifndef _ConvexHull_H_
#define _ConvexHull_H_

#include <vector>
#include "Vector2D.h"

std::vector<Vector2D> convexHull(std::vector<Vector2D> points);
std::vector<Vector2D> minimalBoundingRectangle(const std::vector<Vector2D> &convex_hull);
bool pointInConvexHull(Vector2D point, const std::vector<Vector2D> &hull);
bool pointInConvexHull2(Vector2D point, const std::vector<Vector2D> &hull, unsigned inc = 1, float err=0.1f);
float pointDistanceFromHull(Vector2D point, const std::vector<Vector2D> &hull, unsigned inc=1);
void expandConvexHull(std::vector<Vector2D> &hull, float expand_amount);

#endif



#ifndef _ConvexHull_H_
#define _ConvexHull_H_

#include <vector>
#include "Vector2D.h"

std::vector<Vector2D> convexHull(std::vector<Vector2D> points);
std::vector<Vector2D> minimalBoundingRectangle(const std::vector<Vector2D> &convex_hull);
bool pointInConvexHull(Vector2D point, const std::vector<Vector2D> &hull);
void expandConvexHull(std::vector<Vector2D> &hull, float expand_amount);

#endif


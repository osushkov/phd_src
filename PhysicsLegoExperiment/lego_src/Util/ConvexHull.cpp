

#include "ConvexHull.h"
#include <cassert>

#define _USE_MATH_DEFINES
#include <math.h>

struct LineSegment {
    Vector2D start_point;
    Vector2D vec;
};


bool linesIntersect(Vector2D p1, Vector2D p2, Vector2D p3, Vector2D p4,
                    Vector2D &intersect_point);

float rectangleArea(LineSegment *rectangle);
std::vector<Vector2D> getCorners(LineSegment *rectangle);


std::vector<Vector2D> convexHull(std::vector<Vector2D> points){
    std::vector<Vector2D> result;

    Vector2D cur_point, cur_vec(0.0f, 1.0f);

    // Find the topmost point
    for(unsigned i = 0; i < points.size(); i++){
        if(i == 0 || points[i].y > cur_point.y){
            cur_point = points[i];
        }
    }

    result.push_back(cur_point);

    while(true){
        Vector2D best_point;
        float cur_best_angle = 0.0f;
        bool have_found = false;

        float cur_angle = atan2(cur_vec.x, cur_vec.y);
        while(cur_angle < 0.0f){ cur_angle += 2.0f*(float)M_PI; }

        for(unsigned i = 0; i < points.size(); i++){
            Vector2D to_point = points[i] - cur_point;
            if(to_point.length() < 0.001f){ continue; }
            to_point.normalise();

            float new_angle = atan2(to_point.x, to_point.y);
            while(new_angle < 0.0f){ new_angle += 2.0f*(float)M_PI; }

            float diff = new_angle - cur_angle;
            while(diff < 0.0f){ diff += 2.0f*(float)M_PI; }

            if(!have_found || diff > cur_best_angle){
                have_found = true;
                cur_best_angle = diff;
                best_point = points[i];
            }
        }

        cur_vec = best_point - cur_point;
        cur_vec.normalise();
        cur_point = best_point;

        if((cur_point-result.front()).length() < 0.0001f){
            break;
        }

        result.push_back(cur_point);
    }

    return result;
}


std::vector<Vector2D> minimalBoundingRectangle(const std::vector<Vector2D> &convex_hull){
    assert(convex_hull.size() > 0);

    Vector2D minxP, minyP, maxxP, maxyP; // extreme vertices of the convex hull.
    minxP = minyP = maxxP = maxyP = convex_hull[0];

    unsigned minxI = 0, minyI = 0, maxxI = 0, maxyI = 0;

    for(unsigned i = 0; i < convex_hull.size(); i++){
        if(convex_hull[i].x < minxP.x){ minxP = convex_hull[i]; minxI = i; }
        if(convex_hull[i].y < minyP.y){ minyP = convex_hull[i]; minyI = i; }
        if(convex_hull[i].x > maxxP.x){ maxxP = convex_hull[i]; maxxI = i; }
        if(convex_hull[i].y > maxyP.y){ maxyP = convex_hull[i]; maxyI = i; }
    }

    LineSegment current_rectangle[4];
    unsigned cur_indices[4] = { minxI, minyI, maxxI, maxyI };

    current_rectangle[0].start_point = minxP;
    current_rectangle[1].start_point = minyP;
    current_rectangle[2].start_point = maxxP;
    current_rectangle[3].start_point = maxyP;

    current_rectangle[0].vec = Vector2D(0.0f, -1.0f);
    current_rectangle[1].vec = Vector2D(1.0f, 0.0f);
    current_rectangle[2].vec = Vector2D(0.0f, 1.0f);
    current_rectangle[3].vec = Vector2D(-1.0f, 0.0f);


    float min_area = rectangleArea(current_rectangle);
    std::vector<Vector2D> result = getCorners(current_rectangle);

    for(unsigned i = 0; i < convex_hull.size(); i++){
        float min_rot = 0.0f;
        unsigned pivot_index = 0;

        for(unsigned j = 0; j < 4; j++){
            Vector2D next_edge = convex_hull[(cur_indices[j]+1)%convex_hull.size()] - convex_hull[cur_indices[j]];
            next_edge.normalise();

            float angle = acosf(current_rectangle[j].vec.dotProduct(next_edge));
            if(j == 0 || angle < min_rot){
                min_rot = angle;
                pivot_index = j;
            }
        }

        cur_indices[pivot_index] = (cur_indices[pivot_index]+1)%convex_hull.size();

        for(unsigned j = 0; j < 4; j++){
            current_rectangle[j].vec.rotate(min_rot);
        }
        current_rectangle[pivot_index].start_point = convex_hull[cur_indices[pivot_index]];

        float area = rectangleArea(current_rectangle);
        if(area < min_area){
            min_area = area;
            result = getCorners(current_rectangle);
        }
    }

    return result;
}

bool pointInConvexHull(Vector2D point, const std::vector<Vector2D> &hull){
    // TODO: dynamically work out how far to project
    Vector2D projected_point = point + Vector2D(1000.0f, 0.0f);
    unsigned num_intersects = 0;

    for(unsigned i = 0; i < hull.size(); i++){
        Vector2D tmp;
        if(linesIntersect(point, projected_point, hull[i], hull[(i+1)%hull.size()], tmp)){
            num_intersects++;
        }
    }

    return (num_intersects == 1);
}

void expandConvexHull(std::vector<Vector2D> &hull, float expand_amount){
    std::vector<Vector2D> new_hull;

    for(int i = 0; i < (int)hull.size(); i++){
        Vector2D left = hull[i] - hull[(i-1+hull.size())%hull.size()];
        Vector2D right =  hull[(i+1)%hull.size()] - hull[i];

        left.normalise();
        right.normalise();

        left = left.perpendicular();
        right = right.perpendicular();

        float dot = left.dotProduct(right);
        float l = expand_amount/cos(acos(dot)/2.0f);

        Vector2D adj = left + right;
        adj.normalise();

        new_hull.push_back(hull[i] + adj*l);
    }

    hull = new_hull;
}


// P1 and P2 are the extents of the first line segment
// P3 and P4 are the extents of the second line segment
bool linesIntersect(Vector2D p1, Vector2D p2, Vector2D p3, Vector2D p4,
                    Vector2D &intersect_point){

    const float epsilon = 0.000001f;
    float denominator = (p4.y - p3.y)*(p2.x - p1.x) - (p4.x - p3.x)*(p2.y - p1.y);

    if(fabs(denominator) < epsilon){ return false; } // lines are parallel.

    float numerator1 = (p4.x - p3.x)*(p1.y - p3.y) - (p4.y - p3.y)*(p1.x - p3.x);
    float numerator2 = (p2.x - p1.x)*(p1.y - p3.y) - (p2.y - p1.y)*(p1.x - p3.x);

    float u1 = numerator1/denominator;
    float u2 = numerator2/denominator;

    if(u1 < -epsilon || u1 > 1.0f+epsilon){ return false; } // intesect point falls outside the segment.
    if(u2 < -epsilon || u2 > 1.0f+epsilon){ return false; }

    intersect_point.x = p1.x + u1*(p2.x - p1.x);
    intersect_point.y = p1.y + u1*(p2.y - p1.y);

    return true;
}

float rectangleArea(LineSegment *rectangle){
    std::vector<Vector2D> corners = getCorners(rectangle);
    assert(corners.size() == 4);

    float A = (corners[0]-corners[1]).length();
    float B = (corners[1]-corners[2]).length();
    return A*B;
}

std::vector<Vector2D> getCorners(LineSegment *rectangle){
    assert(rectangle != NULL);

    std::vector<Vector2D> corners;
    for(unsigned i = 0; i < 4; i++){
        Vector2D s1 = rectangle[i].start_point - 1000.0f*rectangle[i].vec;
        Vector2D s2 = rectangle[i].start_point + 1000.0f*rectangle[i].vec;
        Vector2D p1 = rectangle[(i+1)%4].start_point - 1000.0f*rectangle[(i+1)%4].vec;
        Vector2D p2 = rectangle[(i+1)%4].start_point + 1000.0f*rectangle[(i+1)%4].vec;

        Vector2D corner;
        if(!linesIntersect(s1, s2, p1, p2, corner)){
            std::cout << "Bad rectangle" << std::endl;
            return std::vector<Vector2D>();
        }
        corners.push_back(corner);
    }

    return corners;
}


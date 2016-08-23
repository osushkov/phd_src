

#include "ConvexHull.h"
#include <cmath>
#include <cassert>
#include <algorithm>

struct LineSegment {
    Vector2D start_point;
    Vector2D vec;
};

struct CHPoint {
    Vector2D pos;
    float angle_to_focus;
};

struct CHPointComp {
    bool operator()(const Vector2D &p0, const Vector2D &p1){
        if(p0.x < p1.x){
            return true;
        }
        else if(p0.x == p1.x && p0.y < p1.y){
            return true;
        }
        else {
            return false;
        }
    }
};


bool linesIntersect(Vector2D p1, Vector2D p2, Vector2D p3, Vector2D p4,
                    Vector2D &intersect_point);

float rectangleArea(LineSegment *rectangle);
std::vector<Vector2D> getCorners(LineSegment *rectangle);


inline float isLeft(Vector2D P0, Vector2D P1, Vector2D P2 ){
    return (P1.x - P0.x)*(P2.y - P0.y) - (P2.x - P0.x)*(P1.y - P0.y);
}
//===================================================================


// chainHull_2D(): Andrew's monotone chain 2D convex hull algorithm
//     Input:  P[] = an array of 2D points
//                   presorted by increasing x- and y-coordinates
//             n = the number of points in P[]
//     Output: H[] = an array of the convex hull vertices (max is n)
//     Return: the number of points in H[]
int
chainHull_2D( Vector2D* P, int n, Vector2D* H )
{
    // the output array H[] will be used as the stack
    int    bot=0, top=(-1);  // indices for bottom and top of the stack
    int    i;                // array scan index

    // Get the indices of points with min x-coord and min|max y-coord
    int minmin = 0, minmax;
    float xmin = P[0].x;
    for (i=1; i<n; i++){
        if (P[i].x != xmin){
            break;
        }
    }
    minmax = i-1;
    if (minmax == n-1) {       // degenerate case: all x-coords == xmin
        H[++top] = P[minmin];
        if (P[minmax].y != P[minmin].y){ // a nontrivial segment
            H[++top] = P[minmax];
        }
        H[++top] = P[minmin];           // add polygon endpoint
        return top+1;
    }

    // Get the indices of points with max x-coord and min|max y-coord
    int maxmin, maxmax = n-1;
    float xmax = P[n-1].x;
    for (i=n-2; i>=0; i--)
        if (P[i].x != xmax) break;
    maxmin = i+1;

    // Compute the lower hull on the stack H
    H[++top] = P[minmin];      // push minmin point onto stack
    i = minmax;
    while (++i <= maxmin)
    {
        // the lower line joins P[minmin] with P[maxmin]
        if (isLeft( P[minmin], P[maxmin], P[i]) >= 0 && i < maxmin)
            continue;          // ignore P[i] above or on the lower line

        while (top > 0)        // there are at least 2 points on the stack
        {
            // test if P[i] is left of the line at the stack top
            if (isLeft( H[top-1], H[top], P[i]) > 0)
                break;         // P[i] is a new hull vertex
            else
                top--;         // pop top point off stack
        }
        H[++top] = P[i];       // push P[i] onto stack
    }

    // Next, compute the upper hull on the stack H above the bottom hull
    if (maxmax != maxmin)      // if distinct xmax points
        H[++top] = P[maxmax];  // push maxmax point onto stack
    bot = top;                 // the bottom point of the upper hull stack
    i = maxmin;
    while (--i >= minmax)
    {
        // the upper line joins P[maxmax] with P[minmax]
        if (isLeft( P[maxmax], P[minmax], P[i]) >= 0 && i > minmax)
            continue;          // ignore P[i] below or on the upper line

        while (top > bot)    // at least 2 points on the upper stack
        {
            // test if P[i] is left of the line at the stack top
            if (isLeft( H[top-1], H[top], P[i]) > 0)
                break;         // P[i] is a new hull vertex
            else
                top--;         // pop top point off stack
        }
        H[++top] = P[i];       // push P[i] onto stack
    }
    if (minmax != minmin)
        H[++top] = P[minmin];  // push joining endpoint onto stack

    return top+1;
}

float ccw(Vector2D p1, Vector2D p2, Vector2D p3){
    return (p2.x - p1.x)*(p3.y - p1.y) - (p2.y - p1.y)*(p3.x - p1.x);
}

std::vector<Vector2D> convexHull(std::vector<Vector2D> points){
    if(points.size() <= 2){
        return points;
    }

    CHPointComp comp;
    sort(points.begin(), points.end(), comp);

    Vector2D *input = new Vector2D[points.size()];
    Vector2D *output = new Vector2D[points.size()+1];

    for(unsigned i = 0; i < points.size(); i++){
        input[i] = points[i];
    }

    int num_r = chainHull_2D(input, points.size(), output);
    std::vector<Vector2D> result;
    for(int i = 0; i < num_r; i++){
        result.push_back(output[i]);
    }

    delete[] input;
    delete[] output;
    return result;
/*
    unsigned focus_index = 0;
    Vector2D cur_focus(0.0f, 0.0f);

    for(unsigned i = 0; i < points.size(); i++){
        if(points[i].y > cur_focus.y){
            focus_index = i;
            cur_focus = points[i];
        }
        else if(points[i].y == cur_focus.y && points[i].x > cur_focus.x){
            focus_index = i;
            cur_focus = points[i];
        }
    }

    std::vector<CHPoint> ch_points;
    for(unsigned i = 0; i < points.size(); i++){
        CHPoint new_ch_point;
        new_ch_point.pos = points[i];

        if(i == focus_index){
            new_ch_point.angle_to_focus = 0.0f;
        }
        else{
            Vector2D from_focus = points[i] - cur_focus;
            new_ch_point.angle_to_focus = atan2f(from_focus.y, from_focus.x);
            while(new_ch_point.angle_to_focus < 0.0f){
                new_ch_point.angle_to_focus += 2.0f*M_PI;
            }
        }
        ch_points.push_back(new_ch_point);
    }

    CHPointComp comp;
    sort(ch_points.begin(), ch_points.end(), comp);
    for(unsigned i = 0; i < ch_points.size()-1; i++){
        assert(ch_points[i].angle_to_focus <= ch_points[i+1].angle_to_focus);
    }
    std::vector<CHPoint> stack_w;
    stack_w.push_back(ch_points[0]);
    stack_w.push_back(ch_points[1]);

    unsigned i = 2;
    while(i < ch_points.size()){
        CHPoint pt1 = stack_w.at(stack_w.size()-1);
        CHPoint pt2 = stack_w.at(stack_w.size()-2);

        if(ccw(pt1.pos, ch_points[i].pos, pt2.pos) >= 0.0f){
            stack_w.push_back(ch_points[i]);
            i++;
        }
        else{
            stack_w.pop_back();
        }
    }

    std::vector<Vector2D> result;
    for(unsigned j = 0; j < stack_w.size(); j++){
        result.push_back(stack_w[j].pos);
    }

    return result;
    */
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

bool pointInConvexHull2(Vector2D point, const std::vector<Vector2D> &hull, unsigned inc, float err){
    float sum = 0.0f;

    std::vector<Vector2D> thull;
    for(unsigned i = 0; i < hull.size(); i++){
        if(i%inc == 0){
            thull.push_back(hull[i]);
        }
    }

    for(unsigned i = 0; i < thull.size(); i++){
        unsigned next = (i+1)%thull.size();
        Vector2D v0 = thull[i] - point;
        Vector2D v1 = thull[next] - point;
        v0.normalise();
        v1.normalise();

        float d = v0.dotProduct(v1);
        float angle = acosf(d);

        sum += angle;
    }

    return fabs(sum - 2.0*M_PI) < err;
}

float pointDistanceFromHull(Vector2D point, const std::vector<Vector2D> &hull, unsigned inc){
    float least_dist = 0.0f;
    for(unsigned i = 0; i < hull.size(); i += inc){
        float d = (point-hull[i]).length();
        if(i == 0 || d < least_dist){
            least_dist = d;
        }
    }
    return least_dist;
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


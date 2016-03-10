#ifndef QUADTREE_H
#define QUADTREE_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_traits.h>
#include <pcl/for_each_type.h>
#include <pcl/Vertices.h>

// CGAL
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_2.h>


namespace QTD{

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef K::Point_2 Point;
typedef CGAL::Polygon_2<K>                                  Polygon;

class QuadTree{

public:
    enum CellType{ Interior, Boundary, Exterior, Hole, Parent};
    struct Cell{
        float x;
        float y;
        float width;
        float r;
        float g;
        float b;
        CellType cellType;
    };

private:
    float max_width_;
    int max_level_;
    int level_;
    bool use_color_;
    bool is_leaf_;
    bool is_boundary_;
    CellType cellType_;
    float x_, y_;
    float width_;
    std::vector<QuadTree> nodes;

private:
    int level(){ return level_; }
    bool useColor(){ return use_color_;}
    bool isLeaf(){ return is_leaf_; }
    CellType cellType(){ return cellType_; }

public:

    QuadTree(int level, float width, float x, float y, CellType type);
    QuadTree(int level, float width, float x, float y)
        : QuadTree(level, width, x, y, Parent){}
    QuadTree()
        : QuadTree(1,10,0,0, Parent){};

    ~QuadTree(){}

    // NOTE
    // This method is very inefficient.
    // It is checking way to many times if a point falls within the polygon
    // Makes more sense to send in a single line segment from the polygon and
    // then check for the cells if the line segment crosses it.
    // Afterwards check if remaining cells are inside or outside.
    //
    bool insertBoundary(Polygon polygon);


    void insertHole(Polygon polygon);
    void extractCells(std::vector<QuadTree::Cell> &cells);

    // Determines if color information will be used in decemation.
    void useColor(bool use_color){ use_color_ = use_color; }
    void setMaxLevel(int level){ max_level_ = level; }
    void setMaxWidth(float width){ max_width_ = width; }
    // void setX(float x){ x_ = x; }
    // void setY(float y){ y_ = y; }
    // void setWidth(float width){ width_ = width; }
    float x(){ return x_; }
    float y(){ return y_; }
    float width(){ return width_; }


private:
    void clear();
    bool maxDepth();
    void createSubNodesAndInherit();
    void createSubNodesAndInherit(CellType type);
    bool polygonCompletelyWithinCell(Polygon &polygon);
};

}
#endif

#ifndef QUADTREE_H
#define QUADTREE_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_traits.h>
#include <pcl/for_each_type.h>
#include <pcl/Vertices.h>


namespace QTD{

struct quadPoint{
    float x;
    float y;

    quadPoint(){};
    quadPoint(float x_, float y_){
        x = x_;
        y = y_;
    }

    inline bool operator==(quadPoint a) {
       if (a.x==x && a.y== y)
          return true;
       else
          return false;
    }

    inline bool operator!=(quadPoint a) {
       if (a.x!=x || a.y!= y)
          return true;
       else
          return false;
    }
};

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
    bool is_leaf_;
    CellType cellType_;
    float x_, y_;
    float width_;
    std::vector<quadPoint> intersectionPoints_;
    std::vector<quadPoint> cornerPoints_;
    std::vector<QuadTree> nodes;

private:
    int level(){ return level_; }
    bool isLeaf(){ return is_leaf_; }
    CellType cellType(){ return cellType_; }

public:

    QuadTree(int level, float width, float x, float y, CellType type);
    QuadTree(int level, float width, float x, float y)
        : QuadTree(level, width, x, y, Parent){}
    QuadTree()
        : QuadTree(1,10,0,0, Parent){};

    ~QuadTree(){}

    bool insertSegment(quadPoint a, quadPoint b);
    void markAsExternal(const std::vector<quadPoint> &polygon,
                        const std::vector<int> &holeIdx,
                        const quadPoint &min,
                        const quadPoint &max);
    // void insertHole(Polygon polygon);
    void extractCells(std::vector<QuadTree::Cell> &cells);

    // TODO: creates 3 points for each triangle, these points have ofted been created before => can be reused.
    void extractTriangles(std::vector<quadPoint> &cells, std::vector<std::vector<int> > &indices);


    void extractBoundaryPoints(std::vector<quadPoint> &points);

    // Determines if color information will be used in decemation.
    void setMaxLevel(int level){ max_level_ = level; }
    void setMaxWidth(float width){ max_width_ = width; }
    float x(){ return x_; }
    float y(){ return y_; }
    float width(){ return width_; }


private:
    void clear();
    bool maxDepth();
    void createSubNodesAndInherit();
    void createSubNodesAndInherit(CellType type);
    bool segmentIntersectsCell(quadPoint a, quadPoint b);
    bool lineOnCellIntersections(quadPoint a, quadPoint b);
    bool lineOnLineIntersectionPoint(quadPoint lstart, quadPoint lend, quadPoint cstart, quadPoint cend, quadPoint &out);
    bool rayTraycing(quadPoint p, float width, const std::vector<quadPoint> &polygon, const std::vector<int> &holeIdx);
};

}
#endif

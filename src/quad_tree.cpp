#include <quad_tree_decimation/quad_tree/quad_tree.h>

#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/Polygon_2_algorithms.h>

namespace QTD{

QuadTree::QuadTree(int level, float width, float x, float y, CellType type){
    x_ = x;
    y_ = y;
    width_ = width;
    level_ = level;
    max_width_ = -1;
    max_level_ = -1;
    use_color_ = false;
    is_leaf_ = true;
    cellType_ = type;
}

bool QuadTree::maxDepth(){

    if( level_ >= max_level_ && max_level_ > 0 ){
        return true;
    }
    if( width_ < max_width_ && max_width_> 0 ){
        return true;
    }
    return false;
}

void QuadTree::createSubNodesAndInherit(){
    QuadTree::createSubNodesAndInherit(CellType::Parent);
}

void QuadTree::createSubNodesAndInherit(CellType type){

    nodes.resize(4);
    float half =  width_/2.0;
    nodes[0] = QuadTree(level_ + 1, half, x_, y_, type);
    nodes[1] = QuadTree(level_ + 1, half, x_ + half, y_, type);
    nodes[2] = QuadTree(level_ + 1, half, x_, y_ + half, type);
    nodes[3] = QuadTree(level_ + 1, half, x_ + half, y_ + half, type);

    for(auto &n : nodes){
        n.useColor(use_color_);
        n.setMaxWidth(max_width_);
        n.setMaxLevel(max_level_);
    }

    is_leaf_ = false;

}


void QuadTree::clear(){

    // TODO either copy all points to parent node or get average color.

    // Clear all points.
    // points.clear();
    // Recursively clear all subnodes;
    for(auto &n : nodes){
        // points.insert(points.end(), n.pointsBegin(), n.pointsEnd());
        n.clear();
    }
    // Clear nodes vector;
    nodes.clear();
    is_leaf_ = true;
}


void QuadTree::markAsExternal(  const std::vector<quadPoint> &polygon,
                                const quadPoint &min,
                                const quadPoint &max){

    // Check if we are in a leaf
    if(is_leaf_){
        // Check if we are looking at a boundary
        if(cellType_ != Boundary){
            // need to check one corner of cell
            // Check 1: inside or outside of bounding box.
            if(x_ < min.x || x_ + width_ > max.x || y_ < min.y || y_ + width_ > max.y){
                // some part of cell outside --> external
                cellType_ = Exterior;
            } else {
                // need to do raytraycing
                int count = 0;
                for(size_t i = 0; i < polygon.size()-1; ++i){

                    // First check if one x component is on the right
                    if(polygon[i].x > x_+width_ || polygon[i+1].x > x_+width_){
                        // count++;
                        if(polygon[i].y == polygon[i+1].y){
                            continue;
                        }

                        else if(polygon[i].y > polygon[i+1].y){
                            if(y_ < polygon[i].y && y_ > polygon[i+1].y){
                                float k = (polygon[i].x - polygon[i+1].x) / (polygon[i].y - polygon[i+1].y);
                                float m = polygon[i].x - k * polygon[i].y;

                                if(x_ < k * y_ + m){
                                    count++;
                                }
                            }
                        } else{
                            if(y_ < polygon[i+1].y && y_ > polygon[i].y){
                                float k = (polygon[i].x - polygon[i+1].x) / (polygon[i].y - polygon[i+1].y);
                                float m = polygon[i].x - k * polygon[i].y;

                                if(x_ < k * y_ + m){
                                    count++;
                                }
                            }
                        }
                    }
                }
                if(!(count % 2)){
                    cellType_ = Exterior;
                }
            }
        }
    } else {
        for(auto &n : nodes){
            n.markAsExternal(polygon, min, max);
        }
    }

}

bool QuadTree::insertSegment(quadPoint a, quadPoint b){

    bool doesIntersect = segmentIntersectsCell(a,b);
    // std::cout << "intersects: " << doesIntersect << std::endl;
    // std::cout << " " << std::endl;
    if( doesIntersect ){
        if( !QuadTree::maxDepth() ){
            // Create new leafs.
            if(is_leaf_){
                QuadTree::createSubNodesAndInherit();
            }
            for(auto &n : nodes){
                n.insertSegment(a,b);
            }
        } else {
            cellType_ = Boundary;
        }
        return true;
    } else {
        if(is_leaf_ && cellType_ != Boundary)
            cellType_ = Interior;
        return false;
    }
}

bool QuadTree::segmentIntersectsCell(quadPoint a, quadPoint b){
    // Check if line intersects cells x-axis shadow

    // std::cout << "cell x: " << x_ << " y_: " << y_ << " width_: " << width_ << std::endl;
    // std::cout << "A x: " << a.x << " y: " << a.y << std::endl;
    // std::cout << "B x: " << b.x << " y: " << b.y << std::endl;


    if(a.x > x_+width_ && b.x > x_+width_) return false;
    if(a.y > y_+width_ && b.y > y_+width_) return false;
    if(a.x < x_ && b.x < x_) return false;
    if(a.y < y_ && b.y < y_) return false;

    // y and x axis shadows do intersect.
    // Check if all for cell corners lie on the same side of the line.
    float y_slope = (b.y-a.y);
    float x_slope = (a.x-b.x);
    float m = b.x*a.y-a.x*b.y;
    bool topleftSign = x_slope*(y_+width_) + y_slope*x_ +  m >= 0;

    // Check if rest of corners have same sign
    if( x_slope*(y_+width_) + y_slope*(x_+width_) +  m > 0 != topleftSign ) return true;
    if( x_slope*(y_) + y_slope*(x_+width_) +  m > 0 != topleftSign ) return true;
    if( x_slope*(y_) + y_slope*(x_) +  m > 0 != topleftSign ) return true;

    // All corner points lie on the same side of the line
    return false;
}


bool QuadTree::insertBoundary(Polygon polygon){

    Polygon cell;
    cell.push_back( Point(x_,          y_) );
    cell.push_back( Point(x_ + width_, y_) );
    cell.push_back( Point(x_ + width_, y_ + width_) );
    cell.push_back( Point(x_,          y_+width_) );


    // check to see if polygon is contained completely by the cell
    if(QuadTree::polygonCompletelyWithinCell(polygon)){
        if( !QuadTree::maxDepth() ){
            // Create new leafs.
            QuadTree::createSubNodesAndInherit();
            for(auto &n : nodes){
                n.insertBoundary(polygon);
            }
        } else {
            cellType_ = Boundary;
        }
    }


    if(cellType_ == Parent){

        // This cell hasn't been processed
        bool intersect = CGAL::do_intersect(polygon, cell);
        if(intersect){


            // need to check if cell completely inside polygon
            bool inside = false;
            bool outside = false;
            bool edge = false;
            for (size_t i = 0; i < cell.size(); i++) {
                auto bounded = CGAL::bounded_side_2(polygon.vertices_begin(), polygon.vertices_end(), cell[i], K());

                if(bounded == CGAL::ON_BOUNDED_SIDE){
                    // std::cout << " is inside the polygon.\n";
                    inside = true;
                } else if(bounded == CGAL::ON_BOUNDARY){
                    // std::cout << " The polygon and cells edges touch.\n";
                    edge = true;
                } else {
                    // std::cout << " is outside the polygon.\n";
                    outside = true;
                }
            }

            if(inside && outside || edge || (outside && !inside && !edge) ){
                // The boundary travels through the cell.
                // if the cell isn't a smallest possible size or biggest depth split it.
                if( !QuadTree::maxDepth() ){
                    // Create new leafs.
                    QuadTree::createSubNodesAndInherit();
                    for(auto &n : nodes){
                        n.insertBoundary(polygon);
                    }
                } else {
                    cellType_ = Boundary;
                }
            } else if(inside){
                cellType_ = Interior;
            } else {
                cellType_ = Exterior;
            }

        } else {
            // std::cout << "does not intersect" << std::endl;
            cellType_ = Exterior;
            return false;
        }
    }
    return true;
}


void QuadTree::insertHole(Polygon polygon){

    // Check to see if exterior node.
    // check to see if polygon is contained completely by the cell
    if(QuadTree::polygonCompletelyWithinCell(polygon)){
        if(cellType_ == Parent){
            for(auto &n : nodes){
                n.insertHole(polygon);
            }
        } else if( !QuadTree::maxDepth() && cellType_ == Interior ){
            // Create new leafs.
            QuadTree::createSubNodesAndInherit(Interior);
            for(auto &n : nodes){
                n.insertHole(polygon);
            }
        } else {
            cellType_ = Boundary;
        }
        return;
    }

    Polygon cell;
    cell.push_back( Point(x_,          y_) );
    cell.push_back( Point(x_ + width_, y_) );
    cell.push_back( Point(x_ + width_, y_ + width_) );
    cell.push_back( Point(x_,          y_+width_) );

    bool intersect = CGAL::do_intersect(polygon, cell);

    if(intersect){
        // std::cout << "intersect: " << cellType << std::endl;
        if(cellType_ == Interior){
            // std::cout << "Interior" << std::endl;
            // need to check if cell completely inside polygon
            bool inside = false;
            bool outside = false;
            bool edge = false;
            for (size_t i = 0; i < cell.size(); i++) {
                auto bounded = CGAL::bounded_side_2(polygon.vertices_begin(), polygon.vertices_end(), cell[i], K());

                if(bounded == CGAL::ON_BOUNDED_SIDE){
                    // std::cout << " is inside the polygon.\n";
                    inside = true;
                } else if(bounded == CGAL::ON_BOUNDARY){
                    // std::cout << " The polygon and cells edges touch.\n";
                    edge = true;
                } else {
                    // std::cout << " is outside the polygon.\n";
                    outside = true;
                }
            }

            if(inside && outside || edge || (outside && !inside && !edge) ){
                // The boundary travels through the cell.
                // if the cell isn't a smallest possible size or biggest depth split it.
                if(cellType_ == Parent){
                    for(auto &n : nodes){
                        n.insertHole(polygon);
                    }
                }
                else if( !QuadTree::maxDepth() ){
                    // Create new leafs.
                    QuadTree::createSubNodesAndInherit(Interior);
                    for(auto &n : nodes){
                        n.insertHole(polygon);
                    }
                } else {
                    cellType_ = Boundary;
                }
            } else if(inside){
                cellType_ = Hole;
            } else {
                cellType_ = Interior;
            }
        } else if(cellType_ == Parent){
            for(auto &n : nodes){
                n.insertHole(polygon);
            }
        }
    }
}

bool QuadTree::polygonCompletelyWithinCell(Polygon &polygon){
    auto left = CGAL::left_vertex_2(polygon.vertices_begin(), polygon.vertices_end(), K());
    auto right = CGAL::right_vertex_2(polygon.vertices_begin(), polygon.vertices_end(), K());
    auto top = CGAL::top_vertex_2(polygon.vertices_begin(), polygon.vertices_end(), K());
    auto bottom = CGAL::bottom_vertex_2(polygon.vertices_begin(), polygon.vertices_end(), K());
    if( (*left)[0] >= x_) {
        if( (*right)[0] <= x_ + width_) {
            if( (*bottom)[1] >= y_) {
                if( (*top)[1] <= y_ + width_) {
                    return true;
                }
            }
        }
    }
    return false;
}


void QuadTree::extractCells(std::vector<QuadTree::Cell> &cells){

    if(is_leaf_ && cellType_ != Exterior && cellType_ != Hole){
    // if(is_leaf_  && cellType_ != Hole){
        QuadTree::Cell c;
        c.x = x_;
        c.y = y_;
        c.width = width_;
        c.r = 0;
        c.g = 0;
        c.b = 0;
        c.cellType = cellType_;
        cells.push_back(c);
    } else {
        for(auto &n : nodes){
            n.extractCells(cells);
        }
    }

}

}

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
    is_leaf_ = true;
    cellType_ = type;
    intersectionPoints_.resize(0);

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
                                const std::vector<int> &holeIdx,
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
            } else if( !QuadTree::rayTraycing(quadPoint(x_,y_), width_, polygon, holeIdx) ){
                cellType_ = Exterior;
            }
        }
        else if(cellType_ == Boundary){
            // auto last = std::unique(intersectionPoints_.begin(), intersectionPoints_.end());
            // intersectionPoints_.erase(last, intersectionPoints_.end());

            std::vector<quadPoint> tmpPoints;
            quadPoint q(x_,y_);
            if(QuadTree::rayTraycing( q, 0, polygon, holeIdx) ){
                // q is inside polygon, add to points
                cornerPoints_.push_back(q);
            }
            q.x += width_;
            if(QuadTree::rayTraycing( q, 0, polygon, holeIdx) ){
                // q is inside polygon, add to points
                cornerPoints_.push_back(q);
            }
            q.y += width_;
            if(QuadTree::rayTraycing( q, 0, polygon, holeIdx) ){
                // q is inside polygon, add to points
                cornerPoints_.push_back(q);
            }
            q.x -= width_;
            if(QuadTree::rayTraycing( q, 0, polygon, holeIdx) ){
                // q is inside polygon, add to points
                cornerPoints_.push_back(q);
            }

        }


    } else {
        for(auto &n : nodes){
            n.markAsExternal(polygon, holeIdx, min, max);
        }
    }

}

bool QuadTree::insertSegment(quadPoint a, quadPoint b){

    bool doesIntersect = segmentIntersectsCell(a,b);
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
            // std::cout << "planning to find intersection points" << std::endl;
            if(QuadTree::lineOnCellIntersections(a, b)){
                // std::cout << "found intersection points" << std::endl;
            }

        }
        return true;
    } else {
        if(is_leaf_ && cellType_ != Boundary)
            cellType_ = Interior;
        return false;
    }
}

bool QuadTree::lineOnCellIntersections(quadPoint a, quadPoint b){
    // start by checking if only one point is inside the cell

    auto insertPoints = [](quadPoint p, std::vector<quadPoint> &intersectionPoints){
        if(intersectionPoints.size() == 0 || intersectionPoints.back() != p){
            intersectionPoints.push_back(p);
        }
    };

    int count_inside_before = 0;
    bool ainsert = false, binsert = false;
    int count_inside = 0;
    if(a.x >= x_ && a.x <= x_+width_ && a.y >= y_ && a.y <= y_+width_){
        if(intersectionPoints_.size() != 0){
            insertPoints(a, intersectionPoints_);
        } else {
            ainsert = true;
        }
        // if(intersectionPoints_.back() != a)
        //     intersectionPoints_.push_back(a);
        count_inside++;
    }
    if(b.x >= x_ && b.x <= x_+width_ && b.y >= y_ && b.y <= y_+width_){
        if(intersectionPoints_.size() != 0){
            insertPoints(b, intersectionPoints_);
        } else {
            binsert = true;
        }
        // if(intersectionPoints_.back() != b)
        //     intersectionPoints_.push_back(b);
        count_inside++;
    }
    count_inside_before = count_inside;
    std::vector<quadPoint> dafuck;
    quadPoint cstart, cend, out;
    // left bottom to right bottom
    cstart.x = x_;
    cstart.y = y_;
    cend.x = x_+ width_;
    cend.y = y_;
    if(QuadTree::lineOnLineIntersectionPoint(a,b,cstart,cend,out)){
        insertPoints(out, intersectionPoints_);
        count_inside++;
        // if(intersectionPoints_.back() != out)
        //     intersectionPoints_.push_back(out);
    }
    dafuck.push_back(out);
    // right bottom to right top
    cstart.x = x_ + width_;
    cstart.y = y_;
    cend.x = x_+ width_;
    cend.y = y_ + width_;
    if(QuadTree::lineOnLineIntersectionPoint(a,b,cstart,cend,out)){
        insertPoints(out, intersectionPoints_);
        count_inside++;
        // if(intersectionPoints_.back() != out)
        //     intersectionPoints_.push_back(out);
    }
    dafuck.push_back(out);

    // right top to left top
    cstart.x = x_ + width_;
    cstart.y = y_ + width_;
    cend.x = x_;
    cend.y = y_ + width_;
    if(QuadTree::lineOnLineIntersectionPoint(a,b,cstart,cend,out)){
        insertPoints(out, intersectionPoints_);
        count_inside++;
        // if(intersectionPoints_.back() != out)
        //     intersectionPoints_.push_back(out);
    }
    dafuck.push_back(out);

    // left top to left bottom
    cstart.x = x_;
    cstart.y = y_ + width_;
    cend.x = x_;
    cend.y = y_;
    if(QuadTree::lineOnLineIntersectionPoint(a,b,cstart,cend,out)){
        insertPoints(out, intersectionPoints_);
        count_inside++;
        // if(intersectionPoints_.back() != out)
        //     intersectionPoints_.push_back(out);
    }
    dafuck.push_back(out);

    if(ainsert){
        insertPoints(a, intersectionPoints_);
        count_inside++;
    }
    if(binsert){
        insertPoints(b, intersectionPoints_);
        count_inside++;
    }

    if(count_inside < 2){
        std::cout << "before: " << count_inside_before << std::endl;
        std::cout << "DAFUCKKKKKKKKKKKKKK" << std::endl;
        std::cout << "cell: " << x_ << ", " << y_ << ", " << width_ << std::endl;
        std::cout << "pointa: " << a.x << "," << a.y << std::endl;
        std::cout << "pointb: " << b.x << "," << b.y << std::endl;
        for(auto p : dafuck){
            std::cout << "out: " << p.x << ", " << p.y << std::endl;
        }
    }

    if(intersectionPoints_.size() > 0)
        return true;
    return false;
}

bool QuadTree::lineOnLineIntersectionPoint(quadPoint lstart, quadPoint lend, quadPoint cstart, quadPoint cend, quadPoint &out){

    auto equality = [](float A, float B){
        // Calculate the difference.
        float diff = std::fabs(A - B);
        A = std::fabs(A);
        B = std::fabs(B);
        // Find the largest
        float largest = (B > A) ? B : A;

        if (diff <= largest * FLT_EPSILON)
            return true;
        return false;

    };


    float den = (lstart.x-lend.x)*(cstart.y-cend.y) - (lstart.y-lend.y)*(cstart.x-cend.x);
    float numa = lstart.x*lend.y - lstart.y*lend.x;
    float numb = cstart.x*cend.y - cstart.y*cend.x;

    out.x = (numa*(cstart.x-cend.x) - (lstart.x-lend.x)*numb) / den;
    out.y = (numa*(cstart.y-cend.y) - (lstart.y-lend.y)*numb) / den;
    // std::cout << "out x: " << out.x << ", y: " << out.y << std::endl;
    // std::cout << "cell: x: " << x_ << ", y: " << y_ << ", width: " << width_ << std::endl;
    // std::cout << "lstart: x_ "<< lstart.x << ", y_ " << lstart.y << std::endl;
    // std::cout << "lend: x_ "<< lend.x << ", y_ " << lend.y << std::endl;
    // check if point is within both line segments


    // if(equality(out.x, std::max(lstart.x,lend.x)) && equality(out.x, std::min(lstart.x,lend.x)))

    if(out.x <= std::max(lstart.x,lend.x) + 0.001 && out.x >= std::min(lstart.x,lend.x) - 0.001){
        if(out.y <= std::max(lstart.y,lend.y) + 0.001 && out.y >= std::min(lstart.y,lend.y) - 0.001){
            if(out.x <= std::max(cstart.x,cend.x) + 0.001 && out.x >= std::min(cstart.x,cend.x) - 0.001){
                if(out.y <= std::max(cstart.y,cend.y) + 0.001 && out.y >= std::min(cstart.y,cend.y) - 0.001){
                    return true;
                }
            }
        }
    }
    return false;

}


bool QuadTree::segmentIntersectsCell(quadPoint a, quadPoint b){
    // Check if line intersects cells x-axis shadow

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


void QuadTree::extractBoundaryPoints(std::vector<quadPoint> &points){
    if(cellType_ == Boundary){
        for(auto point : intersectionPoints_){
            points.push_back(point);
        }
        for(auto point : cornerPoints_){
            points.push_back(point);
        }
    } else {
        for(auto &n : nodes){
            n.extractBoundaryPoints(points);
        }
    }
}

void QuadTree::extractTriangles(std::vector<quadPoint> &cells, std::vector<std::vector<int>> &vertices){
    if(is_leaf_){
        if(cellType_ == Interior){
            // Create a quadpoint for each corners.
            std::vector<int> vert(3);
            vert[0] = cells.size() + 0;
            vert[1] = cells.size() + 1;
            vert[2] = cells.size() + 2;
            vertices.push_back(vert);

            vert[0] = cells.size() + 0;
            vert[1] = cells.size() + 2;
            vert[2] = cells.size() + 3;
            vertices.push_back(vert);

            cells.push_back(quadPoint(x_,           y_));
            cells.push_back(quadPoint(x_ + width_,  y_));
            cells.push_back(quadPoint(x_ + width_,  y_ + width_));
            cells.push_back(quadPoint(x_,           y_ + width_));
        }

        else if(cellType_ == Boundary){

            auto pointDist = [](quadPoint a, quadPoint b){
                return std::sqrt(std::pow(b.x-a.x,2) + std::pow(b.y-a.y,2));
            };

            auto pointAngleX = [](quadPoint a, quadPoint b){
                return atan2(b.y-a.y,b.x-a.x);
            };

            auto pointAngleY = [](quadPoint a, quadPoint b){
                return atan2(b.x-a.x,b.y-a.y);
            };

            if(cornerPoints_.size() == 2){
                // find average corner point
                quadPoint avePoint((cornerPoints_[0].x + cornerPoints_[1].x) / 2.0, (cornerPoints_[0].y + cornerPoints_[1].y) / 2.0);

                std::vector<std::pair<quadPoint, float>> itmp;
                itmp.reserve(intersectionPoints_.size());
                for(auto i : intersectionPoints_){
                    if(cornerPoints_[0].x == cornerPoints_[1].x){
                        itmp.push_back(std::pair<quadPoint,float>(i,pointAngleX(avePoint, i)));
                    } else {
                        itmp.push_back(std::pair<quadPoint,float>(i,pointAngleY(avePoint, i)));
                    }
                }
                for(auto i : cornerPoints_){
                    if(cornerPoints_[0].x == cornerPoints_[1].x){
                        itmp.push_back(std::pair<quadPoint,float>(i,pointAngleX(avePoint, i)));
                    } else {
                        itmp.push_back(std::pair<quadPoint,float>(i,pointAngleY(avePoint, i)));
                    }
                }
                std::sort(itmp.begin(),itmp.end(), [](const std::pair<quadPoint,float> &left, const std::pair<quadPoint,float> &right) {
                    return left.second < right.second;
                });
                std::vector<int> vert(3);
                for(int i = 1; i < itmp.size()-1; ++i){
                    vert[0] = cells.size() + 0;
                    vert[1] = cells.size() + i;
                    vert[2] = cells.size() + i+1;
                    vertices.push_back(vert);
                }
                for(auto p : itmp){
                    cells.push_back(p.first);
                }

}

            // } else if(cornerPoints_.size() == 1){
            //     // find average corner point
            //     quadPoint avePoint(cornerPoints_[0].x, cornerPoints_[0].y);
            //
            //     std::vector<std::pair<quadPoint, float>> itmp;
            //     itmp.reserve(intersectionPoints_.size());
            //     for(auto i : intersectionPoints_){
            //         itmp.push_back(std::pair<quadPoint,float>(i,pointAngleX(avePoint, i)));
            //     }
            //     std::sort(itmp.begin(),itmp.end(), [](const std::pair<quadPoint,float> &left, const std::pair<quadPoint,float> &right) {
            //         return left.second < right.second;
            //     });
            //
            //     itmp.insert(itmp.begin(), std::pair<quadPoint, float>(avePoint, 0));
            //     std::vector<int> vert(3);
            //     for(int i = 1; i < itmp.size()-1; ++i){
            //         vert[0] = cells.size() + 0;
            //         vert[1] = cells.size() + i;
            //         vert[2] = cells.size() + i+1;
            //         vertices.push_back(vert);
            //     }
            //     for(auto p : itmp){
            //         cells.push_back(p.first);
            //     }
            // }


             else {
                std::vector<int> vert(3);
                vert[0] = cells.size() + 0;
                vert[1] = cells.size() + 1;
                vert[2] = cells.size() + 2;
                vertices.push_back(vert);

                vert[0] = cells.size() + 0;
                vert[1] = cells.size() + 2;
                vert[2] = cells.size() + 3;
                vertices.push_back(vert);

                cells.push_back(quadPoint(x_,           y_));
                cells.push_back(quadPoint(x_ + width_,  y_));
                cells.push_back(quadPoint(x_ + width_,  y_ + width_));
                cells.push_back(quadPoint(x_,           y_ + width_));
            }


        }

    } else {
        for(auto &n : nodes){
            n.extractTriangles(cells, vertices);
        }
    }
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


bool QuadTree::rayTraycing(quadPoint p, float width, const std::vector<quadPoint> &polygon, const std::vector<int> &holeIdx){
    // need to do raytraycing
    int count = 0;
    // holeIdx.push_back(polygon.size());
    auto split = holeIdx.begin();
    split++;
    // std::cout << "split: " << *split << std::endl;
    // int holesegment;
    for(size_t i = 0; i < polygon.size(); ++i){
        if(i+1 == *split){
            split++;
            continue;
        }

        // First check if one x component is on the right
        if(polygon[i].x > p.x+width || polygon[i+1].x > p.x+width){
            // count++;
            if(polygon[i].y == polygon[i+1].y){
                continue;
            }

            else if(polygon[i].y > polygon[i+1].y){
                if(p.y < polygon[i].y && p.y > polygon[i+1].y){
                    float k = (polygon[i].x - polygon[i+1].x) / (polygon[i].y - polygon[i+1].y);
                    float m = polygon[i].x - k * polygon[i].y;

                    if(p.x < k * p.y + m){
                        count++;
                    }
                }
            } else{
                if(p.y < polygon[i+1].y && p.y > polygon[i].y){
                    float k = (polygon[i].x - polygon[i+1].x) / (polygon[i].y - polygon[i+1].y);
                    float m = polygon[i].x - k * polygon[i].y;

                    if(p.x < k * p.y + m){
                        count++;
                    }
                }
            }
        }
    }
    return count % 2;
}

}

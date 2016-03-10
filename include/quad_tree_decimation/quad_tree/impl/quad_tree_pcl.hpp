#include <pcl/common/transforms.h>
#include <algorithm>
#include <pcl/common/common.h>

namespace QTD{

template <typename PointT>
void QuadTreePCL<PointT>::insertBoundary(typename pcl::PointCloud<PointT>::Ptr boundary){
    // Convert to cgal polygon
    if(boundary->size() == 0) return;

    QuadTreePCL<PointT>::rotateToAxis(boundary);
    PointT min, max;
    pcl::getMinMax3D(*boundary, min, max);

    std::cout << "min: " << min << std::endl;
    std::cout << "max: " << max << std::endl;

    if(!inserted_){
        // Determin the inital size of the quadtree
        z_ = boundary->points[0].z;
        float x = QuadTreePCL<PointT>::roundDown(min.x);
        float y = QuadTreePCL<PointT>::roundDown(min.y);
        float width = std::max(QuadTreePCL<PointT>::roundUp(max.x - x), QuadTreePCL<PointT>::roundUp(max.y - y));

        std::cout << "x: " << x << std::endl;
        std::cout << "y: " << y << std::endl;
        std::cout << "width: " << width << std::endl;

        // initialize the quadtree
        quad = QuadTree(1,width,x,y);
        quad.setMaxWidth(0.05);
    }

    Polygon polygon;
    for (size_t i = 0; i < boundary->size(); i++) {
        polygon.push_back(Point(boundary->points[i].x, boundary->points[i].y));
    }
    std::vector<Polygon> polygons;
    // if(!polygon.is_simple()){
    std::cout << "Polygon isn't simple, returning" << std::endl;
    QuadTreePCL<PointT>::makePolygonSimple(polygon, polygons, 0.05);
    polygon = polygons[0];
        // return;
    // }
    CGAL::Orientation orientation = polygon.orientation();
    if(orientation == CGAL::NEGATIVE){
        std::cout << "need to invert polygon" << std::endl;
        std::reverse(polygon.vertices_begin(), polygon.vertices_end());
    }
    std::cout << "inserting" << std::endl;
    quad.insertBoundary(polygon);
    for(size_t i = 1; i < polygons.size(); ++i){
        orientation = polygons[i].orientation();
        if(orientation == CGAL::NEGATIVE){
            std::cout << "need to invert polygon" << std::endl;
            std::reverse(polygons[i].vertices_begin(), polygons[i].vertices_end());
        }
        quad.insertHole(polygons[i]);
    }
    std::cout << "inserted" << std::endl;
    inserted_ = true;
}

template <typename PointT>
template <typename T>
void QuadTreePCL<PointT>::createMesh(typename pcl::PointCloud<T>::Ptr cloud, std::vector< pcl::Vertices > &vertices){

    std::vector<QuadTree::Cell> cells;
    quad.extractCells(cells);

    cloud->reserve(cells.size() * 4);
    vertices.reserve(cells.size() * 2);

    T p;
    p.z = z_;
    pcl::Vertices vert;
    vert.vertices.resize(3);
    for(auto c : cells){
        int size = cloud->size();
        if(c.cellType == QuadTree::Interior){
            p.r = 255; p.g = 255; p.b = 255;
        } else if(c.cellType == QuadTree::Boundary){
            p.r = 255; p.g = 255; p.b = 0;
        } else {
            p.r = 255; p.g = 0; p.b = 255;
        }
        p.x = c.x;
        p.y = c.y;
        cloud->push_back(p);
        p.x = c.x + c.width;
        p.y = c.y;
        cloud->push_back(p);
        p.x = c.x + c.width;
        p.y = c.y + c.width;
        cloud->push_back(p);
        p.x = c.x;
        p.y = c.y + c.width;
        cloud->push_back(p);

        vert.vertices[0] = size;
        vert.vertices[1] = size + 1;
        vert.vertices[2] = size + 2;
        vertices.push_back(vert);

        vert.vertices[0] = size;
        vert.vertices[1] = size + 2;
        vert.vertices[2] = size + 3;
        vertices.push_back(vert);

    }
    QuadTreePCL<PointT>::rotateFromAxis(cloud);
}

template <typename PointT>
void QuadTreePCL<PointT>::setNormal(Eigen::Vector3f normal){
    normal_ = normal;
    Eigen::Vector3f znorm;
    znorm << 0,0,1;
    quaternion_ = Eigen::Quaternion<float>::FromTwoVectors(normal_, znorm);
    normalVectorSet = true;
}

template <typename PointT>
void QuadTreePCL<PointT>::setNormal(pcl::ModelCoefficients::Ptr coeff){
    normal_[0] = coeff->values[0];
    normal_[1] = coeff->values[1];
    normal_[2] = coeff->values[2];
    Eigen::Vector3f znorm;
    znorm << 0,0,1;
    quaternion_ = Eigen::Quaternion<float>::FromTwoVectors(normal_, znorm);
    normalVectorSet = true;
}

template <typename PointT>
void QuadTreePCL<PointT>::rotateToAxis(typename pcl::PointCloud<PointT>::Ptr cloud){

    if(quaternion_.x() == 0 &&quaternion_.y() == 0 && quaternion_.z() == 0){
        // no rotation needed.
        return;
    }
    // first check if we need to rotate
    Eigen::Affine3f rot(quaternion_.matrix());
    pcl::transformPointCloud (*cloud, *cloud, rot);

}

template <typename PointT>
void QuadTreePCL<PointT>::rotateToAxis(typename pcl::PointCloud<PointT>::Ptr cloud, typename pcl::PointCloud<PointT>::Ptr out){

    if(quaternion_.x() == 0 &&quaternion_.y() == 0 && quaternion_.z() == 0){
        // no rotation needed.
        return;
    }
    // first check if we need to rotate
    Eigen::Affine3f rot(quaternion_.matrix());
    pcl::transformPointCloud (*cloud, *out, rot);

}

template <typename PointT>
void QuadTreePCL<PointT>::rotateFromAxis(typename pcl::PointCloud<PointT>::Ptr cloud){

    if(quaternion_.x() == 0 &&quaternion_.y() == 0 && quaternion_.z() == 0){
        // no rotation needed.
        return;
    }
    // first check if we need to rotate
    Eigen::Affine3f rot(quaternion_.conjugate().matrix());
    pcl::transformPointCloud (*cloud, *cloud, rot);

}
template <typename PointT>
bool QuadTreePCL<PointT>::makePolygonSimple(Polygon &polygon, std::vector<Polygon> &polygons, float distance){


    // Idea:
    // find the most left/right/top/bottom vertex and start from there, then we are sure that
    // we start with the external polygon.

    float dist_threshold = distance*distance;

    // Returns true if the distance between a and b is less than dist_threshold.
    // return |b-a| < dist_threshold.
    auto squared_point_distance = [](Point a, Point b, float dist_threshold){
        return std::pow((b[0]-a[0]),2) + std::pow((b[1]-a[1]),2) < dist_threshold;
    };

    int start = 0;
    int stop = polygon.size();
    bool distance_greater = false;

    while(start != stop){
        Polygon poly;
        distance_greater = false;
        for(int i = start; i < stop; ++i){
            std::cout << "start: " << start << " stop: " << stop << " i: " << i << std::endl;
            if(i == stop - 1){
                // Have traveled the entire polygon. STOP
                poly.push_back(polygon[i]);
                polygons.push_back(poly);
                start = stop;
                break;
            }
            bool close_to_origin = squared_point_distance(polygon[i], polygon[start], dist_threshold);

            // Need to check if we have actually moved further than dist_threshold away from origin.
            if( !distance_greater && !close_to_origin ){
                // We have not moved far enough from origin before this check.
                // The distance to origin is greater than dist_threshold.
                distance_greater = true;
                poly.push_back(polygon[i]);
            }

            // Check if point is very close to the starting point.
            else if(distance_greater && close_to_origin ){
                // Possible that we have reached a hole
                poly.push_back(polygon[i]);
                // Check if the next point in polygon (i+1) is inside poly
                // Does not work properly, needs better logic.
                // if(poly.has_on_bounded_side(polygon[i+1])){
                //     // The next point is inside poly.
                //     // i+1 is the start of a hole
                //     polygons.push_back(poly);
                //     start = i+1;
                //     break;
                // }

                // Simple distance check.
                if( !squared_point_distance(polygon[i], polygon[i+1], dist_threshold) ){
                    // Points to far away from each other, was in a hole, probably in external now.
                    polygons.push_back(poly);
                    start = i+1;
                    break;
                }

            } else {
                // Just a normal point push it back.
                poly.push_back(polygon[i]);
            }
        }
    }

    // Need to loop over polygons and find polygon with max size and make that external.
    if(polygons.size() > 0){
        float max = 0;
        int max_idx = 0;
        for(int i = 0; i < polygons.size(); ++i){
            float tmp_max = std::abs(polygons[i].area());
            if( tmp_max > max ){
                max_idx = 0;
                max = tmp_max;
            }
        }
        if(max_idx != 0){
            std::cout << "swapping" << std::endl;
            std::swap(polygons[0], polygons[max_idx]);
        }
    }

    // Very stupid test based on distance.
    // int last_idx = 0;
    // for(size_t i = 0; i < polygon.size() - 1; ++i){
    //     if( !squared_point_distance(polygon[i], polygon[i+1]) ){
    //         std::cout << "distance to great i: " << i << std::endl;
    //         std::cout << "point a: " << polygon[i] << std::endl;
    //         std::cout << "point b: " << polygon[i+1] << std::endl;
    //         if(squared_point_distance(polygon[i], polygon[last_idx])){
    //             // we have closed the polygon
    //             std::cout << "close to last" << std::endl;
    //             Polygon polygon_tmp;
    //             for(size_t j = last_idx; j < i+1; ++j){
    //                 polygon_tmp.push_back(polygon[j]);
    //             }
    //             polygons.push_back(polygon_tmp);
    //             Polygon hole_tmp;
    //             for(size_t j = i+1; j < polygon.size(); ++j){
    //                 hole_tmp.push_back(polygon[j]);
    //             }
    //             polygons.push_back(hole_tmp);
    //         }
    //     }
    // }
    // if(polygons.size() == 0){
    //     polygons.push_back(polygon);
    // }
}


template <typename PointT>
template <typename T>
void QuadTreePCL<PointT>::createTexture(
        const typename pcl::PointCloud<T>::Ptr texture_cloud,
        const typename pcl::PointCloud<T>::Ptr triangle_cloud,
        cv::Mat &image,
        std::vector<Eigen::Vector2f> &vertex_texture){

    float r = 10;
    typename pcl::PointCloud<T>::Ptr texture_tmp (new pcl::PointCloud<T>);
    typename pcl::PointCloud<T>::Ptr triangle_tmp (new pcl::PointCloud<T>);
    rotateToAxis(texture_cloud, texture_tmp);
    rotateToAxis(triangle_cloud, triangle_tmp);

    image = cv::Mat::zeros(r*quad.width(), r*quad.width(), CV_8UC3); // RGB image

    // for(int i  = 0; i < image.rows; ++i){
    //     for(int j  = 0; j < image.cols; ++j){
    //         image.at<cv::Vec3b>(i, j)[0] = 255;
    //         image.at<cv::Vec3b>(i, j)[1] = 0;
    //         image.at<cv::Vec3b>(i, j)[2] = 255;
    //     }
    // }

    std::cout << "image, rows: " << image.rows <<", cols: " << image.cols << std::endl;
    std::cout << "quad width: " << quad.width() << std::endl;

    for(auto p : texture_tmp->points){
        // vertex_texture[i][0] = (p.x - quad.x())/quad.width();
        // vertex_texture[i][1] = (p.y - quad.y())/quad.width();
        int x = std::floor( (p.x - quad.x())/quad.width() * image.cols);
        int y = std::floor( (p.y - quad.y())/quad.width() * image.rows);
        // std::cout << "x: " << x << ", y: " << y << std::endl;
        image.at<cv::Vec3b>(image.rows - y - 1, x)[0] = p.b;
        image.at<cv::Vec3b>(image.rows - y - 1, x)[1] = p.g;
        image.at<cv::Vec3b>(image.rows - y - 1, x)[2] = p.r;
    }

    vertex_texture.resize(triangle_tmp->size());
    for(int i = 0; i < triangle_tmp->size(); ++i){
        vertex_texture[i][0] = (triangle_tmp->at(i).x - quad.x())/quad.width();
        vertex_texture[i][1] = (triangle_tmp->at(i).y - quad.y())/quad.width();
    }

}
}

#include <pcl/common/transforms.h>
#include <algorithm>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <random>

namespace QTD{

template <typename PointT>
void QuadTreePCL<PointT>::createBoundary(typename pcl::PointCloud<PointT>::Ptr boundary){
    std::default_random_engine generator;
    std::normal_distribution<double> distribution(0.0,0.01);

    std::vector<quadPoint> points;
    quad.extractBoundaryPoints(points);
    for(auto p : points){
        PointT pc;
        pc.x = p.x;
        pc.y = p.y;
        pc.z = z_ + distribution(generator);
        boundary->push_back(pc);
    }
}

template <typename PointT>
void QuadTreePCL<PointT>::insertBoundary(typename pcl::PointCloud<PointT>::Ptr boundary){
    // Convert to cgal polygon
    if(boundary->size() == 0) return;

    QuadTreePCL<PointT>::rotateToAxis(boundary);
    PointT min, max;
    pcl::getMinMax3D(*boundary, min, max);

    std::vector<std::vector<int> > boundaryCells;

    if(!inserted_){
        // Determin the inital size of the quadtree
        z_ = boundary->points[0].z;
        float x = QuadTreePCL<PointT>::roundDown(min.x);
        float y = QuadTreePCL<PointT>::roundDown(min.y);
        float width = std::max(QuadTreePCL<PointT>::roundUp(max.x - x), QuadTreePCL<PointT>::roundUp(max.y - y));

        // initialize the quadtree
        quad = QuadTree(1,std::ceil(std::max(max.x-std::floor(min.x), max.y-std::floor(min.y))),std::floor(min.x),std::floor(min.y));
        // quad = QuadTree(1,5,0,0);
        quad.setMaxWidth(max_width_);
        // boundaryCells.resize(std::ceil(5.0/0.1));
        inserted_ = true;
    }


    std::vector<int> holeIdx;
    // QuadTreePCL<PointT>::holeCheck(boundary, holeIdx, 0.2);
    std::vector<typename pcl::PointCloud<PointT>::Ptr> newBoundary;
    QuadTreePCL<PointT>::holeCheckNew(boundary, newBoundary, 0.4);

    if(newBoundary.size() == 0){
        std::cout << "GADDAMMIT" << std::endl;
    }

    std::vector<QTD::quadPoint> qtd_boundary;
    std::vector<int> polygonstartIdx;
    for(auto &cloud : newBoundary){
        if(cloud->size() < 100) continue;
        polygonstartIdx.push_back( qtd_boundary.size() );
        qtd_boundary.reserve(qtd_boundary.size() + cloud->size());
        for(auto p : cloud->points){
            qtd_boundary.push_back(QTD::quadPoint(p.x, p.y));
        }
    }

    if(polygonstartIdx.size() > 0){
        auto startNew = polygonstartIdx.begin();
        startNew++;
        for(size_t i = 0; i < qtd_boundary.size()-1; ++i){

            if(i+1 != *startNew)
                quad.insertSegment( qtd_boundary[i], qtd_boundary[i+1] );
            else{
                if(*startNew != polygonstartIdx.back())
                    startNew++;
            }
        }

        QTD::quadPoint qtd_min(min.x, min.y);
        QTD::quadPoint qtd_max(max.x, max.y);
        polygonstartIdx.push_back(qtd_boundary.size());
        quad.markAsExternal(qtd_boundary, polygonstartIdx, qtd_min, qtd_max);
    }
    QuadTreePCL<PointT>::rotateFromAxis<PointT>(boundary);

}

template <typename PointT>
bool QuadTreePCL<PointT>::combineSegments(const typename pcl::PointCloud<PointT>::Ptr a, const typename pcl::PointCloud<PointT>::Ptr b, typename pcl::PointCloud<PointT>::Ptr out, float dist){

    if(out->size() != 0)  out->clear();

    float distance_threshold = dist*dist;

    auto squared_point_distance = [distance_threshold](PointT a, PointT b){
        return std::pow((b.x-a.x),2) + std::pow((b.y-a.y),2) + std::pow((b.z-a.z),2) < distance_threshold;
    };

    // Need to check if tmp_segment connects to new segment
    if(squared_point_distance( a->back(), b->back())){
        // Need to reverse the new boundary and add to segment.
        std::reverse(b->begin(), b->end());
        *out += *a;
        *out += *b;
        return true;

    } else if( squared_point_distance( a->front(), b->back()) ){
        std::reverse(a->begin(), a->end());
        *out += *b;
        *out += *a;
        return true;

    }
    else if( squared_point_distance( a->front(), b->front()) ){
        std::reverse(a->begin(), a->end());
        *out += *a;
        *out += *b;
        return true;
    }
    else if( squared_point_distance( a->back(), b->front()) ){
        *out += *a;
        *out += *b;
        return true;
    }

    else return false;
}


template <typename PointT>
bool QuadTreePCL<PointT>::holeCheckNew(const typename pcl::PointCloud<PointT>::Ptr boundary, std::vector<typename pcl::PointCloud<PointT>::Ptr> &newBoundary, float dist){
    // Do a very simple distance check and return indices to first point after split.

        float distance_threshold = dist*dist;

        // Returns true if the distance between a and b is less than dist_threshold.
        // return |b-a| < dist_threshold.
        auto squared_point_distance = [distance_threshold](PointT a, PointT b){
            return std::pow((b.x-a.x),2) + std::pow((b.y-a.y),2) + std::pow((b.z-a.z),2) < distance_threshold;
        };

        auto squared_point_distance_value = [distance_threshold](PointT a, PointT b){
            return std::sqrt(std::pow((b.x-a.x),2) + std::pow((b.y-a.y),2) + std::pow((b.z-a.z),2));
        };

        int start = 0;
        bool has_left_origin = false;

        // pcl::PointCloud<PointT>::Ptr tmp (new pcl::PointCloud<PointT>());
        newBoundary.push_back(typename pcl::PointCloud<PointT>::Ptr (new pcl::PointCloud<PointT>()));
        typename pcl::PointCloud<PointT>::Ptr tmp_segment (new pcl::PointCloud<PointT>());
        for(size_t i = 0; i < boundary->size(); ++i){

            if( i == boundary->size() - 1){
                // Need to close last boundary.

                newBoundary.back()->push_back(boundary->at(i));

                if(squared_point_distance(newBoundary.back()->front(), newBoundary.back()->back())){
                    newBoundary.back()->push_back(newBoundary.back()->points.front());
                    if(tmp_segment->size() != 0){
                        // close it as well
                        newBoundary.push_back(tmp_segment);
                        newBoundary.back()->push_back(newBoundary.back()->front());
                    }
                }


                else if(tmp_segment->size() != 0){
                    typename pcl::PointCloud<PointT>::Ptr tmp_combined (new pcl::PointCloud<PointT>());


                    if(combineSegments(newBoundary.back(), tmp_segment, tmp_combined, dist)){
                        *newBoundary.back() = *tmp_combined;
                        return true;
                    } else {
                        newBoundary.resize(1);
                        *newBoundary[0] = *boundary;
                        newBoundary.push_back(tmp_segment);
                        return false;
                    }
                }
                // Check if this newest segment is a polygon (should be)
                // else if(squared_point_distance(newBoundary.back()->front(), newBoundary.back()->back())){
                //     newBoundary.back()->push_back(newBoundary.back()->points.front());
                // }
                else {
                    // remove this segment from the dataset
                    newBoundary.resize(newBoundary.size()-1);
                }

                // if(tmp_segment->size() != 0){
                //     typename pcl::PointCloud<PointT>::Ptr tmp_combined (new pcl::PointCloud<PointT>());
                //     if(combineSegments(newBoundary.back(), tmp_segment, tmp_combined, dist)){
                //         *newBoundary.back() = *tmp_combined;
                //     } else {
                //         std::cout << "Strange ASS boundary, can't detect holes, hoping for the best !!!" << std::endl;
                //         newBoundary.resize(1);
                //         *newBoundary[0] = *boundary;
                //         newBoundary.push_back(tmp_segment);
                //         return false;
                //     }
                // }
                //
                // else if(squared_point_distance(newBoundary.back()->front(), newBoundary.back()->back())){
                //     newBoundary.back()->push_back(newBoundary.back()->points.front());
                // } else {
                //     // remove this segment from the dataset
                //     newBoundary.resize(newBoundary.size()-1);
                // }

                return true;
            }

            else if( newBoundary.back()->size() == 0 ){
                newBoundary.back()->push_back(boundary->at(i));
            }

            else {
                // Check if distance between current and next is to large
                if( !squared_point_distance(boundary->at(i), boundary->at(i+1)) ){
                    // Step to big, still need to check if circle has been closed => close to starting point
                    if( squared_point_distance(boundary->at(i), newBoundary.back()->points.front()) ){
                        // Can close circle.
                        newBoundary.back()->push_back(boundary->at(i));
                        newBoundary.back()->push_back(newBoundary.back()->points.front());
                        start = i+1;
                        newBoundary.push_back(typename pcl::PointCloud<PointT>::Ptr (new pcl::PointCloud<PointT>()));
                        // newBoundary.back()->clear();

                        // has_left_origin = false;
                    } else if( squared_point_distance(boundary->at(i+1), newBoundary.back()->points.front()) ){
                        // have a stupid ordering of points
                        newBoundary.back()->push_back(boundary->at(i));
                        std::reverse(newBoundary.back()->points.begin(), newBoundary.back()->points.end());
                    } else {
                        newBoundary.back()->push_back(boundary->at(i));
                        if(tmp_segment->size() == 0){
                            *tmp_segment = *newBoundary.back();
                            newBoundary.back()->clear();
                        } else {
                            // Need to check if tmp_segment connects to new segment
                            typename pcl::PointCloud<PointT>::Ptr tmp_combined (new pcl::PointCloud<PointT>());
                            if(combineSegments(newBoundary.back(), tmp_segment, tmp_combined, dist)){
                                *tmp_segment = *tmp_combined;
                                newBoundary.back()->clear();

                                // Check if we have closed boundary
                                if( squared_point_distance(tmp_segment->front(), tmp_segment->back()) ){
                                    // have closed boundary
                                    *newBoundary.back() = *tmp_segment;
                                    newBoundary.back()->push_back(newBoundary.back()->front());
                                    newBoundary.push_back(typename pcl::PointCloud<PointT>::Ptr (new pcl::PointCloud<PointT>()));
                                    tmp_segment->clear();
                                }
                            } else {
                                std::cout << "Strange ASS boundary, can't detect holes, hoping for the best !!!" << std::endl;
                                newBoundary.resize(1);
                                *newBoundary[0] = *boundary;
                                return false;
                            }
                        }
                    }
                } else {
                    newBoundary.back()->push_back(boundary->at(i));
                }

            }
        }
}



template <typename PointT>
void QuadTreePCL<PointT>::holeCheck(const typename pcl::PointCloud<PointT>::Ptr boundary, std::vector<int> &splits, float dist){
    // Do a very simple distance check and return indices to first point after split.

    float distance_threshold = dist*dist;

    // Returns true if the distance between a and b is less than dist_threshold.
    // return |b-a| < dist_threshold.
    auto squared_point_distance = [distance_threshold](PointT a, PointT b){
        return std::pow((b.x-a.x),2) + std::pow((b.y-a.y),2) < distance_threshold;
    };

    auto squared_point_distance_value = [distance_threshold](PointT a, PointT b){
        return std::sqrt(std::pow((b.x-a.x),2) + std::pow((b.y-a.y),2));
    };

    int start = 0;
    bool has_left_origin = false;
    for(size_t i = 0; i < boundary->size()-1; ++i){

        if( i == start ) continue;

        else if(!has_left_origin){
            if( !squared_point_distance(boundary->at(start), boundary->at(i)) ){
                has_left_origin = true;
            }
        } else {
            // Check if distance between current and next is to large
            if( !squared_point_distance(boundary->at(i), boundary->at(i+1)) ){
                // Step to big, still need to check if circle has been closed => close to starting point
                if( squared_point_distance(boundary->at(i), boundary->at(start)) ){
                    // Can close circle.
                    splits.push_back(i+1);
                    start = i+1;
                    has_left_origin = false;
                } else {
                    std::cout << "BIG ASS DISTANCE, WHAT TO DO??? " << std::endl;
                    std::cout << "origin: " <<  squared_point_distance_value(boundary->at(i), boundary->at(start)) << std::endl;
                    std::cout << "next: " <<  squared_point_distance_value(boundary->at(i), boundary->at(i+1)) << std::endl;
                    std::cout << "first last: " <<  squared_point_distance_value(boundary->at(start), boundary->at(boundary->size()-1)) << std::endl;
                    std::cout << "size: " << splits.size() << std::endl;
                    splits.push_back(i+1);
                    start = i+1;
                    has_left_origin = false;
                }
            }
        }
    }
    splits.push_back(boundary->size());
}

template <typename PointT>
template <typename T>
void QuadTreePCL<PointT>::createMeshNew(typename pcl::PointCloud<T>::Ptr cloud, std::vector< pcl::Vertices > &vertices){


    int pointCount = cloud->size();

    std::vector<std::vector<int>> vert;
    std::vector<quadPoint> points;
    std::cout << "createMeshNew start" << std::endl;
    quad.extractTriangles(points, vert);
    std::cout << "createMeshNew end" << std::endl;


    T p;
    p.z = z_;
    for(auto point : points){
        p.x = point.x;
        p.y = point.y;
        cloud->push_back(p);
    }

    pcl::Vertices v;
    v.vertices.resize(3);
    for(auto i : vert){
        v.vertices[0] = pointCount + i[0];
        v.vertices[1] = pointCount + i[1];
        v.vertices[2] = pointCount + i[2];
        vertices.push_back(v);
    }

    QuadTreePCL<PointT>::rotateFromAxis<T>(cloud);

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
    QuadTreePCL<PointT>::rotateFromAxis<T>(cloud);
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
template <typename T>
void QuadTreePCL<PointT>::rotateFromAxis(typename pcl::PointCloud<T>::Ptr cloud){

    if(quaternion_.x() == 0 &&quaternion_.y() == 0 && quaternion_.z() == 0){
        // no rotation needed.
        return;
    }
    // first check if we need to rotate
    Eigen::Affine3f rot(quaternion_.conjugate().matrix());
    pcl::transformPointCloud (*cloud, *cloud, rot);

}


template <typename PointT>
template <typename T>
void QuadTreePCL<PointT>::createTexture(
        const typename pcl::PointCloud<T>::Ptr texture_cloud,
        const typename pcl::PointCloud<T>::Ptr triangle_cloud,
        cv::Mat &image,
        std::vector<Eigen::Vector2f> &vertex_texture){


    float r = 100;
    typename pcl::PointCloud<T>::Ptr texture_tmp (new pcl::PointCloud<T>);
    typename pcl::PointCloud<T>::Ptr triangle_tmp (new pcl::PointCloud<T>);
    rotateToAxis(texture_cloud, texture_tmp);
    rotateToAxis(triangle_cloud, triangle_tmp);

    image = cv::Mat::zeros(r*quad.width(), r*quad.width(), CV_8UC3);
    cv::Mat image_tmp = cv::Mat::zeros(r*quad.width(), r*quad.width(), CV_16UC3); // RGB image
    cv::Mat mask = cv::Mat::zeros(image_tmp.rows,image_tmp.cols,CV_8UC1);
    cv::Mat average = cv::Mat::ones(image_tmp.rows,image_tmp.cols,CV_8UC1);


    for(auto p : texture_tmp->points){
        int x = std::floor( (p.x - quad.x())/quad.width() * image_tmp.cols);
        int y = std::floor( (p.y - quad.y())/quad.width() * image_tmp.rows);

        image_tmp.at<cv::Vec3s>(image_tmp.rows - y - 1, x)[0] += p.b;
        image_tmp.at<cv::Vec3s>(image_tmp.rows - y - 1, x)[1] += p.g;
        image_tmp.at<cv::Vec3s>(image_tmp.rows - y - 1, x)[2] += p.r;
        average.at<uchar>(image_tmp.rows - y - 1, x) += 1;
        mask.at<uchar>(image_tmp.rows - y - 1, x) = 1;

    }
    // for(size_t rows = 0; rows < image_tmp.rows; ++rows){
    //     for(size_t cols = 0; cols < image_tmp.cols; ++cols){
    //         if( (int)(average.at<uchar>(rows,cols)) > 1 ){
    //             image_tmp.at<cv::Vec3s>(rows,cols)[0] = image_tmp.at<cv::Vec3s>(rows,cols)[0] / average.at<uchar>(rows,cols);
    //             image_tmp.at<cv::Vec3s>(rows,cols)[1] = image_tmp.at<cv::Vec3s>(rows,cols)[1] / average.at<uchar>(rows,cols);
    //             image_tmp.at<cv::Vec3s>(rows,cols)[2] = image_tmp.at<cv::Vec3s>(rows,cols)[2] / average.at<uchar>(rows,cols);
    //         }
    //     }
    // }
    //
    // cv::Mat kernel = cv::Mat::ones(10,10, CV_8UC1);
    // cv::Mat mask_blur = cv::Mat::zeros(image_tmp.rows,image_tmp.cols,CV_8UC1);
    // cv::Mat image_blur;
    // filter2D( mask, mask_blur, -1, kernel, cv::Point(-1,-1), 0, cv::BORDER_DEFAULT );
    // filter2D( image_tmp, image_blur, -1, kernel, cv::Point(-1,-1), 0, cv::BORDER_DEFAULT );
    //
    // cv::Mat kernel2 = cv::Mat::ones(30,30, CV_8UC1);
    // cv::Mat mask_blur2;
    // cv::Mat image_blur2;
    //
    // filter2D( mask, mask_blur2, -1, kernel2, cv::Point(-1,-1), 0, cv::BORDER_DEFAULT );
    // filter2D( image_tmp, image_blur2, -1, kernel2, cv::Point(-1,-1), 0, cv::BORDER_DEFAULT );
    //
    // for(size_t rows = 0; rows < image_tmp.rows; ++rows){
    //     for(size_t cols = 0; cols < image_tmp.cols; ++cols){
    //         if((int)(mask.at<uchar>(rows,cols)) > 0){
    //             image.at<cv::Vec3b>(rows,cols)[0] = image_tmp.at<cv::Vec3s>(rows,cols)[0];
    //             image.at<cv::Vec3b>(rows,cols)[1] = image_tmp.at<cv::Vec3s>(rows,cols)[1];
    //             image.at<cv::Vec3b>(rows,cols)[2] = image_tmp.at<cv::Vec3s>(rows,cols)[2];
    //         }
    //         else if((int)(mask_blur.at<uchar>(rows,cols)) > 0){
    //             image.at<cv::Vec3b>(rows,cols)[0] = image_blur.at<cv::Vec3s>(rows,cols)[0] / mask_blur.at<uchar>(rows,cols);
    //             image.at<cv::Vec3b>(rows,cols)[1] = image_blur.at<cv::Vec3s>(rows,cols)[1] / mask_blur.at<uchar>(rows,cols);
    //             image.at<cv::Vec3b>(rows,cols)[2] = image_blur.at<cv::Vec3s>(rows,cols)[2] / mask_blur.at<uchar>(rows,cols);
    //         }
    //         else if((int)(mask_blur2.at<uchar>(rows,cols)) > 0){
    //             image.at<cv::Vec3b>(rows,cols)[0] = image_blur2.at<cv::Vec3s>(rows,cols)[0] / mask_blur2.at<uchar>(rows,cols);
    //             image.at<cv::Vec3b>(rows,cols)[1] = image_blur2.at<cv::Vec3s>(rows,cols)[1] / mask_blur2.at<uchar>(rows,cols);
    //             image.at<cv::Vec3b>(rows,cols)[2] = image_blur2.at<cv::Vec3s>(rows,cols)[2] / mask_blur2.at<uchar>(rows,cols);
    //         }
    //     }
    // }

    pcl::KdTreeFLANN<T> kdtree;
    kdtree.setInputCloud(texture_tmp);
    T p;
    p.z = texture_tmp->at(0).z;
    for(size_t rows = 0; rows < image.rows; ++rows){
        for(size_t cols = 0; cols < image.cols; ++cols){
            // p.x = quad.x() + cols / r + 0.5/r;
            p.x = cols*quad.width()/image.cols + quad.x() + 0.5/r;
            p.y = quad.y()+quad.width() - rows*quad.width()/image.rows - 0.5/r;

            if(image.at<cv::Vec3b>(rows,cols)[0] != 0 || image.at<cv::Vec3b>(rows,cols)[1] != 0 || image.at<cv::Vec3b>(rows,cols)[2] != 0) continue;

            std::vector<int> pointIdxNKNSearch(1);
            std::vector<float> pointNKNSquaredDistance(1);
            if ( kdtree.nearestKSearch (p, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 ){
                image.at<cv::Vec3b>(rows,cols)[0] = texture_tmp->at(pointIdxNKNSearch[0]).b;
                image.at<cv::Vec3b>(rows,cols)[1] = texture_tmp->at(pointIdxNKNSearch[0]).g;
                image.at<cv::Vec3b>(rows,cols)[2] = texture_tmp->at(pointIdxNKNSearch[0]).r;
            }
        }
    }


    vertex_texture.resize(triangle_tmp->size());
    for(int i = 0; i < triangle_tmp->size(); ++i){
        vertex_texture[i][0] = (triangle_tmp->at(i).x - quad.x())/quad.width();
        vertex_texture[i][1] = (triangle_tmp->at(i).y - quad.y())/quad.width();
    }

}
}

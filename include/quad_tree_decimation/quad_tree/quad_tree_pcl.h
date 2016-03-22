#ifndef QUADTREEPCL_H
#define QUADTREEPCL_H

#include <Eigen/Dense>
#include <quad_tree_decimation/quad_tree/quad_tree.h>
#include <opencv2/opencv.hpp>
#include <pcl/ModelCoefficients.h>

namespace QTD{

template <typename PointT>
class QuadTreePCL{
    typedef QuadTreePCL qtpcl;

    bool normalVectorSet;
    bool inserted_;
    QuadTree quad;
    Eigen::Vector3f normal_;
    Eigen::Quaternion<float> quaternion_;
    float z_;
public:

    QuadTreePCL(int level, float width, float x, float y){
                normalVectorSet = false;
                inserted_ = false;
    }
    QuadTreePCL()
        :  QuadTreePCL(1,0,0,0){}
    ~QuadTreePCL(){}

    void insertBoundary(typename pcl::PointCloud<PointT>::Ptr boundary);
    void createBoundary(typename pcl::PointCloud<PointT>::Ptr boundary);

    template <typename T>
    void createMesh(typename pcl::PointCloud<T>::Ptr cloud, std::vector< pcl::Vertices > &vertices);
    template <typename T>
    void createMeshNew(typename pcl::PointCloud<T>::Ptr cloud, std::vector< pcl::Vertices > &vertices);


    template <typename T>
    void createTexture(
            const typename pcl::PointCloud<T>::Ptr texture_cloud,
            const typename pcl::PointCloud<T>::Ptr triangle_cloud,
            cv::Mat &image,
            std::vector<Eigen::Vector2f> &vertex_texture);

    void setNormal(Eigen::Vector3f normal);
    void setNormal(pcl::ModelCoefficients::Ptr coeff);
    Eigen::Vector3f getNormal(void){ return normal_; }


    void setMaxLevel(int level){ quad.setMaxLevel(level); }
    void setMaxWidth(float width){ quad.setMaxWidth(width); }

private:
    void rotateToAxis(typename pcl::PointCloud<PointT>::Ptr cloud);

    void rotateToAxis(typename pcl::PointCloud<PointT>::Ptr cloud, typename pcl::PointCloud<PointT>::Ptr out);

    template <typename T>
    void rotateFromAxis(typename pcl::PointCloud<T>::Ptr cloud);

    void holeCheck(const typename pcl::PointCloud<PointT>::Ptr boundary, std::vector<int> &splits, float dist);
    bool holeCheckNew(const typename pcl::PointCloud<PointT>::Ptr boundary, std::vector<typename pcl::PointCloud<PointT>::Ptr> &newBoundary, float dist);

    int roundDown(float toRound){
        int tmp = std::abs(std::floor(toRound));
        if(toRound >= 0){
            return tmp - tmp % 10;
        } else {
            return (-10 + tmp % 10) - tmp;
        }
    }

    int roundUp(float toRound){
        int tmp = std::abs(std::ceil(toRound));
        if(toRound >= 0){
            return (10 - tmp % 10) + tmp;
        } else {
            return -tmp + tmp % 10;
        }
    }
};
}
#include "impl/quad_tree_pcl.hpp"
#endif

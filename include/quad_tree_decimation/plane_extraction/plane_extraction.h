#ifndef QTD_PLANEEXTRACTION_H
#define QTD_PLANEEXTRACTION_H


// #include <planeDetectionComparison/utils.h>
#include <quad_tree_decimation/point_types/surfel_type.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/common/common.h>

#include <ransac_primitives/primitive_core.h>
#include <ransac_primitives/plane_primitive.h>
#include <Eigen/Dense>

namespace QTD{

    class planeExtraction{

        typedef pcl::PointXYZRGB PointT;
        typedef pcl::PointCloud<PointT> PointCloudT;
        typedef pcl::Normal PointN;
        typedef pcl::PointCloud<PointN> PointCloudN;
        typedef pcl::ModelCoefficients ModelCoeffT;

        primitive_params params_primitives_;
        bool params_primitives_set_;

    public:
        planeExtraction() : params_primitives_set_(false){}
        ~planeExtraction(){}


        void planeSegmentationEfficientPPR(     const PointCloudT::Ptr              cloud,
                                                PointCloudN::Ptr                    normals,
                                                std::vector<PointCloudT::Ptr>     & plane_vec,
                                                std::vector<ModelCoeffT::Ptr>     & coeff_vec,
                                                PointCloudT::Ptr                    nonPlanar);

        void planeSegmentationEfficient(    const PointCloudT::Ptr              cloud,
                                            PointCloudN::Ptr                    normals,
                                            std::vector<PointCloudT::Ptr>     & plane_vec,
                                            std::vector<ModelCoeffT::Ptr>     & coeff_vec,
                                            PointCloudT::Ptr                    nonPlanar);

        void combinePlanes( const   std::vector<PointCloudT::Ptr>   &planes,
                                    PointCloudT::Ptr                out,
                                    bool                            useColor = false);

        void combinePlanes( const   std::vector<PointCloudT::Ptr>   &planes,
                            const   PointCloudT::Ptr                nonPlanar,
                                    PointCloudT::Ptr                out,
                                    bool                            useColor = false);

        void setPrimitiveParameters( primitive_params params ){
            params_primitives_ = params;
            params_primitives_set_ = true;
        }

    private:

        void checkIfParamsHaveBeenSet();

        void estimateNormals(const PointCloudT::Ptr cloud, PointCloudN::Ptr normals, float rad);


        bool extractPlaneEfficientRANSAC(   const PointCloudT::Ptr  cloud_in,
                                            const PointCloudN::Ptr  normals,
                                            pcl::PointIndices::Ptr  indices,
                                            ModelCoeffT::Ptr        coeff);



        bool runPPRSinglePlane( PointCloudT::Ptr        nonPlanar,
                                PointCloudN::Ptr        normals,
                                ModelCoeffT::Ptr        coeff,
                                PointCloudT::Ptr        plane);


        pcl::PointIndices::Ptr segmentPPR(  pcl::PointCloud<pcl::PointXYZRGB>::Ptr  cloud,
                                            pcl::ModelCoefficients::Ptr             coefficients,
                                            float                                   max_dist);


        // Euclidean Cluster Extraction from PCL.
        // Inputs:
        //  cloud, single PointCloud that we wan't to extract clusters from.
        //  tolerance, maximum distance between points so that they belong to same cluster.
        //  min_size, minimum number of points in a cluster.
        //  max_size, maximum number of points in a cluster.
        // Output:
        //  outvec, vector containing PointClouds where each PointCloud represents a single cluster.
        template<typename PT>
        void ecClustering(  const typename pcl::PointCloud<PT>::Ptr         cloud,
                            const float                                     tolerance,
                            const int                                       min_size,
                            const int                                       max_size,
                            std::vector<typename pcl::PointCloud<PT>::Ptr>  &outvec,
                            std::vector<pcl::PointIndices>                  &cluster_indices);


        template <typename T>
        void extractIndicesAndRemoveFromOriginal(
                typename pcl::PointCloud<T>::Ptr    cloud,
                typename pcl::PointCloud<T>::Ptr    extracted,
                pcl::PointIndices::Ptr              inliers);

        template <typename T>
        void extractIndicesFromOriginal(
                typename pcl::PointCloud<T>::Ptr    cloud,
                typename pcl::PointCloud<T>::Ptr    extracted,
                pcl::PointIndices::Ptr              inliers);

        void extractIndices(
                PointCloudT::Ptr        cloud,
                PointCloudN::Ptr        normals,
                PointCloudT::Ptr        plane,
                pcl::PointIndices::Ptr  inliers);

        template <typename EIG>
        void eigenToModelCefficient(const EIG eig, ModelCoeffT::Ptr coeff);

        template <typename EIG>
        void modelCefficientToEigen(const ModelCoeffT::Ptr coeff, EIG eig);

        void generateRandomColor(   int mix_red,
                                    int mix_green,
                                    int mix_blue,
                                    int & red,
                                    int & green,
                                    int & blue);

    };
}

#include "impl/plane_extraction.hpp"
#endif

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>

namespace QTD{

    template<typename PT>
    void planeExtraction::ecClustering(  const typename pcl::PointCloud<PT>::Ptr         cloud,
                        const float                                     tolerance,
                        const int                                       min_size,
                        const int                                       max_size,
                        std::vector<typename pcl::PointCloud<PT>::Ptr>  &outvec,
                        std::vector<pcl::PointIndices>                  &cluster_indices){

        typename pcl::search::KdTree<PT>::Ptr tree (new pcl::search::KdTree<PT>);
        tree->setInputCloud(cloud);

        // std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<PT> ec;
        ec.setClusterTolerance(tolerance); // 2cm
        ec.setMinClusterSize(min_size);
        ec.setMaxClusterSize(max_size);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud);
        ec.extract(cluster_indices);

        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
        {
            PT p;
            typename pcl::PointCloud<PT>::Ptr tmp_cloud (new pcl::PointCloud<PT>());
            tmp_cloud->points.reserve(it->indices.size());
            for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){
                p = cloud->points[*pit];
                tmp_cloud->points.push_back(p);
            }
            tmp_cloud->width = tmp_cloud->points.size ();
            tmp_cloud->height = 1;
            tmp_cloud->is_dense = true;
            outvec.push_back(tmp_cloud);
        }
    }


    template <typename T>
    void planeExtraction::extractIndicesAndRemoveFromOriginal(typename pcl::PointCloud<T>::Ptr cloud,
                    typename pcl::PointCloud<T>::Ptr extracted,
                    pcl::PointIndices::Ptr inliers){

        typename pcl::ExtractIndices<T> extract;
        extract.setInputCloud (cloud);
        extract.setIndices (inliers);
        extract.setNegative (false);
        extract.filter (*extracted);

        typename pcl::PointCloud<T>::Ptr tmp (new pcl::PointCloud<T>());
        // Remove the planar inliers, extract the rest
        extract.setNegative (true);
        extract.filter (*tmp);
        (*cloud).swap (*tmp);
    }

    template <typename T>
    void planeExtraction::extractIndicesFromOriginal(    typename pcl::PointCloud<T>::Ptr cloud,
                                        typename pcl::PointCloud<T>::Ptr extracted,
                                        pcl::PointIndices::Ptr inliers){

        typename pcl::ExtractIndices<T> extract;
        extract.setInputCloud (cloud);
        extract.setIndices (inliers);
        extract.setNegative (false);
        extract.filter (*extracted);
    }

    template <typename EIG>
    void planeExtraction::eigenToModelCefficient(const EIG eig, ModelCoeffT::Ptr coeff){
        coeff->values.resize(eig.size());
        for (size_t i = 0; i < eig.size(); i++) {
            coeff->values[i] = eig[i];
        }
    }

    template <typename EIG>
    void planeExtraction::modelCefficientToEigen(const ModelCoeffT::Ptr coeff, EIG eig){
        eig.resize(coeff->values.size());
        for (size_t i = 0; i < coeff->values.size(); i++) {
            eig[i] = coeff->values[i];
        }
    }
}

#ifndef QTD_UTILS_H
#define QTD_UTILS_H


#include <quad_tree_decimation/point_types/surfel_type.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>

using NormalT = pcl::Normal;
using NormalCloudT = pcl::PointCloud<NormalT>;
using SurfelT = SurfelType;
using SurfelCloudT = pcl::PointCloud<SurfelT>;

template<typename PointT>
NormalCloudT::Ptr compute_surfel_normals(SurfelCloudT::Ptr& surfel_cloud, typename pcl::PointCloud<PointT>::Ptr segment)
{
    pcl::KdTreeFLANN<SurfelT> kdtree;
    kdtree.setInputCloud(surfel_cloud);
    NormalCloudT::Ptr normals(new NormalCloudT);
    normals->reserve(segment->size());
    for (const PointT& p : segment->points) {
        if (!pcl::isFinite(p)) {
            NormalT crap; crap.normal_x = 0; crap.normal_y = 0; crap.normal_z = 0;
            normals->push_back(crap);
            continue;
        }
        std::vector<int> indices;
        std::vector<float> distances;
        SurfelT s; s.x = p.x; s.y = p.y; s.z = p.z;
        kdtree.nearestKSearchT(s, 1, indices, distances);
        if (distances.empty()) {
            std::cout << "Distances empty, wtf??" << std::endl;
            exit(0);
        }
        SurfelT q = surfel_cloud->at(indices[0]);
        NormalT n; n.normal_x = q.normal_x; n.normal_y = q.normal_y; n.normal_z = q.normal_z;
        normals->push_back(n);
    }
    return normals;
}


#endif

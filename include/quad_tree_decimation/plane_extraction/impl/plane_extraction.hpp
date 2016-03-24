#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/extract_indices.h>

namespace QTD{

    float planeExtraction::normalAngleDifference(const ModelCoeffT::Ptr coeffa, const ModelCoeffT::Ptr coeffb){
        Eigen::Vector3f a;
        a << coeffa->values[0], coeffa->values[1], coeffa->values[2];
        a.normalize();

        Eigen::Vector3f b;
        b << coeffb->values[0], coeffb->values[1], coeffb->values[2];
        b.normalize();

        return std::acos(a.dot(b));
    }

    template <typename Point>
    bool planeExtraction::boundingBoxIntersect(Point mina, Point maxa, Point minb, Point maxb){
        if(
            ( (mina.x <= minb.x && minb.x <= maxa.x) || ( minb.x <= mina.x && mina.x <= maxb.x) ) &&
            ( (mina.y <= minb.y && minb.y <= maxa.y) || ( minb.y <= mina.y && mina.y <= maxb.y) ) &&
            ( (mina.z <= minb.z && minb.z <= maxa.z) || ( minb.z <= mina.z && mina.z <= maxb.z) )
        ){
            return true;
        }
        return false;
    }


    template <typename T>
    void planeExtraction::mergePlanes(std::vector<typename pcl::PointCloud<T>::Ptr> &clouds, std::vector<ModelCoeffT::Ptr> &coeffs){



        for(size_t i = 0; i < clouds.size()-1; ++i){
            int j = i+1;
            while(j < clouds.size()){
            // for(size_t j = i; j < clouds->size(); ++j){
                // Step 1: calculate the angle between the two planes.
                if(planeExtraction::normalAngleDifference(coeffs[i], coeffs[j]) > M_PI/9 ){
                    j++;
                    continue;
                }

                // Step 2: Check if bounding boxes are close together.
                // This is a horrible check since they are not axis aligned
                // TODO: change.

                T mina, maxa, minb, maxb;
                pcl::getMinMax3D(*clouds[i], mina, maxa);
                pcl::getMinMax3D(*clouds[j], minb, maxb);
                if(!planeExtraction::boundingBoxIntersect<T>(mina, maxa, minb, maxb)){
                    j++;
                    continue;
                }

                // Step 3: Check distance between them by checking random points from the plane.
                float plus = 0, minus = 0, minimum = 1;
                for(auto p : clouds[j]->points){
                    float tmp = pcl::pointToPlaneDistanceSigned(p, coeffs[i]->values[0], coeffs[i]->values[1], coeffs[i]->values[2], coeffs[i]->values[3]);
                    if(tmp < 0){
                        if(tmp < minus) minus = tmp;
                    } else {
                        if(tmp > plus) plus = tmp;
                    }
                    if(std::abs(tmp) < minimum) minimum = std::abs(tmp);
                }
                // if( !((minus != 0 && plus != 0) || (minimum < 0.05 && std::max(std::abs(minus), plus) < 0.2))  ){
                if( !((minus != 0 && plus != 0) || minimum < 0.1)  ){
                // if((minus == 0 || plus == 0) && minimum > 0.05){
                    j++;
                    continue;
                }

                // Step 4: If all criterias are fullfilled, merge the planes.
                if(clouds[j]->size() > clouds[i]->size()){
                    *coeffs[i] = *coeffs[j];
                }
                *clouds[i] += *clouds[j];
                clouds.erase(clouds.begin() + j);
                coeffs.erase(coeffs.begin() + j);
            }
        }
    }


    template <typename T>
    void planeExtraction::projectToPlane(std::vector<typename pcl::PointCloud<T>::Ptr> &clouds, const std::vector<ModelCoeffT::Ptr> &coeffs){
        for(size_t i = 0; i < clouds.size(); ++i){
            planeExtraction::projectToPlane<T>(clouds[i], coeffs[i]);
        }
    }

    template <typename T>
    void planeExtraction::projectToPlane(typename pcl::PointCloud<T>::Ptr cloud, const ModelCoeffT::Ptr coeff){
        // Create the projection object
		typename pcl::ProjectInliers<T> proj;
		proj.setModelType(pcl::SACMODEL_PLANE);
	    proj.setInputCloud(cloud);
	    proj.setModelCoefficients(coeff);
	    proj.filter(*cloud);
    }


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

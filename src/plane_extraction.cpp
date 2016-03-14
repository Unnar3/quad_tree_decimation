#include <quad_tree_decimation/plane_extraction/plane_extraction.h>
#include <Refinement/SurfaceRefinement.h>

namespace QTD{

    void planeExtraction::planeSegmentationEfficientPPR(
                const PointCloudT::Ptr              cloud,
                PointCloudN::Ptr                    normals,
                std::vector<PointCloudT::Ptr>     & plane_vec,
                std::vector<ModelCoeffT::Ptr>     & coeff_vec,
                PointCloudT::Ptr                    nonPlanar){


        planeExtraction::checkIfParamsHaveBeenSet();

        *nonPlanar = *cloud;

        if(cloud->points.size() != normals->points.size()){
            std::cout << "Empty normal cloud. Need to estimate the normals !!!" << std::endl;
            estimateNormals(cloud, normals, params_primitives_.normal_neigbourhood);
        }

        int fail_count = 0;
        while(1){

            pcl::PointIndices::Ptr indices (new pcl::PointIndices());
            ModelCoeffT::Ptr coeff (new ModelCoeffT());
            if(extractPlaneEfficientRANSAC(nonPlanar, normals, indices, coeff)){

                PointCloudT::Ptr plane (new PointCloudT());

                if(runPPRSinglePlane(nonPlanar, normals, coeff, plane)){
                    if(plane->points.size() > 250){
                        plane_vec.push_back(plane);
                        coeff_vec.push_back(coeff);
                    } else {
                        *nonPlanar += *plane;
                        fail_count++;
                    }
                } else {
                    std::cout << "DIDN'T WORK" << std::endl;
                    fail_count++;
                }
            } else {
                fail_count++;
            }

            if(fail_count > 5) break;
        }
    }


    void planeExtraction::planeSegmentationEfficient(
            const PointCloudT::Ptr              cloud,
            PointCloudN::Ptr                    normals,
            std::vector<PointCloudT::Ptr>     & plane_vec,
            std::vector<ModelCoeffT::Ptr>     & coeff_vec,
            PointCloudT::Ptr                    nonPlanar){
        // hull_alpha_

        planeExtraction::checkIfParamsHaveBeenSet();

        if(cloud->points.size() != normals->points.size()){
            std::cout << "Empty normal cloud. Need to estimate the normals !!!" << std::endl;
            estimateNormals(cloud, normals, params_primitives_.normal_neigbourhood);
        }


        std::vector<base_primitive*> primitives = { new plane_primitive() };
        primitive_extractor<PointT> extractor(cloud, normals, primitives, params_primitives_, NULL);
        std::vector<base_primitive*> extracted;
        extractor.extract_single_param(false);
        extractor.extract(extracted);


        std::vector<int> ind;
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
        pcl::ExtractIndices<PointT> extract;
        Eigen::VectorXd data;
        for (size_t j = 0; j < extracted.size(); ++j){
            ind = extracted[j]->supporting_inds;

            if(ind.size() < 200) continue;

            inliers->indices.reserve(inliers->indices.size() + ind.size());
            inliers->indices.insert(inliers->indices.end(), ind.begin(), ind.end());

            extracted.at(j)->shape_data(data);
            ModelCoeffT::Ptr coeff (new ModelCoeffT());
            planeExtraction::eigenToModelCefficient(data.segment<4>(0), coeff);
            coeff_vec.push_back(coeff);

            PointCloudT::Ptr test_cloud (new PointCloudT ());
            test_cloud->points.resize(ind.size());
            for (size_t i = 0; i < ind.size(); ++i){
                test_cloud->points[i] = cloud->points[ind[i]];
            }
            plane_vec.push_back(test_cloud);
        }

        extract.setInputCloud (cloud);
        extract.setIndices (inliers);
        extract.setNegative (true);
        extract.filter (*nonPlanar);
    }


    bool planeExtraction::runPPRSinglePlane(
            PointCloudT::Ptr    nonPlanar,
            PointCloudN::Ptr    normals,
            ModelCoeffT::Ptr    coeff,
            PointCloudT::Ptr    plane){



        PointCloudT::Ptr cloud_tmp (new PointCloudT());
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
        inliers = segmentPPR(nonPlanar, coeff, 0.1);
        if(inliers == NULL){
            return false;
        }
        // PointCloudT::Ptr plane (new PointCloudT);
        extractIndices(nonPlanar, normals, plane, inliers);
        return true;

    }


    bool planeExtraction::extractPlaneEfficientRANSAC(
            const PointCloudT::Ptr  cloud_in,
            const PointCloudN::Ptr  normals,
            pcl::PointIndices::Ptr  indices,
            ModelCoeffT::Ptr        coeff){


        // RANSAC
        std::vector<base_primitive*> primitives = { new plane_primitive() };
        primitive_extractor<PointT> extractor(cloud_in, normals, primitives, params_primitives_, NULL);
        std::vector<base_primitive*> extracted;
        extractor.extract_single_param(true);
        extractor.extract(extracted);

        if( extracted.size() > 0){
            indices->indices = extracted.at(0)->supporting_inds;

            Eigen::VectorXd data;
            extracted.at(0)->shape_data(data);
            coeff->values.resize(4);
            coeff->values[0] = data[0];
            coeff->values[1] = data[1];
            coeff->values[2] = data[2];
            coeff->values[3] = data[3];
            return true;
        }
        return false;

    }

    void planeExtraction::estimateNormals(
            const PointCloudT::Ptr  cloud,
            PointCloudN::Ptr        normals,
            float                   rad){

        // Create the normal estimation class, and pass the input dataset to it
        pcl::NormalEstimationOMP<PointT, pcl::Normal> ne;
        ne.setInputCloud(cloud);

        // Create an empty kdtree representation, and pass it to the normal estimation object.
        // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
        typedef pcl::search::KdTree<PointT> kd_tree_type;
        typedef typename kd_tree_type::Ptr kd_tree_type_ptr;
        kd_tree_type_ptr tree(new kd_tree_type());
        //pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB>::Ptr ptr(octree);
        ne.setSearchMethod(tree);

        // Use all neighbors in a sphere of radius normal_radius m
        ne.setRadiusSearch(rad);


        PointT min,max;
        pcl::getMinMax3D(*cloud,min,max);

        ne.setViewPoint (min.x - 5.0, min.y - 5.0, min.z - 5.0);
        // Compute the features
        ne.compute(*normals);
    }


    pcl::PointIndices::Ptr planeExtraction::segmentPPR(
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr  cloud,
            pcl::ModelCoefficients::Ptr             coefficients,
            float                                   max_dist){

		ppr::SurfaceRefinement * refinement;
		refinement = new ppr::SurfaceRefinement();
		refinement->use_colors = true;
		refinement->setDebugg(false);
		refinement->setVisualize(false);
		refinement->setMaxDistance(max_dist);


		pcl::PointIndices::Ptr inl (new pcl::PointIndices);

		int width = cloud->width;
		int height = cloud->height;
		pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
		normals->width = width;
		normals->height = height;
		normals->points.resize(width*height);

		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_normals(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
		pcl::concatenateFields (*cloud, *normals, *cloud_normals);

		ppr::Plane * p = new ppr::Plane();
		p->normal_x = coefficients->values[0];
		p->normal_y = coefficients->values[1];
		p->normal_z = coefficients->values[2];
		float d = -coefficients->values[3];
		p->point_x	= p->normal_x*d;
		p->point_y	= p->normal_y*d;
		p->point_z	= p->normal_z*d;

		// TODO
		// Shouldnt have to check the distances again, already have list of indices.
		bool ** inliers = new bool*[width];
		for(int i = 0; i < width; i++){
			inliers[i] = new bool[height];
			for(int j = 0; j < height; j++){
				// TODO This loop is probably not needed
				inliers[i][j] = true;
			}
		}

		for(int w = 0; w < width; w++){
			for(int h = 0; h < height; h++){
				pcl::PointXYZRGBNormal & point = cloud_normals->points.at(h*cloud->width+w);
				if(!isnan(point.z)){
                    refinement->use_colors = true;
					float d =  fabs(p->distance(point.x,point.y,point.z));
					inliers[w][h] = d < 0.1;
				}else{
					inliers[w][h] = false;
				}
			}
		}

        bool worked = true;
		if(refinement->improve(inliers,cloud_normals,p) == NULL){
            pcl::PointIndices::Ptr tmp;
            return tmp;
        }

		for(int w = 0; w < width; w++){
			for(int h = 0; h < height; h++){
				if(inliers[w][h]){
					inl->indices.push_back(h*width+w);
				}
			}
		}


		// std::cout << "Coefficients" << std::endl;
        // std::cout << coefficients->values[0] << ", " << coefficients->values[1] << ", " << coefficients->values[2] << std::endl;
        // std::cout << p->normal_x << ", " << p->normal_y << ", " << p->normal_z << std::endl;
		coefficients->values[0] = p->normal_x;
		coefficients->values[1] = p->normal_y;
		coefficients->values[2] = p->normal_z;
		coefficients->values[3] = -p->distance(0.0,0.0,0.0);



		delete(refinement);
		delete(p);
		delete(inliers);
		return inl;
	}

    void planeExtraction::extractIndices(PointCloudT::Ptr        cloud,
                        PointCloudN::Ptr        normals,
                        PointCloudT::Ptr        plane,
                        pcl::PointIndices::Ptr  inliers){

        extractIndicesAndRemoveFromOriginal<PointT>(cloud, plane, inliers);
        PointCloudN::Ptr normals_plane (new PointCloudN());
        extractIndicesAndRemoveFromOriginal<PointN>(normals, normals_plane, inliers);

        std::vector<PointCloudT::Ptr> planes;

        std::vector<pcl::PointIndices> cluster_indices;
        ecClustering<PointT>(plane, 0.05, 0, 1000000, planes, cluster_indices);

        if(planes.size() > 1){
            std::cout << "ecClustering" << std::endl;
            auto result = std::max_element(planes.begin(), planes.end(),
                [](PointCloudT::Ptr a,PointCloudT::Ptr b){
                    return a->points.size() < b->points.size();
                }
            );
            int ind = std::distance(planes.begin(), result);
            // *planet = *plane;

            for (size_t i = 0; i < planes.size(); i++) {
                if(i == ind){
                    *plane = *planes[i];
                } else {
                    *cloud += *planes[i];
                    PointCloudN::Ptr normals_tmp (new PointCloudN());
                    pcl::PointIndices::Ptr inliers_tmp (new pcl::PointIndices());
                    inliers_tmp->indices = cluster_indices[i].indices;
                    extractIndicesFromOriginal<PointN>(normals_plane, normals_tmp, inliers_tmp);
                    *normals += *normals_tmp;
                }
            }
        }

    }

    void planeExtraction::generateRandomColor(
            int mix_red,
            int mix_green,
            int mix_blue,
            int & red,
            int & green,
            int & blue){

        red = ((rand() % 255) + mix_red) / 2;
        green = ((rand() % 255) + mix_green) / 2;
        blue = ((rand() % 255) + mix_blue) / 2;

        if(red < 20 && green < 20 && blue < 20){
            red += 30;
            green += 30;
            blue += 30;
        }

    }

    void planeExtraction::combinePlanes(
            const   std::vector<PointCloudT::Ptr>   &planes,
                    PointCloudT::Ptr                out,
                    bool                            useColor){

        // Check if out is empty an clear it if is not
        if( out->points.size() == 0 ) out->clear();

        int red, green, blue;
        for( auto plane : planes ){
            int sizeOut = out->points.size();
            *out += *plane;
            if(useColor){
                planeExtraction::generateRandomColor(137,196,244, red,green,blue);
                for (size_t i = sizeOut; i < out->points.size(); i++) {
                    out->points[i].r = red;
                    out->points[i].g = green;
                    out->points[i].b = blue;
                }
            }
        }
    }

    void planeExtraction::combinePlanes(
            const   std::vector<PointCloudT::Ptr>   &planes,
            const   PointCloudT::Ptr                nonPlanar,
                    PointCloudT::Ptr                out,
                    bool                            useColor){

        // Check if out is empty an clear it if is not
        if( out->points.size() == 0 ) out->clear();

        planeExtraction::combinePlanes(planes, out, useColor);

        int red, green, blue;
        int sizeOut = out->points.size();
        *out += *nonPlanar;
        if(useColor){
            planeExtraction::generateRandomColor(137,196,244, red,green,blue);
            for (size_t i = sizeOut; i < out->points.size(); i++) {
                out->points[i].r = 255;
                out->points[i].g = 255;
                out->points[i].b = 255;
            }
        }
    }

    void planeExtraction::checkIfParamsHaveBeenSet(){
        if(!params_primitives_set_){
            std::cout << " " << std::endl;
            std::cout << "ERROR, PARAMETERS HAVE NOT BEEN SET!!!" << std::endl;
            std::cout << "Need to be set using setPrimitiveParameters" << std::endl;
            std::cout << "EXITING" << std::endl;
            std::cout << " " << std::endl;
            exit(0);
        }
    }


}


#include "provenancevectors_it.h"



ProvenanceVectors_iT::ProvenanceVectors_iT( pcl::PointCloud<pcl::PointXYZ>::Ptr ibs, pcl::PointCloud<pcl::PointXYZ>::Ptr scene_cloud ) {

    this->ibs = ibs;
    this->sceneCloud = scene_cloud;
    
    maxV      = std::numeric_limits<float>::min();
    minV      = std::numeric_limits<float>::max();

}

void ProvenanceVectors_iT::calculateProvenanceVectors(int knnToSmooth) {
    
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtreeSCN;
    std::vector<int>                NNidSC(1);
    std::vector<float>              NNdistSC(1);

    

    this->rawProvenanceVectors.reset(new pcl::PointCloud<pcl::PointNormal>);
    this->smoothedProvenanceVectors.reset(new pcl::PointCloud<pcl::PointNormal>);


    kdtreeSCN.setInputCloud(this->sceneCloud);
    
    float   maxS      = std::numeric_limits<float>::min();
    float   minS      = std::numeric_limits<float>::max();
    float   sum       = 0;
    double  sumSmooth = 0;


    std::vector<int> bad_ids;
    // Try to ingnore "bad vectors" which would be something very large (silly)
    //int somethingSilly=1000; // TODO this must change
    int somethingSilly = 0;

    //std::cout<<"Checking for normals_z greater than "<<somethingSilly<<std::endl;

    for(int i=0; i<this->ibs->size(); i++)
    {

        pcl::PointXYZ ibs_point = this->ibs->at(i);

        int num_neighbours = kdtreeSCN.nearestKSearch(ibs_point,knnToSmooth,NNidSC,NNdistSC);

        if( num_neighbours > 0 )
        {

            Eigen::Vector3f resultant(0,0,0);
            Eigen::Vector3f scaled_v;
            float norm_resultant;
            float norm_scaled_v;

            for(int j=0; j<knnToSmooth; j++)
            {

                pcl::PointXYZ sc;

                Eigen::Vector3f component;
                float norm_component;


                sc             = this->sceneCloud->at( NNidSC.at(j) );
                component      = Eigen::Vector3f( sc.x-ibs_point.x, sc.y-ibs_point.y, sc.z-ibs_point.z );
                norm_component = component.norm();


                if(component[2]>somethingSilly && j==0) {
                    //this means the provenance vector goes in a different direction to the scene POSSIBLE penetration of the IBS
                    bad_ids.push_back(i);
                }
                if(j==0)
                {
                    // first NN is used for provenance vectors
                    pcl::PointNormal pn;

                    pn.x = ibs_point.x; 
                    pn.y = ibs_point.y;
                    pn.z = ibs_point.z;

                    pn.normal_x = component[0];
                    pn.normal_y = component[1];
                    pn.normal_z = component[2];

                    scaled_v      = component;
                    norm_scaled_v = norm_component;

                    if(minV > norm_component)
                        minV = norm_component;
                    if(maxV < norm_component)
                        maxV = norm_component;

                    sum += norm_component;      // it seems that this is unuseful


                    rawProvenanceVectors->push_back(pn);
                }

                resultant += component;
            }

            //"smmother" provenance vector
            scaled_v       = norm_scaled_v * resultant.normalized();
            norm_resultant = resultant.norm();

            pcl::PointNormal pn;
            pn.x = ibs_point.x; 
            pn.y = ibs_point.y;
            pn.z = ibs_point.z;
            pn.normal_x = scaled_v[0];
            pn.normal_y = scaled_v[1];
            pn.normal_z = scaled_v[2];
            smoothedProvenanceVectors->push_back( pn );

            if( minS > norm_resultant )
                minS = norm_resultant;
            if( maxS < norm_resultant )
                maxS = norm_resultant;

            sumSmooth += scaled_v.norm();   //it seems unuseful
        }
    }



    //If there were some "bad" provenance vectors remove them
    if(bad_ids.size()>0)
    {
        pcl::ExtractIndices<pcl::PointNormal> extractN;
        pcl::ExtractIndices<pcl::PointNormal> extractF;
        pcl::ExtractIndices<pcl::PointXYZ> extractP;

        pcl::PointIndices::Ptr outliers (new pcl::PointIndices ());

        outliers->indices = bad_ids;

        // Extract the outliers
        extractN.setInputCloud (smoothedProvenanceVectors);
        extractF.setInputCloud(rawProvenanceVectors);
        extractP.setInputCloud(this->ibs);


        extractN.setIndices (outliers);
        extractF.setIndices(outliers);
        extractP.setIndices(outliers);


        extractN.setNegative (true);
        extractF.setNegative(true);
        extractP.setNegative(true);

        extractN.filter (*smoothedProvenanceVectors);
        extractF.filter (*rawProvenanceVectors);
        extractP.filter (*this->ibs);
    }

}







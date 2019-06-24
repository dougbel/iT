#include "../include/Testing/it_testing.h"


IT_Testing::IT_Testing(std::string path, std::string paremeters_file)
{
    
    if( ! boost::algorithm::ends_with(path, "/"))
        this->path_data = path + "/";
    
    boost::property_tree::ptree parameters;
    boost::property_tree::read_json( this->path_data + paremeters_file, parameters);
    
    sample_percentage =  parameters.get<float>("parameters.Sample")/100;
    log_level         =  parameters.get<int>("parameters.Debug");
    agg_th            =  parameters.get<float>("parameters.VectorDiff")/100;
    pred_t            =  parameters.get<float>("parameters.PredictionT")/100;
        
    
    for( const boost::property_tree::ptree::value_type &to_test :  parameters.get_child("interactions") )
    {   
        std::string affordance_name;
        std::string object_name;
        std::string aff_path;
        
        affordance_name = to_test.second.get<std::string>("affordance_name");  
        object_name     = to_test.second.get<std::string>("object_name");
        aff_path        = this->path_data + Util_iT::getWorkingDirectory( affordance_name, object_name );
        
        std::string json_file_name;
        
        json_file_name = aff_path + affordance_name + "_" + object_name + ".json";
        {
            boost::property_tree::ptree interaction_json;
            boost::property_tree::read_json(json_file_name, interaction_json);
            
            int sampleSize;
            int numOrientations;
            sampleSize      = interaction_json.get<int>("Sample size");
            numOrientations = interaction_json.get<int>("Orientations");
            
            Agglomerator_IT agglo = Agglomerator_IT::loadFiles( aff_path, affordance_name, object_name, sampleSize, numOrientations );
            
            this->interactions.push_back(agglo);
            
            
            std::string object_cloud_filename ;
            object_cloud_filename = aff_path + affordance_name + "_" + object_name + "_object.pcd";
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_object (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::io::loadPCDFile(object_cloud_filename, *cloud_object);
            
            this->object_clouds.push_back(cloud_object);
            
            pcl::PointXYZ minObj;
            pcl::PointXYZ maxObj;
                           
            pcl::getMinMax3D( *cloud_object, minObj, maxObj );
            
            
            object_diags_size.push_back( pcl::L2_Norm(minObj.getArray3fMap(),maxObj.getArray3fMap(),3) );
            
  
        }
    }
    
    
    std::cout << "path_data " << path_data << std::endl;
    std::cout << "sample_percentage " << sample_percentage << std::endl;
    std::cout << "log_level " << log_level << std::endl;
    std::cout << "agg_th " << agg_th << std::endl;    
    std::cout << "pred_t " << pred_t << std::endl;
    for( const boost::property_tree::ptree::value_type &v :  parameters.get_child("interactions") )
        std::cout << "interaction " << v.second.get<std::string>("affordance_name") << ", " << v.second.get<std::string>("object_name") << std::endl; 
    
}



void IT_Testing::testInteractions(pcl::PointCloud<pcl::PointXYZ>::Ptr whole_scene){
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr spinCloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr largeData;
    Eigen::Matrix <float, Eigen::Dynamic, 3, Eigen::RowMajor> largeVectors;
    Eigen::Matrix <float, Eigen::Dynamic, 3, Eigen::RowMajor> largeDescriptor;
    std::vector<int> ppCentroid;
    std::vector<int> startppCentroid;
    std::vector<float> large_mags;        //provenance vector NORM of the point asociated in the sample
    std::vector<float> large_lengths;     //MAPPED provenance vector NORM to [1, 0]
    float alternative_data_counts;        // NOOOOO SÉ, en el codigo original se trata de la suma de mags
    
    spinCloud = this->interactions[0].bare_points;               //_descriptor_8_points.pcd
    largeData = this->interactions[0].useful_cloud;         //_descriptor_8_extra.pcd
    largeVectors = this->interactions[0].largeVectors;
    largeDescriptor = this->interactions[0].descriptor;
    ppCentroid = this->interactions[0].ppCentroid;
    startppCentroid= this->interactions[0].startppCentroid;
    large_mags = this->interactions[0].large_mags;
    large_lengths = this->interactions[0].large_lengths;
    alternative_data_counts = this->interactions[0].alternative_data_counts;
    
    
      
    std::cout<<"Creating octree...";
    octree.reset(new pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>(true));
    octree->setResolution( this->object_diags_size[0] * 2 );                                                    //TODO this is object dependend
    octree->setInputCloud (whole_scene);
    octree->defineBoundingBox();
    octree->addPointsFromInputCloud ();
    std::cout<<"done"<<std::endl;
    
/////////////////APO: SAMPLING
    pcl::PointCloud<pcl::PointXYZ>::Ptr sample_scene;
    pcl::RandomSample<pcl::PointXYZ> random_sampler;      //this applies a random sample with uniform probabiliti
    std::vector<int> randomSampleIdx;
   
    sample_scene.reset(new pcl::PointCloud<pcl::PointXYZ>);
    
    random_sampler.setSample( this->sample_percentage * whole_scene->size() );
    random_sampler.setInputCloud( whole_scene );
    random_sampler.setSeed( unsigned ( std::time(0) ) );
    random_sampler.filter(*sample_scene);
  
    
    //to keep track progress
    int topCount = sample_scene->size();
    int progress = 0;
    
    
////////////////APO: for visualizing
    //PointCloud::Ptr object(new PointCloud);
    //pcl::PolygonMesh::Ptr best_object(new pcl::PolygonMesh);
    
    
/////////////////APO: PREPARE CONTAINERS FOR RESULTS
    pcl::PointCloud<pcl::PointXYZ>::Ptr goodData(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr goodPoints(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr sampledPoints(new pcl::PointCloud<pcl::PointXYZ>);
    
    std::vector<float> best_local_angle(1);     //(myTensor.affordance_name.size());  //TODO it is necesary understand how they perform multiple affordance prediction
    std::vector<float> best_angle(1);           //(myTensor.affordance_name.size());
    
    std::vector<float> best_local_score(1,0);   //(myTensor.affordance_name.size(),0);
    std::vector<float> best_score(1,0);         //(myTensor.affordance_name.size(),0);
    
    std::vector<int> success_counter(1,0);           //myTensor.affordance_name.size(),0);
    
    bool goodPoint=false;
    
    
/////////////////APO: SETUP GPU
    // clock to compute "frame rate" or points tested per second
    std::clock_t ppsecond;

    // Set GPU/cuda stuff 

//     // If more than 1 GPU  you can set it here
    dev_array<float>::setGPU(0);
    // Keypoints: 2048 points x 3 components
    dev_array<float> d_kpData(largeDescriptor.rows()*3);
    d_kpData.set(largeDescriptor.data(),largeDescriptor.rows()*3);
    // Provenance Vectors: 512 vectors x 3 components
    dev_array<float> d_pvData(largeVectors.rows()*3);
    d_pvData.set(largeVectors.data(),largeVectors.rows()*3);
    // Provenance Vectors: 512 vector lengths
    dev_array<float> d_weights(large_lengths.size());
    d_weights.set(&large_lengths[0],large_lengths.size());
    // Provenance Vectors: 512 vector weights
    dev_array<float> d_mags(large_mags.size());
    d_mags.set(&large_mags[0],large_mags.size());
    // Usefull for multiple affordance, for single affordance this
    // is a 2048 array full on 1's
    dev_array<int> d_ppCentroid(largeDescriptor.rows());
    d_ppCentroid.set(&ppCentroid[0],largeDescriptor.rows());
    // Usefull for multiple affordance, for single affordance this
    // is a 2048 with the cumsum of ppCentroid 
    dev_array<int> d_startppCentroid(largeDescriptor.rows());
    d_startppCentroid.set(&startppCentroid[0],largeDescriptor.rows());
    //Times 4 because pcl alignment x,y,z + extra byte
    // This has data for orientation, keypoint and affordance id
    dev_array<float> d_ppCData(largeData->size()*4);
    d_ppCData.set(&largeData->at(0).getArray3fMap()[0],largeData->size()*4);

    // GPU stuff in PCL
    // Container for indices in NN-search. Same size as decscriptor since 1-NN for every
    // keypoint in descriptor
    pcl::gpu::NeighborIndices result_device(largeDescriptor.rows(), 1);
    // Octree for NN search: cloud and structure
    pcl::gpu::Octree::PointCloud cloud_device2;
    pcl::gpu::Octree octree_device2;
    // Container for descriptor keypoints (point queries)
    pcl::gpu::Octree::Queries queries_device;
    
    
/////////////////APO: SEARCH FOR MATCHES
    // For "frame rate"
    pcl::console::TicToc tt;
    ppsecond = std::clock();
    
    int points_tested=0;
    int max_cloud=0;
    // Test point counter
    int i=0;    




    

}

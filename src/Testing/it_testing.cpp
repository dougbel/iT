#include "../include/Testing/it_testing.h"


IT_Testing::IT_Testing(std::string path, std::string paremeters_file)
{
    
    if( ! boost::algorithm::ends_with(path, "/"))
        this->path_data = path + "/";
    
    boost::property_tree::ptree root;
    boost::property_tree::read_json( this->path_data + paremeters_file, root);
    
    sample_percentage =  root.get<float>("parameters.Sample")/100;
    log_level         =  root.get<int>("parameters.Debug");
    agg_th            =  root.get<float>("parameters.VectorDiff")/100;
    pred_t            =  root.get<float>("parameters.PredictionT")/100;
        
    
    for( const boost::property_tree::ptree::value_type &v :  root.get_child("interactions") )
    {   
        std::string affordance_name = v.second.get<std::string>("affordance_name");  
        std::string object_name = v.second.get<std::string>("object_name");
        std::string aff_path = IT::getDirectory( affordance_name, object_name );
        
        aff_path = this->path_data + aff_path;

        std::string json_file_name= aff_path + affordance_name + "_" + object_name + ".json";
        
        boost::property_tree::ptree json_analyzer;
        boost::property_tree::read_json(json_file_name, json_analyzer);
        
        int sampleSize      = json_analyzer.get<int>("Sample size");
        int numOrientations = json_analyzer.get<int>("Orientations");
        
        Agglomerator_IT agglo = Agglomerator_IT::loadFiles( aff_path, affordance_name, object_name, sampleSize, numOrientations );
        
        this->interactions.push_back(agglo);
  
        
        
        
    }
    
    
    std::cout << "path_data " << path_data << std::endl;
    std::cout << "sample_percentage " << sample_percentage << std::endl;
    std::cout << "log_level " << log_level << std::endl;
    std::cout << "agg_th " << agg_th << std::endl;    
    std::cout << "pred_t " << pred_t << std::endl;
    for( const boost::property_tree::ptree::value_type &v :  root.get_child("interactions") )
        std::cout << "interaction " << v.second.get<std::string>("affordance_name") << ", " << v.second.get<std::string>("object_name") << std::endl; 
    
}



void IT_Testing::testInteractions(pcl::PointCloud<pcl::PointXYZ> whole_scene){
    
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
}

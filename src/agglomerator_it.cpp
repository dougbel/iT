#include "agglomerator_it.h"


Agglomerator_IT::Agglomerator_IT(Eigen::Matrix< float, Eigen::Dynamic, 3 , Eigen::RowMajor > vectors, 
                                 Eigen::Matrix< float, Eigen::Dynamic, 3 , Eigen::RowMajor > descriptor,
                                 std::vector<float> pv_norms ,
                                 std::vector<float> pv_norms_mapped 
                                )
{
    this->vectors       = vectors;
    this->descriptor    = descriptor;
    this->pvNorms      = pv_norms;
    this->pvNormsMapped = pv_norms_mapped;
    
    this->sampleSize    = vectors.rows();
    this->n_orientations= descriptor.rows()/vectors.rows();
    
    this->aux_cloud    = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    this->useful_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    this->better_approx= pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    this->bare_points  = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    this->vector_ids_agglomerative = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    this->vectors_data_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    
    // point counts per affordance per orientation
    // Mostly useful for normalization in multiple affordance prediction
    // for single affordance case: 1xn_orientations matrix with sampleSize in each element
    this->data_individual =  Eigen::MatrixXf::Zero(1,n_orientations);
    this->largeVectors.resize( sampleSize*n_orientations, 3 );
}

void Agglomerator_IT::compileAgglomeration()
{
    
    aux_cloud->resize(descriptor.rows());
    useful_cloud->resize(descriptor.rows());
    better_approx->resize(descriptor.rows());
    bare_points->resize(descriptor.rows());
    vector_ids_agglomerative->resize(descriptor.rows());
    vectors_data_cloud->resize(descriptor.rows());
    
    large_mags.reserve(descriptor.rows());
    large_lengths.reserve(descriptor.rows());
    ppCentroid.reserve(descriptor.rows());
    startppCentroid.reserve(descriptor.rows());
    
    for(int i=0;i<descriptor.rows();i++)
    {
        int orientation_id=std::floor(i/sampleSize); // [0-nOrientations) default: 8 orientations
        int smaller_id=i-(sampleSize*orientation_id); // [0-sampleSize)
        
        data_individual(0,orientation_id) += 1;
        
        aux_cloud->at(i).x = 1;
        aux_cloud->at(i).y = i;
        aux_cloud->at(i).z = 0;
        
        useful_cloud->at(i).x = 1;
        useful_cloud->at(i).y = orientation_id;
        useful_cloud->at(i).z = smaller_id;
        
        better_approx->at(i).x = descriptor(i,0);
        better_approx->at(i).y = descriptor(i,1);
        better_approx->at(i).z = descriptor(i,2);
        
        bare_points->at(i).x = descriptor(i,0);
        bare_points->at(i).y = descriptor(i,1);
        bare_points->at(i).z = descriptor(i,2);
        
        vector_ids_agglomerative->at(i).x = vectors(smaller_id,0);
        vector_ids_agglomerative->at(i).y = vectors(smaller_id,1);
        vector_ids_agglomerative->at(i).z = vectors(smaller_id,2);
        
        Eigen::Vector3f aVector = vectors.row(smaller_id);
        vectors_data_cloud->at(i).x = pvNorms.at(smaller_id);
        vectors_data_cloud->at(i).y = pvNormsMapped.at(smaller_id);
        vectors_data_cloud->at(i).z = 0;
                
        largeVectors.row(i) = vectors.row(smaller_id);        // this has vectors repited as many as orientations
        
        large_mags.push_back(vectors_data_cloud->at(i).x);    //provenance vector NORM of the point asociated in the sample
        large_lengths.push_back(vectors_data_cloud->at(i).y); //MAPPED provenance vector NORM to [1, 0]
        
        ppCentroid.push_back(1);                              //this is a vector of 1's  TODO possibly better to change it for and id
        startppCentroid.push_back(i);                         //this is a vector of consecutive numbers TODO maybe it is not necessary
    }
    
    this->sumMags = std::accumulate(pvNormsMapped.begin(),pvNormsMapped.end(), 0.0f);
    this->alternative_data_counts = 1/this->sumMags;                 // NOOOOO SÉ, en el codigo original se trata de la suma de mags
    
    //std::cout << data_individual << std::endl;
}

Agglomerator_IT Agglomerator_IT::loadFiles(std::string path, std::string affordance_name, std::string object_name, int sampleSize, int numOrientations)
{
      
    pcl::PointCloud<pcl::PointXYZ>::Ptr sampledPoints = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr provenanceVectors = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr provenanceVectorsData = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    
    
    if( ! boost::algorithm::ends_with(path, "/"))
        path = path + "/";
    
    
    std::string file_nameU;
    std::string base_nameU;
    base_nameU = path + "UNew_"+ affordance_name + "_" + object_name + "_descriptor_"+std::to_string(numOrientations);
        
    
    file_nameU=base_nameU+"_points.pcd";
    pcl::io::loadPCDFile(file_nameU.c_str(),*sampledPoints);
    
    file_nameU=base_nameU+"_vectors.pcd";
    pcl::io::loadPCDFile(file_nameU.c_str(),*provenanceVectors);
   
    file_nameU = base_nameU + "_vdata.pcd";
    pcl::io::loadPCDFile(file_nameU.c_str(), *provenanceVectorsData);
    

    
    Eigen::Matrix <float, Eigen::Dynamic, 3, Eigen::RowMajor> vectors;
    Eigen::Matrix <float, Eigen::Dynamic, 3, Eigen::RowMajor> descriptor;
    std::vector<float> pv_norms_mappedU;
    std::vector<float> pv_norms_U;
      
    
    vectors.resize( sampleSize, 3 );
    
    pv_norms_U.reserve(sampleSize);
    pv_norms_mappedU.reserve(sampleSize);
    
    
    for(int i=0;i<sampleSize;i++)
    {
        vectors.row(i)=provenanceVectors->at(i).getVector3fMap();
        pv_norms_U.push_back(provenanceVectorsData->at(i).x);
        pv_norms_mappedU.push_back(provenanceVectorsData->at(i).y);
    }
    
    descriptor = sampledPoints->getMatrixXfMap(3,4,0).transpose();
    
    
//     std::cout << "VECTORS:" <<vectors.rows()<< std::endl;
//     std::cout << vectors<< std::endl;    
//     std::cout << "-------------------------------------------------------------------------------------------"<< std::endl<< std::endl<< std::endl;
//     std::cout << "DESCRIPTOR:" <<descriptor.rows()<< std::endl;
//     std::cout << descriptor<< std::endl;    
    
    Agglomerator_IT agglomeratorU(vectors, descriptor, pv_norms_U, pv_norms_mappedU);
    agglomeratorU.compileAgglomeration();
    
    return agglomeratorU;
    
}




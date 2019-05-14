#include "it.h"



IT::IT( pcl::PointCloud<pcl::PointXYZ>::Ptr scene, pcl::PointCloud<pcl::PointXYZ>::Ptr object, std::string name_affordance, std::string name_object){
       
    this->sceneCloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    this->objectCloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    
     // to compute voronoi diagram over all points
    this->sceneCloud   = scene;
    this->objectCloud  = object;
    this->affordanceName = name_affordance;
    this->objectName   = name_object;
    

}


void IT::calculate(){
    
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //calculate the Region of Interest (RoI) for further calculations
    //a sphere with radio equal to the diagonal of the box which surroundsthe object
    pcl::PointXYZ min, max, middlePointObject;
    float radio;
    
    pcl::getMinMax3D(*objectCloud,min,max);
    
    middlePointObject.x=(max.x+min.x)/2;
    middlePointObject.y=(max.y+min.y)/2;
    middlePointObject.z=(max.z+min.z)/2;
    
    radio=pcl::L2_Norm(min.getArray3fMap(),max.getArray3fMap(),3);
    
    
    
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // extract volume of interest from the scene
    pcl::PointCloud<pcl::PointXYZ>::Ptr sceneCloudFiltered;
    
    sceneCloudFiltered = Util::sphereExtraction(sceneCloud, middlePointObject,radio);

    
    //pcl::io::savePCDFile("scene_cloud_filtered.pcd",*sceneCloudFiltered);
    
 StopWatch sw;
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // calculate the Interaction Bisector Surface (IBS)
//     IBS ibs_calculator(sceneCloudFiltered,objectCloud);

// sw.Restart();
//     ibs_calculator.calculate();
// std::cout << "TIMER: ISB calculation " << sw.ElapsedMs() << std::endl;
//     
//     
// sw.Restart();
//     //as IBS extend to infinity, filter IBS
//     pcl::PointCloud<pcl::PointXYZ>::Ptr ibsFiltered;
//     ibsFiltered = Util::sphereExtraction(ibs_calculator.getIBS(), middlePointObject,radio);
    
    
//     //TODO erase this, it is only for developing porpouses
    pcl::PointCloud<pcl::PointXYZ>::Ptr ibsFiltered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("./tmp/ibs_clouds_prefiltered_filtered.pcd", *ibsFiltered);
    
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Searching nearest neighbours (from IBS to ) and calculating smoot provenance vector

    
    pcl::PointCloud<pcl::PointNormal>::Ptr  field;
    pcl::PointCloud<pcl::PointNormal>::Ptr  smoothField;
    
    float maxV, minV, maxS, minS;
    float  sum, sumSmooth;
    
    ProvenanceVectors_iT pv_it(ibsFiltered, sceneCloudFiltered);
    pv_it.calculateProvenanceVectors(5);
    
    field       = pv_it.rawProvenanceVectors;
    smoothField = pv_it.smoothedProvenanceVectors;
    maxV        = pv_it.maxV;
    minV        = pv_it.minV;
    maxS        = pv_it.maxS;
    minS        = pv_it.minS;
    sum         = pv_it.sum;
    sumSmooth   = pv_it.sumSmooth;
    
     //pcl::io::savePCDFile("test_1_pv_calculation_field.pcd",*field);
     //pcl::io::savePCDFile("test_1_pv_calculation_smoothField.pcd",*smoothField);
    
    
    
    //TODO this seems unecesary
    //This is a cleaner tensor/ibs, which does not have those bad prov vectors
    pcl::PointCloud<pcl::PointXYZ>::Ptr copyIBS(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::copyPointCloud(*ibsFiltered,*copyIBS);

std::cout << "TIMER: Filtering " << sw.ElapsedMs() << std::endl;

    // Print out some data about the tensor and filtering/cleaning
    std::cout<<"Tensor after filtering "<<copyIBS->size()<<std::endl;
    std::cout<<"Min: "<<minV<<" Max: "<<maxV<<std::endl;
    std::cout<<"Sum: "<<sum<<std::endl;
    std::cout<<"======================"<<std::endl;
    std::cout<<"MinS: "<<minS<<" MaxS: "<<maxS<<std::endl;
    std::cout<<"SumSmooth: "<<sumSmooth<<std::endl;
    
    
sw.Restart();    
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // it is possible that this section is useless
    
    // Save the original provenance vectors bacause we are going to normalize
    pcl::PointCloud<pcl::PointNormal>::Ptr normals_backup(new pcl::PointCloud<pcl::PointNormal>);
    pcl::copyPointCloud(*field,*normals_backup);
    
    
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Generate weights for sampling of vizualization in provenance vectors   
    
    //Init probabilities for sampling
    std::vector<float> probs = getSamplingProbabilities(field, minV, maxV);
   
    
    // Newer min/max after filtering needs to be recomputed
    float nMin;
    float nMax;
    
    //No mapping
    nMin = minV;
    nMax = maxV;
    
    //BEGINING Mapping norm magnitudes to [0,1]
//     Util_iT::mapMagnitudes( *smoothField, minV, maxV, 1, 0 );
//     Util_iT::mapMagnitudes( *field, minV, maxV, 1, 0 );
//     nMin=0;
//     nMax=1;
    //END Mapping norm magnitudes to [0,1]
    
    //BEGIN Mapping norm magnituddes to [min_out, maxout]
//     float min_out = 1;
//     float max_out = 100;
//     Util_iT::mapMagnitudesRationalFunction( *smoothField, minV, maxV, min_out, max_out );
//     Util_iT::mapMagnitudesRationalFunction( *field, minV, maxV, min_out, max_out  );
//     //this is because the 1/x function
//     nMin=0;
//     nMax=1;
    //END Mapping norm magnituddes to [min_out, maxout]

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Sampling provenance vector to construct the descriptor
  
    //Sample size to take from tensor
    int sampleSize = 512;
    
    Sampler_iT* samplerW = new SamplerWeighted_iT(field, nMin, nMax, sampleSize);
    samplerW->calculateSample();
    pcl::PointCloud<PointWithVector>::Ptr new_sampleCloud2;//sampleW;
    new_sampleCloud2 = samplerW->sample;
    
    
    Sampler_iT* samplerU = new SamplerUniform_iT(field, sampleSize);
    samplerU->calculateSample();
    pcl::PointCloud<PointWithVector>::Ptr new_sampleCloudU; //sampleU;
    new_sampleCloudU = samplerU->sample;
    
    
    //Save mags in sampled mapped in 0-1 based on full tensor mags
    std::vector<float> mags_c(sampleSize);
    std::vector<float> mags_cU(sampleSize);
    for(int i=0;i<sampleSize;i++)
    {
        pcl::PointNormal backup = normals_backup->at( samplerW->getIdxNormKeypoints().at(i).first );
        Eigen::Vector3f backupNormal( backup.normal_x, backup.normal_y, backup.normal_z);
        
        //Save mags in sampled mapped in 0-1 based on full tensor mags
        mags_c.at(i)=Util_iT::getValueProporcionsRule( backupNormal.norm(), nMin, nMax, 1, 0 ); //FIX this code made reference to nMin and nMax but I think is BETTER minV and maxV
        
        //Save mags in sampled mapped in 0-1 based on full tensor mags
        mags_cU.at(i) = Util_iT::getValueProporcionsRule( backupNormal.norm(), nMin, nMax, 1, 0 );//FIX this code made reference to nMin and nMax but I think is BETTER minV and maxV
    }
    
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Saving iT    
    
    // Some file names to save data
    // Some of the data saved in this file is only kept
    // to not break previous code but for most recent
    // version is not used.
    
    //Check a path exists
    std::string aff_path = this->affordanceName + "/";
    if (!boost::filesystem::exists( this->affordanceName.c_str() ) )
    {
        std::cout << "New affordance? Creating dir -> " <<aff_path<< std::endl;
        //std::string command="mkdir "+aff_path;
        //command=exec(command.c_str());
        boost::filesystem::path dir(aff_path);
		boost::filesystem::create_directory(dir);
    }
    else
    {
        std::cout << "Found affordance dir -> " <<aff_path<< std::endl;
    }


    // Some data for info file
    // Closest point in Tensor to scene and in object to scene
    // were used previously to estimate the pose of Tensor/Object
    // relative to scene. These are still computed and save but no longer
    // used, the new pose is computed using center of bounding boxes.
    
    std::cout<<"Getting closest point in Tensor to scene"<<std::endl;
    //TODO resolver como se ajustan estas relaciones, de "PUNTOS DE REFERENCIA"
    pcl::PointXYZ p_scene;
    int idxScene = Util_iT::indexOfClosestPointInACloud ( sceneCloudFiltered, middlePointObject );
    p_scene      = sceneCloudFiltered->at(idxScene);
    int one      = Util_iT::indexOfClosestPointInACloud ( ibsFiltered, p_scene); //"ClosestTensor"
    
    std::cout<<"Getting closest from object to scene"<<std::endl;
    int two  = Util_iT::indexOfClosestPointInACloud ( objectCloud, p_scene );
    int three= Util_iT::indexOfClosestPointInACloud ( sceneCloudFiltered, p_scene ); //"ClosestSCN");
    Eigen::Vector3f toIBS,toObject;
    toIBS[0]=ibsFiltered->at(one).x-p_scene.x;
    toIBS[1]=ibsFiltered->at(one).y-p_scene.y;
    toIBS[2]=ibsFiltered->at(one).z-p_scene.z;
    toObject[0]=objectCloud->at(two).x-p_scene.x;
    toObject[1]=objectCloud->at(two).y-p_scene.y;
    toObject[2]=objectCloud->at(two).z-p_scene.z;

    // Info file name
    {
    std::string file_name= aff_path + "ibs_full_" + this->affordanceName + "_" + this->objectName + ".txt";
    std::ofstream output_file(file_name.c_str());
    int clusters=1;
    float size=0;

    // Previously some clustering was performed over IBS points to estimate
    // clusters, only one (referencing=single) of this clusters was then used to estimate poses but
    // we dropped that. Data is still saved.
    std::string referencing="Single";

    //Start saving to file
    if(output_file.is_open())
    {
        std::string scene_name = "table";
        output_file<<"Scene name:"<<scene_name<<"\n"; //TODO This could be unnecesary
        output_file<<"Object name:"<<this->objectName<<"\n";
        output_file<<"Clusters:"<<clusters<<"\n";
        for(int i=0;i<clusters;i++)
            output_file<<ibsFiltered->at(one).x<<","<<ibsFiltered->at(one).y<<","<<ibsFiltered->at(one).z<<"\n";
        output_file<<"Distance threshold:"<<size<<"\n";
        output_file<<"Reference:"<<referencing<<"\n";
        output_file<<one<<":"<<ibsFiltered->at(one).x<<","<<ibsFiltered->at(one).y<<","<<ibsFiltered->at(one).z<<"\n";
        output_file<<"ScenePoint\n";
        output_file<<three<<":"<<sceneCloudFiltered->at(three).x<<","<<sceneCloudFiltered->at(three).y<<","<<sceneCloudFiltered->at(three).z<<"\n";
        output_file<<"IbsPointVector\n";
        output_file<<one<<":"<<toIBS[0]<<","<<toIBS[1]<<","<<toIBS[2]<<"\n";
        output_file<<"ObjPointVector\n";
        output_file<<two<<":"<<toObject[0]<<","<<toObject[1]<<","<<toObject[2]<<"\n";
        output_file<<"Object Transformation\n";
        //TODO this is unnecesary
//         output_file<<centroidObjFinal[0]-centroidObjInit[0]<<","<<centroidObjFinal[1]-centroidObjInit[1]<<","<<centroidObjFinal[2]-centroidObjInit[2]<<"\n";
//         output_file<<ob_angle<<"\n"; 
//         output_file<<"Object Transformation Box\n";
//         output_file<<box2.x-box1.x<<","<<box2.y-box1.y<<","<<box2.z-box1.z<<"\n";
//         output_file<<"SceneToBoxCentroid\n";
//         output_file<<box2.x-p_scene.x<<","<<box2.y-p_scene.y<<","<<box2.z-p_scene.z<<"\n";
    }
    else
    {
        std::cout<<"Problem with data file "<<std::endl;
    }
    output_file.close();
    }
    
    // Scene-to-IBS and Scene-to-object are saved in affordance keypoints file
    // As commented earlier it was used to align pointclouds
    // at test time. No longer used but still kept in files.
    PointWithVector secondtolast;
    secondtolast.x=toIBS[0];
    secondtolast.y=toIBS[1];
    secondtolast.z=toIBS[2];
    secondtolast.v1=one;
    secondtolast.v2=secondtolast.v3=0;
    
    
    PointWithVector last;
    last.x=toObject[0];
    last.y=toObject[1];
    last.z=toObject[2];
    last.v1=two;
    last.v2=last.v3=0;

    
    new_sampleCloud2->push_back(secondtolast);
    new_sampleCloud2->push_back(last);
    
    new_sampleCloudU->push_back(secondtolast);
    new_sampleCloudU->push_back(last);
    
    
    // Save everything
    std::string new_ibs_field   = aff_path + this->affordanceName + "_" + this->objectName + "_field.pcd";
    std::string new_ibs_sample  = aff_path + "ibs_sample_512_" + this->affordanceName + "_" + this->objectName + "_better.pcd";
    std::string new_ibs_sampleU = aff_path + "ibs_sample_512_" + this->affordanceName + "_" + this->objectName + "_betterUniform.pcd";
    
    std::string smoother_field  = aff_path + this->affordanceName + "_" +this->objectName + "_smoothfield.pcd";
    std::string clean_ibs       = aff_path + "ibs_full_" + this->affordanceName + "_" + this->objectName + "_clean.pcd";  // TODO It doesn't have any sense
    std::string full_ibs        = aff_path + "ibs_full_" + this->affordanceName + "_" + this->objectName + ".pcd";
    
    pcl::io::savePCDFileASCII( new_ibs_field.c_str(), *field);
    pcl::io::savePCDFile( new_ibs_sample.c_str(), *new_sampleCloud2);
    pcl::io::savePCDFile( new_ibs_sampleU.c_str(), *new_sampleCloudU);
    
    pcl::io::savePCDFile( smoother_field.c_str(), *smoothField);
    pcl::io::savePCDFile( clean_ibs, *copyIBS);
    pcl::io::savePCDFile( full_ibs, *ibsFiltered);
    
    std::cout<<"Done and saved as "<<new_ibs_field<<std::endl;
    
    
 
    // By default compute spin cloud for 8 orientations
    // Can be changed and passed as parameter
    // int n_orientations=8;
    // It was simpler to compute and store the descriptor for X-orientations than
    // for 1 orientation and then rotate X-times at test time.
    // So we compute this X-orientations and store them for testing.

    // Spin cloud for weight-sampled
    createSpin( new_sampleCloud2, ibsFiltered, aff_path );
    // New representation for agglomerative descriptor
    // In following release, multiple affordaces can be detected
    // at same time, single affordance representation (this code)
    // is adapated to work with newer code. This "adaptation" is
    // basically wrap (or format) the descriptor in a highly
    // parallelizble way.

    if(getAggloRepresentation(mags_c,aff_path))
    {
        std::cout<<"Everything ok"<<std::endl;
    }
    
    // Spin cloud for uniform sampled
    createSpin(new_sampleCloudU,ibsFiltered,aff_path,8,true);
    if(getAggloRepresentation(mags_cU,aff_path,true))
    {
        std::cout<<"Everything ok"<<std::endl;
    }
    
}


 std::vector<float> IT::getSamplingProbabilities(pcl::PointCloud<pcl::PointNormal>::Ptr clout_in, float minV, float maxV){
	 std::vector<float> probs( clout_in->size() );
	 
	 for(int i=0;i<clout_in->size();i++)
	 {
	   Eigen::Vector3f oldNormal( clout_in->at(i).normal_x, clout_in->at(i).normal_y, clout_in->at(i).normal_z);
	   probs.at(i) = Util_iT::getValueProporcionsRule( oldNormal.norm(), minV, maxV, 1, 0);
	 }
	 return probs;
	  
 }


bool IT::getAggloRepresentation(std::vector<float> &mags, std::string pathh, bool uniform)
{

    int sampleSize=vectors.rows();
    PointCloudT::Ptr aux_cloud(new PointCloudT);
    aux_cloud->resize(descriptor.rows());
    PointCloudT::Ptr useful_cloud(new PointCloudT);
    useful_cloud->resize(descriptor.rows());
    PointCloudT::Ptr better_approx(new PointCloudT);
    better_approx->resize(descriptor.rows());
    PointCloudT::Ptr bare_points(new PointCloudT);
    bare_points->resize(descriptor.rows());
    PointCloudT::Ptr vector_ids_agglomerative(new PointCloudT);
    vector_ids_agglomerative->resize(descriptor.rows());
    PointCloudT::Ptr vectors_data_cloud(new PointCloudT);
    vectors_data_cloud->resize(descriptor.rows());
    // point counts per affordance per orientation
    // Mostly useful for normalization in multiple affordance prediction
    // for single affordance case: 1x8 matrix with sampleSize in each element
    int n_orientations=descriptor.rows()/vectors.rows();
    Eigen::MatrixXf data_individual(1,n_orientations);
    data_individual<<Eigen::MatrixXf::Zero(1,n_orientations);


    for(int i=0;i<descriptor.rows();i++)
    {
        int orientation_id=std::floor(i/sampleSize); // [0-nOrientations) default: 8 orientations
        data_individual(0,orientation_id)+=1;
        int smaller_id=i-(sampleSize*orientation_id); // [0-sampleSize)
        aux_cloud->at(i).x=1;
        aux_cloud->at(i).y=i;
        aux_cloud->at(i).z=0;

        useful_cloud->at(i).x=1;
        useful_cloud->at(i).y=orientation_id;
        useful_cloud->at(i).z=smaller_id;

        better_approx->at(i).x=bare_points->at(i).x=descriptor(i,0);
        better_approx->at(i).y=bare_points->at(i).y=descriptor(i,1);
        better_approx->at(i).z=bare_points->at(i).z=descriptor(i,2);

        vector_ids_agglomerative->at(i).x=vectors(smaller_id,0);
        vector_ids_agglomerative->at(i).y=vectors(smaller_id,1);
        vector_ids_agglomerative->at(i).z=vectors(smaller_id,2);

        Eigen::Vector3f aVector=vectors.row(smaller_id);
        vectors_data_cloud->at(i).x=aVector.norm();
        vectors_data_cloud->at(i).y=mags.at(smaller_id);
        vectors_data_cloud->at(i).z=0;
    }
    std::cout<<"Point counts "<<data_individual<<std::endl;
    std::string base_name;
    std::stringstream ii;
    ii<<n_orientations;
    // Save everything with a correct name
    // if uniform sampling or different
    if(uniform)
        base_name = pathh + "UNew_"+ this->affordanceName + "_" + this->objectName + "_descriptor_" + ii.str();
    else
        base_name = pathh + "New_" + this->affordanceName + "_" + this->objectName + "_descriptor_" + ii.str();
    
    std::string file_name=base_name+"_members.pcd";
    pcl::io::savePCDFile(file_name.c_str(),*aux_cloud);
    
    file_name=base_name+"_extra.pcd";
    pcl::io::savePCDFile(file_name.c_str(),*useful_cloud);
    
    file_name=base_name+".pcd";
    pcl::io::savePCDFile(file_name.c_str(),*better_approx);
    
    file_name=base_name+"_points.pcd";
    pcl::io::savePCDFile(file_name.c_str(),*bare_points);
    
    file_name=base_name+"_vectors.pcd";
    pcl::io::savePCDFile(file_name.c_str(),*vector_ids_agglomerative);
    
    file_name=base_name+"_vdata.pcd";
    pcl::io::savePCDFile(file_name.c_str(),*vectors_data_cloud);
    
    file_name=pathh+this->affordanceName+"_"+this->objectName+"_point_count.dat";
    std::ofstream ofs (file_name, std::ofstream::out);
    pcl::saveBinary(data_individual,ofs);
    return true;
}

//TODO Erase me, this way of calculating translations is not necesary
void IT::translateCloud( pcl::PointCloud<pcl::PointXYZ>::Ptr in, pcl::PointCloud<pcl::PointXYZ>::Ptr out, pcl::PointXYZ translation){//TODO  no comprendo por que se toma como referencia el cetroide de la nube de puntos
    
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*in,centroid);
    
    pcl::PointXYZ reference;
    reference.getVector4fMap() = centroid;
    
    translateCloud( in,  out,  translation,  reference);
    
//     transform.translation() << translation.x-centroid[0],translation.y-centroid[1],translation.z-centroid[2];
//     transform.rotate (Eigen::AngleAxisf (0, Eigen::Vector3f::UnitZ()));
//     pcl::transformPointCloud (*in, *out, transform);
}
     
void IT::translateCloud( pcl::PointCloud<pcl::PointXYZ>::Ptr in, pcl::PointCloud<pcl::PointXYZ>::Ptr out, pcl::PointXYZ translation, pcl::PointXYZ reference){
    
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    
    transform.translation() << translation.x-reference.x,translation.y-reference.y,translation.z-reference.z;
    transform.rotate (Eigen::AngleAxisf (0, Eigen::Vector3f::UnitZ()));
    pcl::transformPointCloud (*in, *out, transform);
}


//TODO origin is always setted as true some sections in this code is unreachable
void IT::rotateCloud( pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out, float angle, char axis,bool origin){
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    Eigen::Vector4f centroid (Eigen::Vector4f::Zero());
    pcl::compute3DCentroid(*cloud_in, centroid);
    Eigen::Vector3f ax;
    switch(axis)
    {
    case 'x': ax = Eigen::Vector3f::UnitX();
        break;
    case 'y': ax = Eigen::Vector3f::UnitY();
        break;
    case 'z': ax = Eigen::Vector3f::UnitZ();
        break;
    }
    Eigen::Matrix3f rotation(Eigen::AngleAxisf (angle, ax));
    transform.rotate(rotation);
    transform.translation()<<0,0,0;
    pcl::transformPointCloud (*cloud_in, *cloud_out, transform);
    if (!origin)   //TODO This condition is never reached, it is not necesary this way of calculating translations
    {
        Eigen::Vector4f centroid_new (Eigen::Vector4f::Zero());
        pcl::compute3DCentroid(*cloud_out, centroid_new);
        //Eigen::Vector4f diff=centroid-centroid_new; //unused variable
        translateCloud(cloud_out,cloud_out,pcl::PointXYZ(centroid[0],centroid[1],centroid[2]));
    }
    
}
    
    
void IT::rotateCloud( pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out, float angle, char axis,pcl::PointXYZ pivot){
    
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    Eigen::Vector4f centroid (Eigen::Vector4f::Zero());
    Eigen::Vector3f ax;
    //pcl::PointXYZ pivot;
    int id=cloud_in->size();
    cloud_in->push_back(pivot);
    //pivot.x=cloud_in->at(pivot_id).x;
    //pivot.y=cloud_in->at(pivot_id).y;
    //pivot.z=cloud_in->at(pivot_id).z;
    switch(axis)
    {
    case 'x':ax=Eigen::Vector3f::UnitX();
        break;
    case 'y':ax=Eigen::Vector3f::UnitY();
        break;
    case 'z':ax=Eigen::Vector3f::UnitZ();
        break;
    }
    Eigen::Matrix3f rotation(Eigen::AngleAxisf (angle, ax));
    transform.rotate(rotation);
    transform.translation()<<0,0,0;
    pcl::transformPointCloud (*cloud_in, *cloud_out, transform);
    pcl::PointXYZ newPivot=cloud_out->at(id);
    //newPivot.x=cloud_out->at(pivot_id).x;
    //newPivot.y=cloud_out->at(pivot_id).y;
    //newPivot.z=cloud_out->at(pivot_id).z;
    translateCloud(cloud_out,cloud_out,pivot,newPivot);
    PointCloud::iterator iter=cloud_out->end();
    --iter;
    cloud_out->erase(iter);
    
}

void IT::getSpinMatrix(pcl::PointCloud<PointWithVector>::Ptr sample, int orientations, pcl::PointCloud<pcl::PointXYZ>::Ptr full){
    
    pcl::PointCloud<PointWithVector>::iterator iter=sample->end();
    PointCloudT::Ptr relativePoints(new PointCloud);
    PointWithVector p=sample->at(sample->size()-2);
    pcl::PointXYZ refPointIBS(p.x,p.y,p.z);
    int refPointIBSId=int(p.v1);
    std::cout<<"ref: "<<refPointIBSId<<std::endl;
    pcl::PointXYZ ref(full->at(refPointIBSId).x-refPointIBS.x,full->at(refPointIBSId).y-refPointIBS.y,full->at(refPointIBSId).z-refPointIBS.z);
    //pcl::PointXYZ ref2=full->at(refPointIBSId);//unused variable

    //pcl::PointXYZ actual_ref()
    --iter;
    sample->erase(iter);
    --iter;
    sample->erase(iter);

    descriptor.resize(sample->size()*orientations,3);
    vectors.resize(sample->size(),3);
    PointCloud::Ptr xyz_target(new PointCloud);
    std::cout<<"REf: "<<ref<<std::endl;
    for(int j=0;j<sample->size();j++)
    {
        PointWithVector aP;
        aP.x=sample->at(j).x-ref.x;
        aP.y=sample->at(j).y-ref.y;
        aP.z=sample->at(j).z-ref.z;
        relativePoints->push_back(pcl::PointXYZ(aP.x,aP.y,aP.z));
        //descriptor.row(j)=aP.getVector3fMap();
        Eigen::Vector3f v(sample->at(j).v1,sample->at(j).v2,sample->at(j).v3);
        vectors.row(j)=v;
        //mags.at(j)=(v.norm()-v.norm()*.2)*(v.norm()-v.norm()*.2);
        //lengths.at(j)=1-((v.norm()- minW) * (1 - 0) / (maxW - minW) + 0);
        //xyz_target->push_back(pcl::PointXYZ(aP.x,aP.y,aP.z));
    }
    PointCloudC::Ptr anchor(new PointCloudC);
    pcl::PointXYZRGB coloredanchor(0,255,0);
    coloredanchor.x=ref.x;
    coloredanchor.y=ref.y;
    coloredanchor.z=ref.z;
    anchor->push_back(coloredanchor);
    //viewer->addPointCloud(anchor,"Anchor");
    //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5,"Anchor");
    PointCloud::Ptr spinCloud(new PointCloud);
    PointCloud::Ptr relative_spin(new PointCloud);
    PointCloud::Ptr xyz_2(new PointCloud);
    pcl::copyPointCloud(*sample,*xyz_target);
    pcl::copyPointCloud(*xyz_target,*spinCloud);
    pcl::copyPointCloud(*relativePoints,*relative_spin);
    std::cout<<"Spining "<<xyz_2->size()<<" points"<<std::endl;
    //if(viewer->contains("Spincloud"))
    //  viewer->updatePointCloud(relative_spin,"Spincloud");
    //else
    //  viewer->addPointCloud(relative_spin,"Spincloud");
    //while(!viewer->wasStopped())
    //        viewer->spinOnce(100);
    //    viewer->resetStoppedFlag();

    for (int i=1;i<orientations;i++)
    {
        pcl::copyPointCloud(*sample,*xyz_2);
        rotateCloud(xyz_2,xyz_target, i*2*M_PI/orientations,'z',ref);
        *spinCloud+=*xyz_target;
        pcl::copyPointCloud(*relativePoints,*xyz_2);
        rotateCloud(xyz_2,xyz_target, i*2*M_PI/orientations,'z',true);
        *relative_spin+=*xyz_target;
        //std::cout<<"Spincloud: "<<xyz_target->size()<<std::endl;
        //if(viewer->contains("Spincloud"))
        //            viewer->updatePointCloud(relative_spin,"Spincloud");
        //        else
        //            viewer->addPointCloud(relative_spin,"Spincloud");
        //        while(!viewer->wasStopped())
        //            viewer->spinOnce(100);
        //        viewer->resetStoppedFlag();

    }
    for(int i=0;i<spinCloud->size();i++)
        descriptor.row(i)=Eigen::Vector3f(relative_spin->at(i).x,relative_spin->at(i).y,relative_spin->at(i).z);
    std::cout<<"Descriptor "<<descriptor.rows()<<std::endl;
}

bool IT::createSpin( pcl::PointCloud<PointWithVector>::Ptr sample,  pcl::PointCloud<pcl::PointXYZ>::Ptr full_ibs, std::string pathh,int orientations,  bool uniform){
    std::stringstream ii;
    ii<<orientations;
    getSpinMatrix(sample,orientations,full_ibs);
    std::string spin_file;
    std::string spinvectors_file;
    if(uniform)
    {
        spin_file        = pathh + this->affordanceName + "_" + this->objectName + "_spinU_" + ii.str() + ".dat";
        spinvectors_file = pathh + this->affordanceName + "_" + this->objectName + "_spinUvectors_" + ii.str() + ".dat";
    }
    else
    {
        spin_file        = pathh + this->affordanceName + "_" + this->objectName + "_spin_"+ii.str()+".dat";
        spinvectors_file = pathh + this->affordanceName + "_" + this->objectName + "_spinvectors_"+ii.str()+".dat";
    }
    std::ofstream file(spin_file.c_str());
    pcl::saveBinary(descriptor,file);
    std::ofstream file2(spinvectors_file.c_str());
    pcl::saveBinary(vectors,file2);
    std::cout<<"Wrote: "<<spin_file<<" and "<<spinvectors_file<<std::endl;
    //if reached this point everything is ok
    return true;
}




#include "it.h"



IT::IT( pcl::PointCloud<pcl::PointXYZ>::Ptr scene, pcl::PointCloud<pcl::PointXYZ>::Ptr object, std::string name_affordance, std::string name_object){
       
    this->sceneCloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    this->objectCloud.reset(new pcl::PointCloud<pcl::PointXYZ>);

    this->sceneCloudFiltered.reset(new pcl::PointCloud<pcl::PointXYZ>);
    this->ibsFiltered.reset(new pcl::PointCloud<pcl::PointXYZ>);
    
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
    pcl::PointXYZ   middlePointObject;  // this is used to construct all reference poins
    pcl::PointXYZ min, max;
    float radio;
    
    pcl::getMinMax3D(*objectCloud,min,max);
    
    middlePointObject.x=(max.x+min.x)/2;
    middlePointObject.y=(max.y+min.y)/2;
    middlePointObject.z=(max.z+min.z)/2;
    
    radio=pcl::L2_Norm(min.getArray3fMap(),max.getArray3fMap(),3);
    
    
    
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // extract volume of interest from the scene
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
//     ibsFiltered = Util::sphereExtraction(ibs_calculator.getIBS(), middlePointObject,radio);
    
    
//     //TODO erase this, it is only for developing porpouses
    
    pcl::io::loadPCDFile("../../test/data/ibs_clouds_prefiltered_filtered.pcd", *ibsFiltered);
    
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
    std::cout<< pv_it <<std::endl;
    
    
sw.Restart();    
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // it is possible that this section is useless
    
    // Save the original provenance vectors bacause we are going to normalize
    pcl::PointCloud<pcl::PointNormal>::Ptr normals_backup(new pcl::PointCloud<pcl::PointNormal>);
    pcl::copyPointCloud(*field,*normals_backup);
    
    
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
  
    Sampler_iT* samplerW = new SamplerWeighted_iT(field, nMin, nMax, sampleSize);
    samplerW->calculateSample();
    
    new_sampleCloud2 = samplerW->sample;
    
    
    Sampler_iT* samplerU = new SamplerUniform_iT(field, sampleSize);
    samplerU->calculateSample();
    
    new_sampleCloudU = samplerU->sample;
    
    
    //Save mags in sampled mapped in 0-1 based on full tensor mags
    mags_c  = Util_iT::calculatedMappedMagnitudesToVector( *new_sampleCloud2, nMin, nMax, 1, 0 );
    mags_cU = Util_iT::calculatedMappedMagnitudesToVector( *new_sampleCloudU, nMin, nMax, 1, 0 );
    
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Saving iT    
    
    // Some file names to save data
    // Some of the data saved in this file is only kept
    // to not break previous code but for most recent
    // version is not used.
    
    //Check a path exists
    std::string aff_path = prepareDirectory();


    // Some data for info file
    // Closest point in Tensor to scene and in object to scene
    // were used previously to estimate the pose of Tensor/Object
    // relative to scene. These are still computed and save but no longer
    // used, the new pose is computed using center of bounding boxes.
    std::cout<<"Getting closest point in Tensor to scene"<<std::endl;
    this->defineReferences(middlePointObject);
    
    this->saveBasicInfo(aff_path);
    
    
    
    // Scene-to-IBS and Scene-to-object are saved in affordance keypoints file
    // As commented earlier it was used to align pointclouds
    // at test time. No longer used but still kept in files.
    PointWithVector secondtolast;
    secondtolast.x  = vectSceneToIBS[0];
    secondtolast.y  = vectSceneToIBS[1];
    secondtolast.z  = vectSceneToIBS[2];
    secondtolast.v1 = idxRefIBS;
    secondtolast.v2 = 0;
    secondtolast.v3 = 0;
    
    
    PointWithVector last;
    last.x  = vectSceneToObject[0];
    last.y  = vectSceneToObject[1];
    last.z  = vectSceneToObject[2];
    last.v1 = idxRefObject;
    last.v2 = 0;
    last.v3 = 0;

    
    new_sampleCloud2->push_back(secondtolast);
    new_sampleCloud2->push_back(last);
    
    new_sampleCloudU->push_back(secondtolast);
    new_sampleCloudU->push_back(last);
    
    
    // Save everything
    std::string new_ibs_field   = aff_path + this->affordanceName + "_" + this->objectName + "_field.pcd";
    std::string new_ibs_sample  = aff_path + "ibs_sample_" + std::to_string(sampleSize) + "_" + this->affordanceName + "_" + this->objectName + "_better.pcd";
    std::string new_ibs_sampleU = aff_path + "ibs_sample_" + std::to_string(sampleSize) + "_" + this->affordanceName + "_" + this->objectName + "_betterUniform.pcd";
    
    std::string smoother_field  = aff_path + this->affordanceName + "_" +this->objectName + "_smoothfield.pcd";
    std::string clean_ibs       = aff_path + "ibs_full_" + this->affordanceName + "_" + this->objectName + "_clean.pcd";  // TODO It doesn't have any sense
    std::string full_ibs        = aff_path + "ibs_full_" + this->affordanceName + "_" + this->objectName + ".pcd";
    
    pcl::io::savePCDFileASCII( new_ibs_field.c_str(), *field);
    pcl::io::savePCDFile( new_ibs_sample.c_str(), *new_sampleCloud2);   // here reference point are saved, it is better not to save them 
    pcl::io::savePCDFile( new_ibs_sampleU.c_str(), *new_sampleCloudU);  // I erase such necesity in the spin creations
    
    pcl::io::savePCDFile( smoother_field.c_str(), *smoothField);
    pcl::io::savePCDFile( clean_ibs, *copyIBS);
    pcl::io::savePCDFile( full_ibs, *ibsFiltered);
    
    std::cout<<"Done and saved as "<<new_ibs_field<<std::endl;
    
    
    //TODO I add this point but inmediatly erase it because I erased their necesity in the "SPIN CALCULATION"
    new_sampleCloud2->erase( new_sampleCloud2->end()-1 );
    new_sampleCloud2->erase( new_sampleCloud2->end()-1 );
    new_sampleCloudU->erase( new_sampleCloudU->end()-1 );
    new_sampleCloudU->erase( new_sampleCloudU->end()-1 );
    
 
    // By default compute spin cloud for 8 orientations
    // Can be changed and passed as parameter
    // int n_orientations=8;
    // It was simpler to compute and store the descriptor for X-orientations than
    // for 1 orientation and then rotate X-times at test time.
    // So we compute this X-orientations and store them for testing.

    // Spin cloud for weight-sampled
    Spinner_iT spinnerW( new_sampleCloud2, refPointScene, numOrientations);
    spinnerW.calculateSpinings();
    saveSpin(spinnerW, aff_path );
    // New representation for agglomerative descriptor in following release, multiple affordaces can be detected
    // at same time, single affordance representation (this code) is adapated to work with newer code. This "adaptation" is
    // basically wrap (or format) the descriptor in a highly parallelizble way.
    Agglomerator_IT agglomeratorW( spinnerW.vectors, spinnerW.descriptor, mags_c );
    agglomeratorW.compileAgglomeration();
    saveAggloRepresentation(agglomeratorW,aff_path);
    
    
    // Spin cloud for uniform sampled
    Spinner_iT spinnerU( new_sampleCloudU, refPointScene, numOrientations);
    spinnerU.calculateSpinings();
    saveSpin(spinnerU,aff_path,true);
    
    Agglomerator_IT agglomeratorU( spinnerU.vectors, spinnerU.descriptor, mags_c );
    agglomeratorU.compileAgglomeration();
    saveAggloRepresentation(agglomeratorU, aff_path,true);
    
    
}



bool IT::saveAggloRepresentation(Agglomerator_IT agglomerator,  std::string pathh, bool uniform)
{

   
    std::cout<<"Point counts "<<agglomerator.data_individual<<std::endl;
    std::string base_name;
    
    // Save everything with a correct name
    // if uniform sampling or different
    if(uniform)
        base_name = pathh + "UNew_"+ this->affordanceName + "_" + this->objectName + "_descriptor_" + std::to_string(agglomerator.n_orientations);
    else
        base_name = pathh + "New_" + this->affordanceName + "_" + this->objectName + "_descriptor_" + std::to_string(agglomerator.n_orientations);
    
    std::string file_name=base_name+"_members.pcd";
    pcl::io::savePCDFile(file_name.c_str(),*agglomerator.aux_cloud);
    
    file_name=base_name+"_extra.pcd";
    pcl::io::savePCDFile(file_name.c_str(),*agglomerator.useful_cloud);
    
    file_name=base_name+".pcd";
    pcl::io::savePCDFile(file_name.c_str(),*agglomerator.better_approx);
    
    file_name=base_name+"_points.pcd";
    pcl::io::savePCDFile(file_name.c_str(),*agglomerator.bare_points);
    
    file_name=base_name+"_vectors.pcd";
    pcl::io::savePCDFile(file_name.c_str(),*agglomerator.vector_ids_agglomerative);
    
    file_name=base_name+"_vdata.pcd";
    pcl::io::savePCDFile(file_name.c_str(),*agglomerator.vectors_data_cloud);
    
    file_name=pathh+this->affordanceName+"_"+this->objectName+"_point_count.dat";
    std::ofstream ofs (file_name, std::ofstream::out);
    pcl::saveBinary(agglomerator.data_individual,ofs);
    
    return true;
}



bool IT::saveSpin(Spinner_iT spinner, std::string pathh, bool uniform){
    
    std::string spin_file;
    std::string spinvectors_file;
    
    if(uniform)
    {
        spin_file        = pathh + this->affordanceName + "_" + this->objectName + "_spinU_" + std::to_string(numOrientations) + ".dat";
        spinvectors_file = pathh + this->affordanceName + "_" + this->objectName + "_spinUvectors_" + std::to_string(numOrientations) + ".dat";
    }
    else
    {
        spin_file        = pathh + this->affordanceName + "_" + this->objectName + "_spin_"+ std::to_string(numOrientations)+".dat";
        spinvectors_file = pathh + this->affordanceName + "_" + this->objectName + "_spinvectors_"+ std::to_string(numOrientations)+".dat";
    }
    
    std::ofstream file(spin_file.c_str());
    pcl::saveBinary(spinner.descriptor,file);
    
    std::ofstream file2(spinvectors_file.c_str());
    pcl::saveBinary(spinner.vectors,file2);
    
    return true;
}


void IT::saveBasicInfo( std::string aff_path ){
    // Info file name
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
        output_file<<"Scene name:"<<scene_name<<"\n"; 
        output_file<<"Object name:"<<this->objectName<<"\n";
        output_file<<"Clusters:"<<clusters<<"\n";
        for(int i=0;i<clusters;i++)
            output_file<<refPointIBS.x<<","<<refPointIBS.y<<","<<refPointIBS.z<<"\n";
        output_file<<"Distance threshold:"<<size<<"\n";
        output_file<<"Reference:"<<referencing<<"\n";
        output_file<<idxRefIBS<<":"<<refPointIBS.x<<","<<refPointIBS.y<<","<<refPointIBS.z<<"\n";
        output_file<<"ScenePoint\n";
        output_file<<idxRefScene<<":"<<refPointScene.x<<","<<refPointScene.y<<","<<refPointScene.z<<"\n";
        output_file<<"IbsPointVector\n";
        output_file<<idxRefIBS<<":"<<vectSceneToIBS[0]<<","<<vectSceneToIBS[1]<<","<<vectSceneToIBS[2]<<"\n";
        output_file<<"ObjPointVector\n";
        output_file<<idxRefObject<<":"<<vectSceneToObject[0]<<","<<vectSceneToObject[1]<<","<<vectSceneToObject[2]<<"\n";
        //TODO this is unnecesary
        //         output_file<<"Object Transformation\n";
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


void IT::defineReferences(pcl::PointXYZ anchorPoint){
    //TODO resolver como se ajustan estas relaciones, de "PUNTOS DE REFERENCIA", por ahora se calcula como el punto mas cercano al centroide del objeto    
    this->idxRefScene   = Util_iT::indexOfClosestPointInACloud ( sceneCloudFiltered, anchorPoint );
    this->refPointScene = sceneCloudFiltered->at(idxRefScene);
    
    this->idxRefIBS     = Util_iT::indexOfClosestPointInACloud ( ibsFiltered, refPointScene); //"ClosestTensor"
    this->refPointIBS   = ibsFiltered->at(idxRefIBS);
    
    this->idxRefObject   = Util_iT::indexOfClosestPointInACloud ( objectCloud, refPointScene );
    this->refPointObject = objectCloud->at(idxRefObject);
    
    this->vectSceneToIBS[0] = refPointIBS.x - refPointScene.x;
    this->vectSceneToIBS[1] = refPointIBS.y - refPointScene.y;
    this->vectSceneToIBS[2] = refPointIBS.z - refPointScene.z;
    
    this->vectSceneToObject[0] = objectCloud->at(idxRefObject).x - refPointScene.x;
    this->vectSceneToObject[1] = objectCloud->at(idxRefObject).y - refPointScene.y;
    this->vectSceneToObject[2] = objectCloud->at(idxRefObject).z - refPointScene.z;
    
}


std::string IT::prepareDirectory(){
    
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
    return aff_path;
}

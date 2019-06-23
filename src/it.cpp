#include "it.h"



IT::IT( pcl::PointCloud<pcl::PointXYZ>::Ptr scene, pcl::PointCloud<pcl::PointXYZ>::Ptr object, std::string name_affordance, std::string name_object){
       
    this->sceneCloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    this->objectCloud.reset(new pcl::PointCloud<pcl::PointXYZ>);

    this->sceneCloudFiltered.reset(new pcl::PointCloud<pcl::PointXYZ>);
    this->ibsFiltered.reset(new pcl::PointCloud<pcl::PointXYZ>);
    
    this->sampleW.reset(new pcl::PointCloud<PointWithVector>);
    this->sampleU.reset(new pcl::PointCloud<PointWithVector>);
    
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

    
 StopWatch sw;
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // calculate the Interaction Bisector Surface (IBS)
    IBS ibs_calculator(sceneCloudFiltered,objectCloud);

sw.Restart();
    ibs_calculator.calculate();
std::cout << "TIMER: ISB calculation " << sw.ElapsedMs() << std::endl;
    
    
sw.Restart();
    //as IBS extend to infinity, filter IBS
    ibsFiltered = Util::sphereExtraction(ibs_calculator.getIBS(), middlePointObject,radio);
    
    
     //This is only for developing porpouses    
    //pcl::io::loadPCDFile("../../test/calculation_data/ibs_clouds_prefiltered_filtered.pcd", *ibsFiltered);
    
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Searching nearest neighbours (from IBS to ) and calculating smoot provenance vector

    
    pcl::PointCloud<pcl::PointNormal>::Ptr  field;
    pcl::PointCloud<pcl::PointNormal>::Ptr  smoothField;
    
    float maxV, minV, maxS, minS;
    float  sum, sumSmooth;
    
    pv_it = new ProvenanceVectors_iT(ibsFiltered, sceneCloudFiltered);
    pv_it->calculateProvenanceVectors(5);
    
    field       = pv_it->rawProvenanceVectors;
    smoothField = pv_it->smoothedProvenanceVectors;
    maxV        = pv_it->maxV;
    minV        = pv_it->minV;
    maxS        = pv_it->maxS;
    minS        = pv_it->minS;
    sum         = pv_it->sum;
    sumSmooth   = pv_it->sumSmooth;
    
     //pcl::io::savePCDFile("test_1_pv_calculation_field.pcd",*field);
     //pcl::io::savePCDFile("test_1_pv_calculation_smoothField.pcd",*smoothField);
    
    
std::cout << "TIMER: Filtering " << sw.ElapsedMs() << std::endl;

    // Print out some data about the tensor and filtering/cleaning
    std::cout<< *pv_it <<std::endl;
    
    
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
    
    //BEGINING Mapping norm magnitudes to [1,0]
//     Util_iT::mapMagnitudes( *smoothField, minV, maxV, 1, 0 );
//     Util_iT::mapMagnitudes( *field, minV, maxV, 1, 0 );
//     nMin=0;
//     nMax=1;
    //END Mapping norm magnitudes to [1,0]
    
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

sw.Restart();    

    SamplerWeighted_iT samplerW (field, nMin, nMax, sampleSize);
    samplerW.calculateSample();
    
    sampleW = samplerW.sample;
    
    
    SamplerUniform_iT samplerU(field, sampleSize);
    samplerU.calculateSample();
    
    sampleU = samplerU.sample;
    
    
   
    

std::cout << "TIMER: Sampling " << sw.ElapsedMs() << std::endl;
    
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Spinning and agglomerating
    // Some data for info file
    // Closest point in Tensor to scene and in object to scene
    // were used previously to estimate the pose of Tensor/Object
    // relative to scene. These are still computed and save but no longer
    // used, the new pose is computed using center of bounding boxes.
    
sw.Restart();    
    
    this->defineReferences(middlePointObject);
    
    // By default compute spin cloud for 8 orientations
    // New representation for agglomerative descriptor in following release, multiple affordaces can be detected
    // at same time, single affordance representation (this code) is adapated to work with newer code. This "adaptation" is
    // basically wrap (or format) the descriptor in a highly parallelizble way.

    // Spin cloud for weight-sampled
    Spinner_iT spinnerW ( sampleW, refPointScene, numOrientations);
    spinnerW.calculateSpinings();
    
    this->spinnedVectorsW    = spinnerW.vectors;
    this->spinnedDescriptorW = spinnerW.descriptor;
    
    //Save mags in sampled mapped in 0-1 based on full tensor mags
    std::vector<float> mappedPVNormsW;
    mappedPVNormsW  = Util_iT::mapMagnitudes( samplerW.vectorsNorms, nMin, nMax, 1, 0 );
    
    agglomeratorW = new Agglomerator_IT( this->spinnedVectorsW, this->spinnedDescriptorW, samplerW.vectorsNorms, mappedPVNormsW );
    agglomeratorW->compileAgglomeration();
    
  
    // Spin cloud for uniform sampled
    Spinner_iT spinnerU ( sampleU, refPointScene, numOrientations);
    spinnerU.calculateSpinings();
    
    this->spinnedVectorsU    = spinnerU.vectors;
    this->spinnedDescriptorU = spinnerU.descriptor;
    
    //Mags in sampled mapped in 0-1 based on full tensor mags
    std::vector<float> mappedPVNormsU; 
    mappedPVNormsU = Util_iT::mapMagnitudes( samplerU.vectorsNorms, nMin, nMax, 1, 0 );
    
    agglomeratorU = new Agglomerator_IT( this->spinnedVectorsU, this->spinnedDescriptorU, samplerU.vectorsNorms, mappedPVNormsU );
    agglomeratorU->compileAgglomeration();
    
    
std::cout << "TIMER: Spinning and agglomering " << sw.ElapsedMs() << std::endl;
    
    
}

void IT::saveFiles()
{
    //Prepare path
    std::string aff_path = prepareDirectory();

    this->saveBasicInfo(aff_path);
    
    this->saveSceneAndQueryObjects(aff_path);
    
    this->saveProvenanceIBS(aff_path);
    
    this->saveSpin( aff_path );
    this->saveAggloRepresentation(agglomeratorW,aff_path);
    
    this->saveSpin( aff_path, true );
    this->saveAggloRepresentation(agglomeratorU, aff_path,true);

}



bool IT::saveAggloRepresentation(Agglomerator_IT* agglomerator,  std::string pathh, bool uniform)
{

   
    std::cout<<"Point counts "<<agglomerator->data_individual<<std::endl;
    std::string base_name;
    
    // Save everything with a correct name
    // if uniform sampling or different
    if(uniform)
        base_name = pathh + "UNew_"+ this->affordanceName + "_" + this->objectName + "_descriptor_" + std::to_string(agglomerator->n_orientations);
    else
        base_name = pathh + "New_" + this->affordanceName + "_" + this->objectName + "_descriptor_" + std::to_string(agglomerator->n_orientations);
    
    std::string file_name=base_name+"_members.pcd";
    pcl::io::savePCDFile(file_name.c_str(),*agglomerator->aux_cloud);
    
    file_name=base_name+"_extra.pcd";
    pcl::io::savePCDFile(file_name.c_str(),*agglomerator->useful_cloud);
    
    file_name=base_name+".pcd";
    pcl::io::savePCDFile(file_name.c_str(),*agglomerator->better_approx);
    
    file_name=base_name+"_points.pcd";
    pcl::io::savePCDFile(file_name.c_str(),*agglomerator->bare_points);
    
    file_name=base_name+"_vectors.pcd";
    pcl::io::savePCDFile(file_name.c_str(),*agglomerator->vector_ids_agglomerative);
    
    file_name=base_name+"_vdata.pcd";
    pcl::io::savePCDFile(file_name.c_str(),*agglomerator->vectors_data_cloud);
    
    file_name=pathh+this->affordanceName+"_"+this->objectName+"_point_count.dat";
    std::ofstream ofs (file_name);
    pcl::saveBinary(agglomerator->data_individual,ofs);
    
    return true;
}


bool IT::saveSpin(std::string pathh, bool uniform){
    
    std::string spin_file;
    std::string spinvectors_file;
    
    if(uniform)
    {
        spin_file        = pathh + this->affordanceName + "_" + this->objectName + "_spinU_" + std::to_string(numOrientations) + ".dat";
        spinvectors_file = pathh + this->affordanceName + "_" + this->objectName + "_spinUvectors_" + std::to_string(numOrientations) + ".dat";
  
        std::ofstream file(spin_file.c_str());
        pcl::saveBinary(this->spinnedDescriptorU,file);
        
        std::ofstream file2(spinvectors_file.c_str());
        pcl::saveBinary(this->spinnedVectorsU,file2);
    }
    else
    {
        spin_file        = pathh + this->affordanceName + "_" + this->objectName + "_spin_"+ std::to_string(numOrientations)+".dat";
        spinvectors_file = pathh + this->affordanceName + "_" + this->objectName + "_spinvectors_"+ std::to_string(numOrientations)+".dat";
        
        std::ofstream file(spin_file.c_str());
        pcl::saveBinary( this->spinnedDescriptorW, file );
        
        std::ofstream file2(spinvectors_file.c_str());
        pcl::saveBinary( this->spinnedVectorsW ,file2);
    }
    
    
    return true;
}


void IT::saveProvenanceIBS(std::string aff_path)
{
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

    
    sampleW->push_back(secondtolast);
    sampleW->push_back(last);
    
    sampleU->push_back(secondtolast);
    sampleU->push_back(last);
    
     // Save everything
    std::string new_ibs_field   = aff_path + this->affordanceName + "_" + this->objectName + "_field.pcd";
    std::string new_ibs_sample  = aff_path + "ibs_sample_" + std::to_string(sampleSize) + "_" + this->affordanceName + "_" + this->objectName + "_better.pcd";
    std::string new_ibs_sampleU = aff_path + "ibs_sample_" + std::to_string(sampleSize) + "_" + this->affordanceName + "_" + this->objectName + "_betterUniform.pcd";
    
    std::string smoother_field  = aff_path + this->affordanceName + "_" +this->objectName + "_smoothfield.pcd";
    std::string full_ibs        = aff_path + "ibs_full_" + this->affordanceName + "_" + this->objectName + ".pcd";
    
    //std::string query_object    = aff_path + this->objectName + ".pcd";
    
    
    pcl::io::savePCDFileASCII( new_ibs_field.c_str(), *pv_it->rawProvenanceVectors);
    pcl::io::savePCDFile( new_ibs_sample.c_str(), *sampleW);   // here reference point are saved, it is better not to save them 
    pcl::io::savePCDFile( new_ibs_sampleU.c_str(), *sampleU);  // I erase such necesity in the spin creations
    
    pcl::io::savePCDFile( smoother_field.c_str(), *pv_it->smoothedProvenanceVectors);
    pcl::io::savePCDFile( full_ibs, *ibsFiltered);
    
    //pcl::io::savePCDFile( query_object, *this->objectCloud); // it is saved two times it is not anymore necesary do it here
    
    
    //TODO I add this point but inmediatly erased it because I erased their necesity in the "SPIN CALCULATION"
    sampleW->erase( sampleW->end()-1 );
    sampleW->erase( sampleW->end()-1 );
    sampleU->erase( sampleU->end()-1 );
    sampleU->erase( sampleU->end()-1 );
    

}

void IT::saveSceneAndQueryObjects(std::string aff_path)
{
    std::string scene_cloud_filename   = aff_path + this->affordanceName + "_" + this->objectName + "_scene_full.pcd";
    pcl::io::savePCDFile( scene_cloud_filename.c_str(), *this->sceneCloud);
    
    std::string scene_cloud_filtered_filename   = aff_path + this->affordanceName + "_" + this->objectName + "_scene_filtered.pcd";
    pcl::io::savePCDFile( scene_cloud_filtered_filename.c_str(), *this->sceneCloudFiltered);
    
    std::string object_cloud_filename   = aff_path + this->affordanceName + "_" + this->objectName + "_object.pcd";
    pcl::io::savePCDFile( object_cloud_filename.c_str(), *this->objectCloud);
    
    //TODO verificar si existe el archivo .PLY
    
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
    
    boost::property_tree::ptree root;
    
    std::ostringstream string_stream;
    
    root.put("Scene name", "not implemented");  //TODO no scene name 
    root.put("Object name", this->objectName);
    root.put("Sample size", this->sampleSize);
    root.put("Orientations", this->numOrientations);
    
    boost::property_tree::ptree refIBS;
    refIBS.put("idxRefIBS", idxRefIBS);
    string_stream << refPointIBS.x<<","<<refPointIBS.y<<","<<refPointIBS.z;
    refIBS.put("refPointIBS", string_stream.str());
    root.add_child("Reference", refIBS);
    
    string_stream.str("");
    boost::property_tree::ptree scenePoint;
    scenePoint.put("idxScenePoint", idxRefScene);
    string_stream << refPointScene.x << "," << refPointScene.y<<","<<refPointScene.z;
    scenePoint.put("refPointScene", string_stream.str());
    root.add_child("ScenePoint", scenePoint);
    
    string_stream.str("");
    boost::property_tree::ptree ibsPointVector;
    ibsPointVector.put("idxRefIBS", idxRefIBS);
    string_stream << vectSceneToIBS[0] << "," << vectSceneToIBS[1]<<","<<vectSceneToIBS[2];
    ibsPointVector.put("vectSceneToIBS", string_stream.str());
    root.add_child("IbsPointVector", ibsPointVector);
    
    string_stream.str("");
    boost::property_tree::ptree objPointVector;
    objPointVector.put("idxRefObject", idxRefObject);
    string_stream << vectSceneToObject[0] << "," << vectSceneToObject[1]<<","<<vectSceneToObject[2];
    objPointVector.put("vectSceneToObject", string_stream.str());
    root.add_child("ObjPointVector", objPointVector);   
    std::string json_file_name= aff_path + this->affordanceName + "_" + this->objectName + ".json";
    boost::property_tree::write_json( json_file_name, root );
}


void IT::defineReferences(pcl::PointXYZ middlePointObject){
    //TODO resolver como se ajustan estas relaciones, de "PUNTOS DE REFERENCIA", por ahora se calcula como el punto mas cercano al centroide del objeto    
    this->idxRefScene   = Util_iT::indexOfClosestPointInACloud ( sceneCloudFiltered, middlePointObject );
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
    
     std::string aff_path ;
     
     aff_path = Util_iT::getWorkingDirectory(this->affordanceName, this->objectName);
     
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








/************************ FILE LOADER************************/

IT IT::loadFiles(std::string affordance_name, std::string object_name)
{

    std::string aff_path = Util_iT::getWorkingDirectory(affordance_name, object_name);
    
    // Create a root
    boost::property_tree::ptree root;
    // Load the json file in this ptree
    std::string json_file_name= aff_path + affordance_name + "_" + object_name + ".json";
    boost::property_tree::read_json(json_file_name, root);
    int sampleSize      = root.get<int>("Sample size");
    int numOrientations = root.get<int>("Orientations");
    
    //load scene point
    std::string scene_cloud_filename   = aff_path + affordance_name + "_" + object_name + "_scene_full.pcd";
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_scene (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile(scene_cloud_filename, *cloud_scene);
    
    
    //load object point 
    std::string object_cloud_filename   = aff_path + affordance_name + "_" + object_name + "_object.pcd";
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_object (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile(object_cloud_filename, *cloud_object);
    
    
    IT itcalculator( cloud_scene, cloud_object, affordance_name, object_name);
    itcalculator.sampleSize = sampleSize;
    itcalculator.numOrientations = numOrientations;
    
    
     //load scene cloud filtered point 
    std::string scene_cloud_filtered_filename = aff_path + affordance_name + "_" + object_name + "_scene_filtered.pcd";
    pcl::io::loadPCDFile(scene_cloud_filtered_filename, *itcalculator.sceneCloudFiltered);
    
    //load ibs_filtered
    std::string full_ibs_filename     = aff_path + "ibs_full_" + affordance_name + "_" + object_name + ".pcd";
    pcl::io::loadPCDFile(full_ibs_filename, *itcalculator.ibsFiltered);
    

    //load provenance vectors
    itcalculator.pv_it = new ProvenanceVectors_iT( itcalculator.ibsFiltered, itcalculator.sceneCloudFiltered );

    std::string new_ibs_field   = aff_path + affordance_name + "_" + object_name + "_field.pcd";
    pcl::io::loadPCDFile( new_ibs_field.c_str(), *itcalculator.pv_it->rawProvenanceVectors);
    std::string smoother_field  = aff_path + affordance_name + "_" +object_name + "_smoothfield.pcd";
    pcl::io::loadPCDFile( smoother_field.c_str(), *itcalculator.pv_it->smoothedProvenanceVectors);
//     pv_it->calculateProvenanceVectors(5);                                 //TODO ///////////////////////////////////////////////////////////////
//     maxV        = pv_it->maxV;
//     minV        = pv_it->minV;
//     maxS        = pv_it->maxS;
//     minS        = pv_it->minS;
//     sum         = pv_it->sum;
//     sumSmooth   = pv_it->sumSmooth;
    
    // load samples
    std::string new_ibs_sample  = aff_path + "ibs_sample_" + std::to_string(sampleSize) + "_" + affordance_name + "_" + object_name + "_better.pcd";
    std::string new_ibs_sampleU = aff_path + "ibs_sample_" + std::to_string(sampleSize) + "_" + affordance_name + "_" + object_name + "_betterUniform.pcd";
    pcl::io::loadPCDFile( new_ibs_sample.c_str(), *itcalculator.sampleW);   // here reference point are saved, it is better not to save them 
    pcl::io::loadPCDFile( new_ibs_sampleU.c_str(), *itcalculator.sampleU);  // I erase such necesity in the spin creations

    // load set of references
    PointWithVector last = itcalculator.sampleU->back();
    itcalculator.vectSceneToObject[0] = last.x;
    itcalculator.vectSceneToObject[1] = last.y;
    itcalculator.vectSceneToObject[2] = last.z;
    itcalculator.idxRefObject = last.v1;
    itcalculator.refPointObject= itcalculator.objectCloud->at( itcalculator.idxRefObject );
       
    itcalculator.sampleW->erase( itcalculator.sampleW->end()-1 );
    itcalculator.sampleU->erase( itcalculator.sampleU->end()-1 );
    
    PointWithVector secondtolast = itcalculator.sampleU->back();
    itcalculator.vectSceneToIBS[0] = secondtolast.x;
    itcalculator.vectSceneToIBS[1] = secondtolast.y;
    itcalculator.vectSceneToIBS[2] = secondtolast.z;
    itcalculator.idxRefIBS = secondtolast.v1;
    itcalculator.refPointIBS   = itcalculator.ibsFiltered->at( itcalculator.idxRefIBS );
  
    itcalculator.sampleW->erase( itcalculator.sampleW->end()-1 );
    itcalculator.sampleU->erase( itcalculator.sampleU->end()-1 );

    itcalculator.idxRefScene =  root.get<int>("ScenePoint.idxScenePoint");
    itcalculator.refPointScene = Util_iT::stringToPoint( root.get<std::string>("ScenePoint.refPointScene") );
    
    //load spinners
    std::string spin_fileU        = aff_path + affordance_name + "_" + object_name + "_spinU_" + std::to_string(numOrientations) + ".dat";
    std::string spinvectors_fileU = aff_path + affordance_name + "_" + object_name + "_spinUvectors_" + std::to_string(numOrientations) + ".dat";
    
    std::ifstream ifs_vectorsU ( spinvectors_fileU );
    itcalculator.spinnedVectorsU.resize( sampleSize, 3 ); 
    pcl::loadBinary( itcalculator.spinnedVectorsU, ifs_vectorsU );
    
    std::ifstream ifs_descriptorU ( spin_fileU );
    itcalculator.spinnedDescriptorU.resize( sampleSize*numOrientations, 3 ); 
    pcl::loadBinary( itcalculator.spinnedDescriptorU, ifs_descriptorU );
   
    std::string spin_fileW        = aff_path + affordance_name + "_" + object_name + "_spin_"+ std::to_string(numOrientations)+".dat";
    std::string spinvectors_fileW = aff_path + affordance_name + "_" + object_name + "_spinvectors_"+ std::to_string(numOrientations)+".dat";

    std::ifstream ifs_vectorsW ( spinvectors_fileW );
    itcalculator.spinnedVectorsW.resize( sampleSize, 3 ); 
    pcl::loadBinary( itcalculator.spinnedVectorsW, ifs_vectorsW );
    
    std::ifstream ifs_descriptorW ( spin_fileW );
    itcalculator.spinnedDescriptorW.resize( sampleSize*numOrientations, 3 ); 
    pcl::loadBinary( itcalculator.spinnedDescriptorW, ifs_descriptorW );
    
    
    //load aglorrepresentation
    std::string file_nameU;
    std::vector<float> pv_norms_U;
    std::vector<float> pv_norms_mappedU;
    std::string base_nameU;
   
    base_nameU = aff_path + "UNew_"+ affordance_name + "_" + object_name + "_descriptor_" + std::to_string(numOrientations);
        
    file_nameU = base_nameU + "_vdata.pcd";
    pcl::PointCloud<pcl::PointXYZ>::Ptr vectors_data_cloudU( new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile(file_nameU.c_str(), *vectors_data_cloudU);
    
    pv_norms_U.reserve(sampleSize);
    pv_norms_mappedU.reserve(sampleSize);
    
    for( int i = 0 ; i < sampleSize ; i++){
        pv_norms_U.push_back( vectors_data_cloudU->at(i).x);
        pv_norms_mappedU.push_back( vectors_data_cloudU->at(i).y );
    }
    
    
    Agglomerator_IT *agglomeratorU = new Agglomerator_IT( itcalculator.spinnedVectorsU, itcalculator.spinnedDescriptorU, pv_norms_U, pv_norms_mappedU );
       

    file_nameU=base_nameU+"_members.pcd";
    pcl::io::loadPCDFile(file_nameU.c_str(),*agglomeratorU->aux_cloud);
    
    file_nameU=base_nameU+"_extra.pcd";
    pcl::io::loadPCDFile(file_nameU.c_str(),*agglomeratorU->useful_cloud);
    
    file_nameU=base_nameU+".pcd";
    pcl::io::loadPCDFile(file_nameU.c_str(),*agglomeratorU->better_approx);
    
    file_nameU=base_nameU+"_points.pcd";
    pcl::io::loadPCDFile(file_nameU.c_str(),*agglomeratorU->bare_points);
    
    file_nameU=base_nameU+"_vectors.pcd";
    pcl::io::loadPCDFile(file_nameU.c_str(),*agglomeratorU->vector_ids_agglomerative);
   
    agglomeratorU->vectors_data_cloud = vectors_data_cloudU;
    
    file_nameU = aff_path + affordance_name + "_" + object_name + "_point_count.dat";
    std::ifstream ifs_agglomeratorU ( file_nameU );
    pcl::loadBinary( agglomeratorU->data_individual, ifs_agglomeratorU );
    
    
    
    std::string file_nameW;
    std::vector<float> pv_norms_W;
    std::vector<float> pv_norms_mappedW;
    std::string base_nameW;
   
    base_nameW = aff_path + "New_"+ affordance_name + "_" + object_name + "_descriptor_" + std::to_string(numOrientations);
        
    file_nameW = base_nameW + "_vdata.pcd";
    pcl::PointCloud<pcl::PointXYZ>::Ptr vectors_data_cloudW(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile(file_nameW.c_str(), *vectors_data_cloudW);
    
    pv_norms_W.reserve(sampleSize);
    pv_norms_mappedW.reserve(sampleSize);
    
    for( int i = 0 ; i < sampleSize ; i++){
        pv_norms_W.push_back( vectors_data_cloudW->at(i).x );
        pv_norms_mappedW.push_back( vectors_data_cloudW->at(i).y );
    }
    
    
    Agglomerator_IT *agglomeratorW = new Agglomerator_IT( itcalculator.spinnedVectorsW, itcalculator.spinnedDescriptorW, pv_norms_W, pv_norms_mappedW);
       

    file_nameW=base_nameW+"_members.pcd";
    pcl::io::loadPCDFile(file_nameW.c_str(),*agglomeratorW->aux_cloud);
    
    file_nameW=base_nameW+"_extra.pcd";
    pcl::io::loadPCDFile(file_nameW.c_str(),*agglomeratorW->useful_cloud);
    
    file_nameW=base_nameW+".pcd";
    pcl::io::loadPCDFile(file_nameW.c_str(),*agglomeratorW->better_approx);
    
    file_nameW=base_nameW+"_points.pcd";
    pcl::io::loadPCDFile(file_nameW.c_str(),*agglomeratorW->bare_points);
    
    file_nameW=base_nameW+"_vectors.pcd";
    pcl::io::loadPCDFile(file_nameW.c_str(),*agglomeratorW->vector_ids_agglomerative);
   
    agglomeratorW->vectors_data_cloud = vectors_data_cloudW;
    
    file_nameW = aff_path + affordance_name + "_" + object_name + "_point_count.dat";
    std::ifstream ifs_agglomeratorW ( file_nameW );
    pcl::loadBinary( agglomeratorW->data_individual, ifs_agglomeratorW );
    
    //TODO it is necessary save number of orientations and sample size in an independet way
    
    itcalculator.agglomeratorU = agglomeratorU;
    itcalculator.agglomeratorW =agglomeratorW;
    
    return itcalculator;
}


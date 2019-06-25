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
            affordance_names.push_back( affordance_name );
            object_names.push_back( object_name );
            aff_paths.push_back( aff_path );
  
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


std::string IT_Testing::testInteractions(std::string scn_name, pcl::PointCloud<pcl::PointXYZ>::Ptr whole_scene){
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr spinCloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr largeData;
    Eigen::Matrix <float, Eigen::Dynamic, 3, Eigen::RowMajor> largeVectors;
    Eigen::Matrix <float, Eigen::Dynamic, 3, Eigen::RowMajor> largeDescriptor;
    std::vector<int> ppCentroid;
    std::vector<int> startppCentroid;
    std::vector<float> large_mags;        //provenance vector NORM of the point asociated in the sample
    std::vector<float> large_lengths;     //MAPPED provenance vector NORM to [1, 0]
    float alternative_data_count;        // Se trata del inverso de  la suma de mags mappeadas [1, 0]
    
    spinCloud = this->interactions[0].bare_points;               //_descriptor_8_points.pcd
    largeData = this->interactions[0].useful_cloud;         //_descriptor_8_extra.pcd
    largeVectors = this->interactions[0].largeVectors;
    largeDescriptor = this->interactions[0].descriptor;
    ppCentroid = this->interactions[0].ppCentroid;
    startppCentroid= this->interactions[0].startppCentroid;
    large_mags = this->interactions[0].large_mags;
    large_lengths = this->interactions[0].large_lengths;
    alternative_data_count = this->interactions[0].alternative_data_count;
    
    //these are only used on saving files
    std::string affordance_name = this->affordance_names[0];
    std::string ob_name         = this->object_names[0];
    
    
    
      
    std::cout<<"Creating octree...";
    octree.reset(new pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>(true));
    octree->setResolution( this->object_diags_size[0] * 2 );                                                    //TODO this is object dependend
    octree->setInputCloud (whole_scene);
    octree->defineBoundingBox();
    octree->addPointsFromInputCloud ();
    std::cout<<"done"<<std::endl;
    
/////////////////APO: SAMPLING
    std::cout<<"Sampling...";
    
   
   
   pcl::PointCloud<pcl::PointXYZ>::Ptr sample_scene(new pcl::PointCloud<pcl::PointXYZ>);
    
    if (!this->toTest){
         pcl::RandomSample<pcl::PointXYZ> random_sampler;      //this applies a random sample with uniform probabiliti
        random_sampler.setSample( this->sample_percentage * whole_scene->size() );
        random_sampler.setInputCloud( whole_scene );
        random_sampler.setSeed( unsigned ( std::time(0) ) );
        random_sampler.filter(*sample_scene);
    }
    else 
        pcl::io::loadPCDFile("../../test/testing_data/01_testing_sample.pcd", *sample_scene);
  
    
    //to keep track progress
    int topCount = sample_scene->size();
    
    
    std::cout << "Cloud size   : "<< whole_scene->size() << std::endl;
    std::cout << "Sample point : "<<topCount<<" "<<sample_scene->size()<< std::endl;
    
    
    
////////////////APO: for visualizing
    //PointCloud::Ptr object(new PointCloud);
    //pcl::PolygonMesh::Ptr best_object(new pcl::PolygonMesh);
    
    
/////////////////APO: PREPARE CONTAINERS FOR RESULTS
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr goodData(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr goodPoints(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr sampledPoints(new pcl::PointCloud<pcl::PointXYZ>);
    
    std::vector<float> best_local_angle(1);     //(myTensor.affordance_name.size());  //TODO it is necesary understand how they perform multiple affordance prediction
    std::vector<float> best_angle(1);           //(myTensor.affordance_name.size());
    
    std::vector<float> best_local_score(1,0);   //(myTensor.affordance_name.size(),0);
    std::vector<float> best_score(1,0);         //(myTensor.affordance_name.size(),0);
    
    std::vector<int> success_counter(1,0);           //myTensor.affordance_name.size(),0);
    
    bool goodPoint=false;
    
/////////////////APO: preparing viewer
//BEGIN viwewer
//     if(log_level>0)
//     {
//         // If debugging, show input scene in pointcloud mode to viewer
//         if(log_level>1)
//         {
//                 
//                 myTensor.plot(inputCloud,"windowC",255,255,255,1);
//                 myTensor.viewer->setPosition(0,0);
//                 myTensor.viewer->setShowFPS(false);
//                 myTensor.viewer->addText("PPS 0",50,50,"Rate");
//         }
//         else
//         {
//                 // If not debugging first instantiate the viewer and set some basic settings
//                 myTensor.viewer.reset(new pcl::visualization::PCLVisualizer);
//                 myTensor.viewer->setSize(1280,720);
//                 myTensor.viewer->addCoordinateSystem(.1,"Coord",0);
//                 myTensor.viewer->setPosition(0,0);
//                 myTensor.viewer->setShowFPS(false);
//                 myTensor.viewer->addText("PPS 0",50,50,"Rate");
// 
//                 // Try to find a CAD (PLY/OBJ) model of the input scene, just because it looks nicer
//                 // Otherwise, show the scene pointcloud
//                 // Code searches for the same file name as pointcloud but with
//                 // prefix "display"
//                 scene_name=myTensor.file_name+myTensor.scn_name+".ply";
//                 std::cout<<"Looking for "<<scene_name<<std::endl;
//                 if(fexists(scene_name))
//                 {
//                         std::cout<<"Nice display file found: "<<scene_name<<std::endl;
//     #if VTK_MAJOR_VERSION < 6
//                         myTensor.viewer->addModelFromPLYFile(scene_name.c_str(),"scene",0);
//     #else
//                         pcl::PolygonMesh mesh;
//                         pcl::io::loadPolygonFilePLY(scene_name.c_str(),mesh);
//                         if(argc==2)
//                                 myTensor.viewer->addPolygonMesh(mesh,scene_name.c_str(),0);
//                         else
//                                 myTensor.viewer->addPolygonMesh(mesh,scene_name.c_str(),0);
//     #endif
//                 }
//                 else
//                 {
//                         std::string scene_name=myTensor.file_name+myTensor.scn_name+".obj";
//                         if(fexists(scene_name))
//                         {
//                                 pcl::PolygonMesh mesh;
//                                 pcl::io::loadOBJFile(scene_name.c_str(),mesh);
//                                 std::cout<<"Nice display file found: "<<scene_name<<std::endl;
//                                 myTensor.viewer->addPolygonMesh(mesh,scene_name.c_str(),0);
//                         }
//                         else
//                         {       
//                                 // Load input pointcloud into viewer if CAD model was not found
//                                 std::cout<<"Nice display file not found. Loading point cloud"<<std::endl;
//                                 myTensor.plot(inputCloud,"windowC",255,255,255,1);
// 
//                         }
//                 }
//         }
//         for(int k=0;k<colours.size();k++)
//         {
//             std::stringstream count;
//             count << success_counter.at(k);
//             std::string window_text=myTensor.affordance_name.at(k)+" "+count.str();
//             myTensor.viewer->addText(window_text.c_str(),100,200-k*20,20.0,double(colours.at(k).r)/255,double(colours.at(k).g)/255,double(colours.at(k).b)/255,myTensor.affordance_name.at(k).c_str(),0);
//             count.str(std::string());
//         }
//         // Show and wait for input
//         while(!myTensor.viewer->wasStopped())
//         {
//                 myTensor.viewer->spinOnce();
//         }
//         myTensor.viewer->resetStoppedFlag();
//     }
//END viwewer
    
/////////////////APO: SETUP GPU
    
    //Print to info regarding files/descriptor loaded from disk
    std::cout<<"Large vectors: "<<largeVectors.rows()<<"x"<<largeVectors.cols()<<" mags: "<<large_mags.size()<<" lenghts: "<<large_lengths.size()<<std::endl;
    
    // Set GPU/cuda stuff 

    // If more than 1 GPU  you can set it here
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
    // Useful for multiple affordance, for single affordance this
    // is a 2048 array full on 1's
    dev_array<int> d_ppCentroid(largeDescriptor.rows());
    d_ppCentroid.set(&ppCentroid[0],largeDescriptor.rows());
    // Useful for multiple affordance, for single affordance this
    // is a 2048 with the cumsum of ppCentroid 
    dev_array<int> d_startppCentroid(largeDescriptor.rows());
    d_startppCentroid.set(&startppCentroid[0],largeDescriptor.rows());
    //Times 4 because pcl alignment x,y,z + extra byte
    // This has data for orientation, keypoint and affordance id
    dev_array<float> d_ppCData(largeData->size()*4);
    d_ppCData.set(&largeData->at(0).getArray3fMap()[0],largeData->size()*4);
    
    

    // GPU stuff in PCL
    // Container for indices in NN-search. Same size as descriptor since 1-NN for every
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
    // clock to compute "frame rate" or points tested per second
    std::clock_t ppsecond;
    
    ppsecond = std::clock();
    
    int points_tested=0;
    int max_cloud=0;
    // Test point counter
    int i=0;
    int progress = 0;
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr auxCloud;    // Clear the pointcloud that will store the voxel extracted from scene
    
    auxCloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    

    while(i < topCount)
    {

        
    /////////////////APO: GET A SAMPLE POINT
        
        // the current test point sampled from scene
        pcl::PointXYZ sampledPoint;
        sampledPoint=sample_scene->at(i);
        // Save all testpoints sampled
        sampledPoints->push_back(sampledPoint);                     /////////////////APO: this variable is not used
        // Some functions expec data as vector or array, so make a copy
        // of the sampled point in different containers. Surely there's a better way to do this
        float testPoint[3] = {sampledPoint.x,sampledPoint.y,sampledPoint.z}; //APO 20190615: en realidad debía ser del tipo dev_array<float>
        Eigen::Vector3f asp(sampledPoint.x,sampledPoint.y,sampledPoint.z); //APO 20190613: sampledPoint.getArray3fMap();

        //Just some info about total progress
        int current_percent = int (100*i/topCount);

        if( current_percent-progress > 0)
        {
            progress = current_percent;
            //BEGIN viwewer

//             if(log_level>1)
//             {
//                 std::cout<<"\n+++++++++   "<<current_percent<<"   ++++++++++"<<std::endl;
//             }
//             else
//             {
            //END viwewer
                std::cout<<" "<<current_percent<<std::endl;//std::flush;
            //BEGIN viwewer
//             }
            
             //END viwewer
            
        }
         //BEGIN viwewer
//         else
//         {
//             //std::cout<<" ";
//             std::cout<<std::flush;
//         }
         //END viwewer

        // Only best_point will be shown at run time
        std::vector<pcl::PointXYZ> best_point(1);                                //myTensor.affordance_name.size());
        // Should a "best point" be found then need to update viewer
        std::vector<bool> update(1,false);                                      //myTensor.affordance_name.size(),false);

        //BEGIN viwewer
//         // If showing viewer, plot a line at the current test-point
//         // just as visual aid
//         if( log_level > 0 )
//         {
//             if(myTensor.viewer->contains("test"))
//             {
//                 myTensor.viewer->removeShape("test");
//             }
//             myTensor.viewer->addLine(sampledPoint,pcl::PointXYZ(sampledPoint.x,sampledPoint.y,sampledPoint.z+(object_diags_size[0]*.5)),1,0,0,"test");
//             myTensor.viewer->spinOnce();
//         }
        //END viwewer
        // In some pointclouds there were points with coordinates 0,0,0
        // ignore those
        // I think this is a bug during the data capturing in real scenes
        if(sampledPoint.x==0 && sampledPoint.y==0 && sampledPoint.z==0) 
        continue;
        // This is a copy of the descriptor that will be
        // moving around tom compute scores
        Eigen::MatrixXf moving_descriptor(largeDescriptor.rows(),3);
        // Move or transform the descriptor relative to the current test-point
        //                 std::cout << largeDescriptor<<endl<<"----"<< asp<<endl<<"----------"<<asp.transpose()<< endl;
        moving_descriptor = largeDescriptor.rowwise() + asp.transpose();
        //BEGIN viwewer
//         if(log_level>1)
//         {
//             std::cout<<"Descriptor size "<<moving_descriptor.rows()<<std::endl;
//         }
        //END viwewer
        // Copy the new descriptor data as pointcloud.
        // I think PCL has a way to map the memory from a Eigen Matrix to a pointcloud structure
        // but haven't found it. So do it manually
        
        pcl::PointCloud <pcl::PointXYZ>::Ptr moving_spinCloud;
        
        moving_spinCloud = Util_iT::copyAsPointCloud(&moving_descriptor);
        

    /////////////////APO: EXTRACT A VOXEL AROUND THE SAMPLE POINT

        /*
        Will do a NN-search to extract a voxel from the scene pointcloud
        around the current test-point
        */
        
        // Containers for NN search: point indices and distances
        std::vector<int> pointIdxVec;
        std::vector<float> pointRadiusSquaredDistance;
        
        //BEGIN TIMER
        // If debugging, count time to extract voxel
//         if(log_level>1)
//         {
//             tt.tic();
//         }
        //END TIMER
        
        bool voxelOutcome=false;

        // Clear the pointcloud that will store the voxel extracted from scene
        auxCloud->clear();

        /* PointcloudType;
        0 -> Synthetic PLY scene with access to polygons and vertices data
        Use the middle point in polygons to extract voxel

        1 -> PCD file with only vertices or point data
        */
        
        //BEGIN TYPEPCL
        // This search is done in CPU. No important difference was notice when using GPU-version
//         if(myTensor.pointCloudType==0)
//         {
//             // Notice the search rad is half size of "desired size" which is diagonal of query-object bouding box
//             voxelOutcome=myTensor.octreeCentres->radiusSearch(sampledPoint,0.5*object_diags_size[0],pointIdxVec,pointRadiusSquaredDistance);
//             // Voxel should have at leats some points, this could change
//             // acording to descriptor size
//             if(pointIdxVec.size()<2)
//             {
//                 if(log_level>1)
//                 {
//                     std::cout<<"Not enough points in scene voxel "<<pointIdxVec.size()<<std::endl;
//                 }
//                 // Free gpu memory and continue sampling test-points
//                 cloud_device2.release();
//                 result_device.data.release();
//                 continue;
//             }
//             // If enough points in voxel: extract and store in auxCloud
//             Tensor::extractCloud(pointIdxVec,myTensor.cellCentres,auxCloud);
//         }
//         else
//         {
        //END TYPEPCL
            // Same for PCD scene
            voxelOutcome = octree->radiusSearch(sampledPoint, 0.5*object_diags_size[0], pointIdxVec,pointRadiusSquaredDistance );
            if( pointIdxVec.size() < 2)
            {
                cloud_device2.release();
                result_device.data.release();
                continue;
            }
            Util_iT::extractCloud(pointIdxVec,whole_scene,auxCloud);
         
        //BEGIN TYPEPCL   
//         }
        //END TYPEPCL

        //BEGIN TIMER
//         if(log_level>1)
//         {
//                 std::cout<<i<<" "<<auxCloud->size()<<" cloud extraction ["<<tt.toc()<<" ms]"<<std::endl;
//         }
        //END TIMER

  

    /////////////////APO: GPU search affordance keypoint in the environment by searching the neares point in voxel from the "moved" origins of the provenances vectors

        // With the current voxel perform 1-NN search for affordance keypoints
        // these will be used to estimate test-vectors
        // Upload voxel to GPU and build octree
        cloud_device2.upload(auxCloud->points);
        octree_device2.setCloud(cloud_device2);
        if(log_level>1)
        std::cout<<"Building tree in GPU"<<std::endl;
        octree_device2.build();
        if(log_level>1)
        std::cout<<"...done"<<std::endl;

        // Upload descriptor (queries) to GPU
        queries_device.upload(moving_spinCloud->points);

        //BEGIN TIMER
        // If debugging, take the time it takes to do NN search and get data
//         if(log_level>1) 
//         {
//             tt.tic();
//         }
        //END TIMER
        // Perform 1-NN search for affordance keypoints in descriptor
        octree_device2.nearestKSearchBatch(queries_device, 1, result_device);
        // Download data to PC memory, point indices are returned
        std::vector<int> downloaded;
        result_device.data.download(downloaded);

        //BEGIN TIMER
//         if(log_level>1){
//                 std::cout<<"NN search ["<<tt.toc()<<" ms]"<<std::endl;
//         }
        //END TIMER

        // Copy NN data to a pointcloud structure, there could be
        // repeated indices
        pcl::PointCloud<pcl::PointXYZ>::Ptr local_nn(new pcl::PointCloud<pcl::PointXYZ>);;
        for(int i=0;i<downloaded.size();i++)
        {
            local_nn->push_back( auxCloud->at( downloaded.at(i) ) );
        }

    /////////////////APO: GPU Compute scores with a kernel
        // Estimate test vectors and compute scores
       //BEGIN TIMER
       // in gpu with CUDA. If debugging, take the time
//         std::clock_t tscore;
//         if(log_level>1)
//         {
//             tscore = std::clock();
//         }
        //END TIMER

        //              local_nn                ==> nearest point in scene from a given provenance vector
        //              testPoint               ==> sample point to test
        //              d_kpData                ==> key point to test, 512 * 8 (samples calculated and rotated)
        //              d_pvData                ==> provenance vectors repited as many as orientations
        //              d_weights               ==> provenance vector norms mapped, the smaller vector the bigger value in map [1,0]
        //              d_mags                  ==> provenance vector norms
        //              d_ppCentroid            ==> it only have "1" as value, possibly to detected multiple affordances in future
        //              d_startppCentroid       ==> index of position 0-4095, possibly to detected multiple affordances in future
        //              d_ppCData               ==> 4 values: affordance id, orientation, #ofKeypoint, and another value to align
        //              &myTensor               ==> all data loades (is this neccessary?)
        std::vector<float> score=computeScore(local_nn,testPoint,d_kpData,d_pvData,d_weights,d_mags,d_ppCentroid,d_startppCentroid,d_ppCData);
        
        //BEGIN TIMER
//         if(log_level>1)
//         {
//             std::clock_t tscoref = std::clock();
//             std::cout<<"GPU vector comparison ["<<1000*( tscoref - tscore ) / (double) CLOCKS_PER_SEC<<" ms]"<<std::endl;
//         }
        //END TIMER
        
        std::vector<float> max;
        std::vector<int> ids,orientations;
        // If showing viewer
        //BEGIN viwewer      
//         if(log_level>0)
//         {
//             // Get the location/orientation that scores higher than prediction Thershold
//             // This only gets the top orientation/score to show in viewer
//             std::cout << myTensor.alternative_data_count << endl;   /////////////////////////////////APO 20190612: no entiendo el valor de esta variable
//             int idMax=getMinMax(score,max,orientations,ids,myTensor.alternative_data_count,largeData);
// 
//             // Increment the number of test-points actually tested
//             // and compute the "points per second" stats
//             points_tested+=1;
//             std::clock_t ppsecond2 = std::clock();
//             double timeInSeconds=1000*( ppsecond2 - ppsecond ) / (double) CLOCKS_PER_SEC;
//             if(timeInSeconds>=1000)
//             {
//                 //updateText in viewer
//                 std::stringstream points_teste_text;
//                 points_teste_text << points_tested;
//                 std::string point_rate="PPS "+points_teste_text.str();
//                 myTensor.viewer->updateText(point_rate.c_str(),50,50,"Rate");
//                 myTensor.viewer->spinOnce();
//                 points_tested=0;
//                 ppsecond=ppsecond2;
//             }
// 
//             // If idMax is >0 then actually found something good
//             // Otherwilse keep searching
//             int idSuccess=idMax>=0?ids.at(idMax):-1;
//             if(ids.size()>0)
//             {
//                 // Save some data about the good location
//                 // coordinates
//                 pcl::PointXYZRGB agp(0,0,0);
//                 agp.x=sampledPoint.x;
//                 agp.y=sampledPoint.y;
//                 agp.z=sampledPoint.z;
//                 // Enconde in colors some other data
//                 // red is how many good matches (affordances x orientations) in this point
//                 agp.r=ids.size();
//                 goodPoints->push_back(agp);
//                 // For single affordance case this loop does not make sense
//                 // but will keep it for future release (multiple affordances)
//                 for(int sId=0;sId<ids.size();sId++)
//                 {
// 
//                     int anId=ids.at(sId);
//                     pcl::PointXYZRGB datap(0,0,0);
//                     //x is affordance id;
//                     datap.x=anId;
//                     //y is orientation id;
//                     datap.y=orientations.at(sId);
//                     //z is score;
//                     datap.z=max.at(sId);
// 
//                     // save data in pointcloud structure
//                     goodData->push_back(datap);
// 
//                     //keep track how many good predictions
//                     success_counter.at(anId)+=1;
// 
//                     // If current result is the best
//                     // save it and update viewer (flag)
//                     if(max.at(sId)>best_score.at(anId))
//                     {
//                         best_score.at(anId)=max.at(sId);
//                         best_angle.at(anId)=2*orientations.at(sId)*M_PI/O;
//                         best_point.at(anId)=sampledPoint;
//                         update.at(anId)=true;
//                     }
//                     // update text in viwewer
//                     std::stringstream count,be;
//                     count << success_counter.at(anId);
//                     be<<best_score.at(anId);
//                     std::string window_text=" "+be.str()+" "+myTensor.affordance_name.at(anId)+" "+count.str();
//                     myTensor.viewer->updateText(window_text.c_str(),100,200-anId*20,myTensor.affordance_name.at(anId).c_str());
// 
//                     best_local_anle.at(anId)=2*orientations.at(sId)*M_PI/O;
//                     best_local_score.at(anId)=max.at(sId);
// 
//                     if(update.at(anId))
//                     {
//                         // Set everything for viewer
// 
//                         std::string object_cloud_name="best_"+myTensor.affordance_name.at(anId)+"_"+myTensor.ob_names.at(anId);
// 
//                         // Transformations needed for query-object cloud/meshes
//                         Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
//                         transform_2.translation() << 0.0, 0.0, 0.0;
//                         transform_2.rotate (Eigen::AngleAxisf (best_angle.at(anId), Eigen::Vector3f::UnitZ()));
//                         pcl::PointXYZ rotToObj=pcl::transformPoint(myTensor.toObject->at(anId),transform_2);
//                         pcl::PointXYZ obRef(sampledPoint.x+rotToObj.x,sampledPoint.y+rotToObj.y,sampledPoint.z+rotToObj.z);
//                         pcl::fromPCLPointCloud2(myTensor.object_meshes.at(anId).cloud,*object);
//                         Tensor::rotateCloud(object,object,best_angle.at(anId),'z',myTensor.original_points->at(anId));
//                         Tensor::translateCloud(object,object,obRef,myTensor.original_points->at(anId));
//                         // Update the query-object mesh                                 
//                         pcl::PCLPointCloud2 vertices;
//                         pcl::toPCLPointCloud2(*object,vertices);
//                         pcl::copyPointCloud(vertices,best_object->cloud);
//                         best_object->polygons.resize(myTensor.object_meshes.at(anId).polygons.size());
//                         std::copy(myTensor.object_meshes.at(anId).polygons.begin(),myTensor.object_meshes.at(anId).polygons.end(),best_object->polygons.begin());
// 
//                         // Actually plot the query-objet in new location
//                         myTensor.plot(*best_object,object_cloud_name.c_str(),colours.at(anId).r,colours.at(anId).g,colours.at(anId).b);
// 
//                         // If debugging plot and wait
//                         if(log_level>1 && update.at(anId))
//                         {
//                             while(!myTensor.viewer->wasStopped())
//                             {
//                                 myTensor.viewer->spinOnce();
//                             }
//                             
//                             myTensor.viewer->resetStoppedFlag();
// 
//                         }
//                         else
//                         {
//                             // if not plot and continue
//                             myTensor.viewer->spinOnce();
//                         }
//                     }
//                     // Flip the update viewer flag
//                     if(update.at(anId))update.at(anId)=false;
//                 }
//             }
//         }
//         else
//         {
        //END viwewer        
        /////////////////APO: analysis of scores computed
        
            // If not showing viewer 
            // compute and get every orientation that is predicted as good
            //std::cout << alternative_data_count << std::endl;   /////////////////////////////////APO 20190612: 1/sumMappedNorms for made a direct measure of proportions
            int thisManyAffordances=getMinMaxMulti(score, max, orientations, ids, alternative_data_count, largeData, goodData);
            // If actually found something save everything
            if(thisManyAffordances>0)
            {
                //save datapoint with info
                pcl::PointXYZRGB agp(0,0,0);
                agp.x=sampledPoint.x;
                agp.y=sampledPoint.y;
                agp.z=sampledPoint.z;
                //red is how many good matches (affordances x orientations) in this point
                // we can use 3 8-bit integers if needed
                // All other data such as scores, orientations and ids are store
                // inside the getMinMaxMulti function
                if(ids.size()>255)
                {
                    agp.r=255;
                    if(ids.size()>510)
                    {
                        agp.g=255;
                        agp.b=ids.size()-510;
                    }
                    else
                    {
                        agp.g=ids.size()-255;
                    }
                }
                else
                {
                    agp.r=ids.size();
                }
                goodPoints->push_back(agp);
            }
           //BEGIN viwewer
//         }
           //END viwewer
        // Free GPU memory
        cloud_device2.release();
        result_device.data.release();

        // get the next test-point
        i+=1;
    }
    std::cout<<" 100"<<std::endl;

    //BEGIN viwewer
//     if(log_level>0)
//     {
//         //spin and exit
//         myTensor.viewer->resetStoppedFlag();
//         while(!myTensor.viewer->wasStopped())
//         {
//             myTensor.viewer->spinOnce(100);
//         }
//     }
    //END viwewer
    if(goodPoints->size()>0)
    {
            //myTensor.converged=true;
            std::cout<<std::endl<<std::endl<<" ========= Success =========="<<std::endl;
            
    }
    else
    {
            std::cout<<std::endl<<std::endl<<" ========= Finish with no success =========="<<std::endl;
    }
    
    std::string fileID= saveClouds( scn_name, affordance_name, ob_name, sample_scene, goodData, goodPoints );
    std::cout<<"Good places: "<<goodPoints->size()<<"\n";

    return fileID;
}

// Estimate test vectors and compute score using cuda
//              local_nn       ==> nearest point in scene from a given provenance vector
//              samplePoint    ==> sample point to test
//              kpData         ==> key point to tests, 512 * 8 (samples calculated and rotated)
//              pvData         ==> provenance vectors repited as many as orientations
//              wData          ==> provenance vector norms mapped, the smaller vector the bigger value in map [1,0]
//              mData          ==> provenance vector norms
//              ppC            ==> it only have "1" as value, possibly to detected multiple affordances in future
//              startppC       ==> index of position 0-4095, possibly to detected multiple affordances in future
//              ppCData        ==> affordance id, orientation, #ofKeypoint
//              &object        ==> all data loades (is this neccessary?)
std::vector< float > IT_Testing::computeScore(pcl::PointCloud< pcl::PointXYZ >::Ptr local_nn, float* samplePoint, dev_array< float >& kpData, dev_array< float >& pvData, dev_array< float >& wData, 
                                              dev_array< float >& mData, dev_array< int >& ppC, dev_array< int >& startppC, dev_array< float >& ppCData)
{
        
        //Get the size of the result container
        int inner_N = kpData.getSize()/3; //object->largeVectors.rows();;
        // This is max for my GPU, can/should be modified
        int maxThreads=2048;
        // Allocate memory on the device
        //Result containers in CPU and GPU memory
        std::vector<float> h_C(inner_N,0);
        dev_array<float> d_C(inner_N);
        d_C.set(&h_C[0],inner_N);
        // NN at test point:  same size as descriptor or neightbours cloud
        dev_array<float> d_nnData(local_nn->size()*4);  
        // NN data
        d_nnData.set(&local_nn->at(0).getArray3fMap()[0], local_nn->size()*4); 
        // Test point: 1 point x 3 components
        dev_array<float> d_testPoint(3);  
        // Current test-point
        d_testPoint.set( samplePoint, 3);
        // Need to copy batches of maxThreads to GPU
        int thisManyBatches=std::ceil(double(local_nn->size())/double(maxThreads));  //Possible BUG in batch computation!
        
        //BEGIN GPU 
        //if(object->plotDebug>1)
//                 std::cout<<"Sending "<<thisManyBatches<<" batches to gpu"<<std::endl;
        //END GPU 
        
        for (int i=0;i<thisManyBatches;i++)
        {
                int start=i*maxThreads;
                int end=start+maxThreads-1;
                if(end>local_nn->size())
                        end=local_nn->size()-1;
                // Call CUDA to compute scores
                //      d_nnData        ==> array of nearest point in scene from a given  provenance vector
                //      kpData          ==> key point to tests, 512 * 8 (samples calculated and rotated)
                //      pvData          ==> provenance vectors repited as many as orientations
                //      d_testPoint     ==> testPoint
                //      d_C             ==> OUTPUT this will be the result container 
                //      start           ==> first number of sample point to test in the batch
                //      end             ==> las number of sample point  to test in the batch
                //      wData           ==> provenance vector norms mapped, the smaller vector the bigger value in map [1,0]
                //      mData           ==> provenance vector norms
                //      object->agg_th  ==> threshold read from parameters.txt (VectorDiff)
                //      ppC            ==> it only have "1" as value, possibly to detected multiple affordances in future
                //      startppC       ==> index of position 0-4095 (asociated with ppC), possibly to detected multiple affordances in future
                //      ppCData        ==> affordance id, orientation, #ofKeypoint
                bayesian_scores(d_nnData.getData(),kpData.getData(),pvData.getData(),d_testPoint.getData(),d_C.getData(),start,end,4,wData.getData(),mData.getData(),this->agg_th,ppC.getData(),startppC.getData(),ppCData.getData());
        }
        // This a long flat vector with individual keypoint scores
        // That is read in getMinMax function
        d_C.get(&h_C[0],inner_N);
        return h_C;
}



// Get all the good predictions per test-point. Used when text-only mode is enabled.
//                      score        => bigScores,    the calculated score in GPU
//                      max          => score,        output variable with the scores analyzed
//                      orientations => orentations,  output orientations asociated with the ourput score
//                     ids           => aff_id,       output affordance id associated with score and orientation
//  myTensor.alternative_data_count => thresholds,   this is the sum of all provenance vectors mapped magnitudes [1,0]
//               largeData  => data          x: 1 , y: #rotation, z: #point in sampling sequence
//                     goodData      => cloudToSave   x is affordances id; y is orientation; z is score;
int IT_Testing::getMinMaxMulti(std::vector< float >& bigScores, std::vector< float >& score, std::vector< int >& orientation, std::vector< int >& aff_id, float thresholds, pcl::PointCloud< pcl::PointXYZ >::Ptr data, pcl::PointCloud< pcl::PointXYZRGB >::Ptr cloudToSave)
{
    //         std::cout << thresholds << std::endl;
    // Matrix with (point_counts)^-1 to normalize 
    Eigen::MatrixXf results=Eigen::MatrixXf::Zero(1,8); //thresholds.rows(),8); TODO this seems to be for having multiple affordance tests
    
    for(int i=0;i<data->size();i++)
    {
        int row=0;  //Affordance
//         if(thresholds.rows()>1)   //TODO this seems to be for having multiple affordance tests
//            row=int(data->at(i).x)-1;
        int col=int(data->at(i).y); //Orientation
        results(row,col)+=bigScores.at(i);
    }
    
    // Normalize scores
    Eigen::MatrixXf results_scores=results * thresholds; //.cwiseProduct(thresholds);TODO this seems to be for having multiple affordance tests
    // At this point everything is in [0,1]
    // Check every score and save those >= prediction thershold
    int output=0;
    for(int i=0;i<results_scores.rows();i++)
    {
        int aff_counter=0;
        for(int j=0;j<results_scores.cols();j++)
        {
            if((results_scores(i,j)-pred_t)>0)  //good
            {
                score.push_back(results_scores(i,j));
                aff_id.push_back(i);
                //        if(i==3 && (j==2 || j==6))
                //          std::cout<<"HERE!!"<<std::endl;
                orientation.push_back(j);
                aff_counter+=1;
                pcl::PointXYZRGB datap(0,0,0);
                //x is affordances id;
                datap.x=i;
                //y is orientation;
                datap.y=j;
                //z is score;
                datap.z=results_scores(i,j);
                cloudToSave->push_back(datap);
            }
        }
        if(aff_counter>0)
        {
                output+=1;
        }
    }
    return output;
}



std::string IT_Testing::saveClouds( std::string scn_name, std::string affordance_name, std::string ob_name, 
                                    pcl::PointCloud<pcl::PointXYZ>::Ptr sample,  pcl::PointCloud< pcl::PointXYZRGB >::Ptr points,  pcl::PointCloud< pcl::PointXYZRGB >::Ptr data )
{
    //Save data of last run
    std::stringstream ss;
    std::string file_name;
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr sampleColor(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr saveCloudC(new pcl::PointCloud<pcl::PointXYZRGB>);
   
    
    std::string now = boost::lexical_cast<std::string>(time(0));
    std::string output_path = path_data+"/output/";
    
    if (!boost::filesystem::exists( output_path ) )
    {
        boost::filesystem::path dir(output_path);
        boost::filesystem::create_directory(dir);
    }
    
    if (data->size() > 0 )
    {
       
        pcl::copyPointCloud(*sample,*sampleColor);
        
        for(int i=0;i<sampleColor->size();i++)
        {
            sampleColor->at(i).r=255;
            sampleColor->at(i).g=0;
            sampleColor->at(i).b=255;
        }
        
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr goodColor(new pcl::PointCloud<pcl::PointXYZRGB>);
        
        pcl::copyPointCloud(*points,*goodColor);
        
        for(int i=0;i<goodColor->size();i++)
        {
            goodColor->at(i).r=255;
            goodColor->at(i).g=0;
            goodColor->at(i).b=0;
        }
        //File name code: affordance_object_scene_method_sampleSize.pcd
        
        
        *saveCloudC+=*sampleColor;
        *saveCloudC+=*goodColor;
        
        file_name = output_path + now + "_" + affordance_name + "_" + ob_name + "_" + scn_name + ".pcd";
        std::cout << "File name example: " << file_name << std::endl;
        pcl::io::savePCDFileBinaryCompressed(file_name,*saveCloudC);
        
        file_name = output_path + now + "_samplePoints.pcd";
        pcl::io::savePCDFileBinaryCompressed(file_name,*sampleColor);
        
        file_name = output_path + now + "_goodPoints.pcd";
        pcl::io::savePCDFileBinaryCompressed(file_name,*data);
        
        // goodPointsX.pcd has number predictions per test-point, their score and orientations
        file_name = output_path + now + "_goodPointsX.pcd";
        pcl::io::savePCDFileBinaryCompressed(file_name,*points);
    }
    else
    {
        //If did not find anything
        
        file_name = output_path + now + "_" + affordance_name + "_" + ob_name + "_NoConv_" + scn_name + "_" + ".pcd";
        std::cout << "File name example: " << file_name << std::endl;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr no_points(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::io::savePCDFileBinaryCompressed(file_name,*no_points);
        
        
        
        pcl::copyPointCloud(*sample,*sampleColor);
        for(int i=0;i<sampleColor->size();i++)
        {
            sampleColor->at(i).r=255;
            sampleColor->at(i).g=0;
            sampleColor->at(i).b=255;
        }
        file_name = output_path + now + "_samplePoints.pcd";
        pcl::io::savePCDFileBinaryCompressed(file_name,*sampleColor);
    }
    return now;
}

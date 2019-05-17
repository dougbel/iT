#include "spinner_it.h"

Spinner_iT::Spinner_iT(pcl::PointCloud<PointWithVector>::Ptr sample, int orientations, pcl::PointCloud<pcl::PointXYZ>::Ptr full):sample(sample),full(full),orientations(orientations)
{

}



void Spinner_iT::calculateSpinings(){
    
    PointCloudT::Ptr relativePoints(new PointCloud);
    
    
    //get reference point in the ibs
    PointWithVector p=sample->at(sample->size()-2);
    pcl::PointXYZ refPointIBS(p.x,p.y,p.z); //vector from reference pointin scene to reference point in IBS
    int refPointIBSId=int(p.v1);   //index of reference point in IBS
    
    //this is SCENE reference point
    pcl::PointXYZ ref(full->at(refPointIBSId).x-refPointIBS.x,full->at(refPointIBSId).y-refPointIBS.y,full->at(refPointIBSId).z-refPointIBS.z);
    

    sample->erase( sample->end()-1 );//erase reference to the relation scene-object
    sample->erase( sample->end()-1 );//erase reference to the relation scene-ibs
    
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
    //BEGIN
    //PointCloudC::Ptr anchor(new PointCloudC);
    //pcl::PointXYZRGB coloredanchor(0,255,0);
    //coloredanchor.x=ref.x;
    //coloredanchor.y=ref.y;
    //coloredanchor.z=ref.z;
    //anchor->push_back(coloredanchor);
    //viewer->addPointCloud(anchor,"Anchor");
    //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5,"Anchor");
    //END
    PointCloud::Ptr spinCloud(new PointCloud);
    PointCloud::Ptr relative_spin(new PointCloud);
    PointCloud::Ptr xyz_2(new PointCloud);
    pcl::copyPointCloud(*sample,*xyz_target);
    pcl::copyPointCloud(*sample,*spinCloud);
    
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
        // a) rotates sample points to the next orientation an save in xyz_target
        pcl::copyPointCloud(*sample,*xyz_2);
        rotateCloud( xyz_2, xyz_target, i*2*M_PI/orientations, 'z', ref );
        *spinCloud+=*xyz_target; //se agrega el resultado de la rotación
        
        
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
    for(int i=0;i<spinCloud->size();i++){
        descriptor.row(i)=Eigen::Vector3f(relative_spin->at(i).x,relative_spin->at(i).y,relative_spin->at(i).z);
        
        //if( (spinCloud->at(i).x-ref.x) != relative_spin->at(i).x  ||  (spinCloud->at(i).y-ref.y) != relative_spin->at(i).y  ||  (spinCloud->at(i).z-ref.z) != relative_spin->at(i).z )  
        //    std::cout << "x " << (spinCloud->at(i).x-ref.x) << ", " << relative_spin->at(i).x << "  y " << (spinCloud->at(i).y-ref.y) << ", " << relative_spin->at(i).y << "  z " << (spinCloud->at(i).z-ref.z) << ", " << relative_spin->at(i).z << std::endl;
        
    }
    std::cout<<"Descriptor "<<descriptor.rows()<<std::endl;
    
}


//TODO origin is always setted as true some sections in this code is unreachable
void Spinner_iT::rotateCloud( pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out, float angle, char axis,bool origin){
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


void Spinner_iT::rotateCloud( pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out, float angle, char axis,pcl::PointXYZ pivot){
    
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    //Eigen::Vector4f centroid (Eigen::Vector4f::Zero());
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

//TODO Erase me, this way of calculating translations is not necesary BECAUSE A ROTATION WITH RESPECET TO THE CENTROID IS NEVER USED
void Spinner_iT::translateCloud( pcl::PointCloud<pcl::PointXYZ>::Ptr in, pcl::PointCloud<pcl::PointXYZ>::Ptr out, pcl::PointXYZ translation){//TODO  no comprendo por que se toma como referencia el cetroide de la nube de puntos
    
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*in,centroid);
    
    pcl::PointXYZ reference;
    reference.getVector4fMap() = centroid;
    
    translateCloud( in,  out,  translation,  reference);
    
//     transform.translation() << translation.x-centroid[0],translation.y-centroid[1],translation.z-centroid[2];
//     transform.rotate (Eigen::AngleAxisf (0, Eigen::Vector3f::UnitZ()));
//     pcl::transformPointCloud (*in, *out, transform);
}

     
void Spinner_iT::translateCloud( pcl::PointCloud<pcl::PointXYZ>::Ptr in, pcl::PointCloud<pcl::PointXYZ>::Ptr out, pcl::PointXYZ translation, pcl::PointXYZ reference){
    
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    
    transform.translation() << translation.x-reference.x,translation.y-reference.y,translation.z-reference.z;
    transform.rotate (Eigen::AngleAxisf (0, Eigen::Vector3f::UnitZ()));
    pcl::transformPointCloud (*in, *out, transform);
}

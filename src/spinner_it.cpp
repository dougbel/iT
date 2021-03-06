#include "spinner_it.h"

Spinner_iT::Spinner_iT(pcl::PointCloud<PointWithVector>::Ptr sample, pcl::PointXYZ spiningPoint, int orientations)
{
    this->sample       = sample;
    this->spiningPoint = spiningPoint;
    this->orientations = orientations;
}



void Spinner_iT::calculateSpinings(){
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr relativePoints(new pcl::PointCloud<pcl::PointXYZ>);
    
    descriptor.resize(sample->size()*orientations,3);
    vectors.resize(sample->size(),3);
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_target(new pcl::PointCloud<pcl::PointXYZ>);
    //std::cout<<"REf: "<<spiningPoint<<std::endl;
    for(int j=0;j<sample->size();j++)
    {
        PointWithVector aP;
        aP.x=sample->at(j).x-spiningPoint.x;
        aP.y=sample->at(j).y-spiningPoint.y;
        aP.z=sample->at(j).z-spiningPoint.z;
        relativePoints->push_back(pcl::PointXYZ(aP.x,aP.y,aP.z));
        //descriptor.row(j)=aP.getVector3fMap();
        Eigen::Vector3f v(sample->at(j).v1,sample->at(j).v2,sample->at(j).v3);
        vectors.row(j)=v;
        //mags.at(j)=(v.norm()-v.norm()*.2)*(v.norm()-v.norm()*.2);
        //lengths.at(j)=1-((v.norm()- minW) * (1 - 0) / (maxW - minW) + 0);
        //xyz_target->push_back(pcl::PointXYZ(aP.x,aP.y,aP.z));
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr spinCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr relative_spin(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_2(new pcl::PointCloud<pcl::PointXYZ>);
    
    pcl::copyPointCloud(*sample,*xyz_target);
    pcl::copyPointCloud(*sample,*spinCloud);
    pcl::copyPointCloud(*relativePoints,*relative_spin);
    
    //pcl::io::savePCDFile("output_spin_orientation_.pcd", *relativePoints);
    
    for (int i=1;i<orientations;i++)
    {   
        // a) rotates sample points to the next orientation an save in xyz_target
        //pcl::copyPointCloud(*sample,*xyz_2);
        //rotateCloud( xyz_2, xyz_target, i*2*M_PI/orientations, 'z', spiningPoint );
        //*spinCloud+=*xyz_target; //se agrega el resultado de la rotación
        
        //b) make rotation with point translate to reference point  //TODO this part is not useful at all
        pcl::copyPointCloud(*relativePoints,*xyz_2);
        rotateCloud(xyz_2,xyz_target, i*2*M_PI/orientations,'z',true);
        *relative_spin+=*xyz_target;
        
        //pcl::io::savePCDFile("output_spin_orientation_"+std::to_string(i)+".pcd", *xyz_target);
        
        //std::cout<<"Spincloud: "<<xyz_target->size()<<std::endl;
        //if(viewer->contains("Spincloud"))
        //            viewer->updatePointCloud(relative_spin,"Spincloud");
        //        else
        //            viewer->addPointCloud(relative_spin,"Spincloud");
        //        while(!viewer->wasStopped())
        //            viewer->spinOnce(100);
        //        viewer->resetStoppedFlag();

    }
    for(int i=0;i<relative_spin->size();i++){
        descriptor.row(i)=Eigen::Vector3f(relative_spin->at(i).x,relative_spin->at(i).y,relative_spin->at(i).z);
    }
    
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
    pcl::PointCloud<pcl::PointXYZ>::iterator iter=cloud_out->end();
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

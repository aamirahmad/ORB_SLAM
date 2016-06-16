/**
* This file is part of ORB-SLAM.
*
* Copyright (C) 2016 Aamir Ahmad <aamir.iitkgp@gmail.com> (max Planck Institute for biological Cybernetics). This code is built on top of Raúl Mur-Artal <raulmur at unizar dot es>'s original ORB-SLAM code
* ORB-SLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM. If not, see <http://www.gnu.org/licenses/>.
*/

#include "FeaturePublisher.h"
#include "MapPoint.h"
#include "KeyFrame.h"

namespace ORB_SLAM
{


FeaturePublisher::FeaturePublisher(int ID): robotID(ID)
{

    //Configure Publisher
    publisher_orb = nh.advertise<std_msgs::Int32MultiArray>("/ORB_SLAM/ORB_Features/robot_"+boost::lexical_cast<string>(robotID), 10);
    publisher_3dPoints = nh.advertise<std_msgs::Int32MultiArray>("/ORB_SLAM/3DPoints_assoc_ORB_Features/robot_"+boost::lexical_cast<string>(robotID), 10);
}

void FeaturePublisher::Refresh()
{

}

void FeaturePublisher::PublishFeatures(cv::Mat& descriptor)
{
    descriptorMatrix = descriptor.clone();

    std_msgs::Int32MultiArray array;
    
    //     # dim[0].label  = "height"
    //     # dim[0].size   = 480
    //     # dim[1].label  = "width"
    //     # dim[1].size   = 640
    
    std_msgs::MultiArrayDimension tempDim;
    
    tempDim.label = "Rows";
    tempDim.size = descriptorMatrix.rows;
    array.layout.dim.push_back(tempDim);
    
    tempDim.label = "Columns";
    tempDim.size = descriptorMatrix.cols;
    array.layout.dim.push_back(tempDim);    

    array.data.clear();
    
    for(int i = 0; i < descriptorMatrix.rows; i++)
    {
      for(int j = 0; j < descriptorMatrix.cols; j++)
      {
	array.data.push_back((int)descriptorMatrix.at<uchar>(j,i));
      }
    }
    publisher_orb.publish(array);
}


void FeaturePublisher::Publish3DPointsAssociated(const std::vector<MapPoint*> &assocMapPoints)
{

    std_msgs::Int32MultiArray array;
    
    //     # dim[0].label  = "height"
    //     # dim[0].size   = 480
    //     # dim[1].label  = "width"
    //     # dim[1].size   = 640
    
    std_msgs::MultiArrayDimension tempDim;
    
    tempDim.label = "Rows";
    tempDim.size = assocMapPoints.size(); 
    array.layout.dim.push_back(tempDim);
    
    tempDim.label = "Columns";
    tempDim.size = 3+1; //last column is a flag that tells whether associated keypoint is bad (0.0) or good (1.0)
    array.layout.dim.push_back(tempDim);    

    array.data.clear();
    
    

    
    for(size_t i=0, iend=assocMapPoints.size(); i<iend;i++)
    {
      if(assocMapPoints[i]!=NULL)
      {
	if(assocMapPoints[i]->isBad())
	{
	    array.data.push_back(0.0);
	    array.data.push_back(0.0);
	    array.data.push_back(0.0);	
	    array.data.push_back(0.0);
	}
	else
	{
	    cv::Mat pos = assocMapPoints[i]->GetWorldPos();
	    array.data.push_back(pos.at<float>(0));
	    array.data.push_back(pos.at<float>(1));
	    array.data.push_back(pos.at<float>(2));
	    array.data.push_back(1.0);
	}
      }
      else
      {
	    array.data.push_back(0.0);
	    array.data.push_back(0.0);
	    array.data.push_back(0.0);	
	    array.data.push_back(0.0);	
      }
    }    
    
    publisher_3dPoints.publish(array);
  
}


} //namespace ORB_SLAM

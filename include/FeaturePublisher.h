/**
* This file is part of ORB-SLAM.
*
* Copyright (C) 2016 Aamir Ahmad <aamir.iitkgp@gmail.com> (max Planck Institute for biological Cybernetics). This code is built on top of Raúl Mur-Artal <raulmur at unizar dot es>'s original ORB-SLAM code
* For more information see <http://webdiis.unizar.es/~raulmur/orbslam/>
*
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

#ifndef FEATUREPUBLISHER_H
#define FEATUREPUBLISHER_H

#include<ros/ros.h>
#include <visualization_msgs/Marker.h>

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"

#include "std_msgs/Int32MultiArray.h"

#include"Map.h"
#include"MapPoint.h"
#include"KeyFrame.h"
#include"Frame.h"

namespace ORB_SLAM
{

class FeaturePublisher
{
public:
    FeaturePublisher(int ID);

    Map* mpMap;

    void Refresh();
    void PublishFeatures(cv::Mat& descriptor);
    void Publish3DPointsAssociated(const std::vector<MapPoint*> &assocMapPoints);

private:

    ros::NodeHandle nh;
    ros::Publisher publisher_orb, publisher_3dPoints;
    
    cv::Mat descriptorMatrix;
    boost::mutex mMutexCamera;
    // robot ID Variable
    int robotID;        
};

} //namespace ORB_SLAM

#endif // FEATUREPUBLISHER_H

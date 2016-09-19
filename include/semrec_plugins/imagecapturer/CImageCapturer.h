/*************************************************************************
 * Copyright (c) 2016, Jan Winkler <jan.winkler.84@gmail.com>.
 * All rights reserved.
 * 
 * This file is part of semrec_plugins.
 * 
 * semrec_plugins is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * semrec_plugins is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with Foobar.  If not, see <http://www.gnu.org/licenses/>.
 *************************************************************************/

/** \author Jan Winkler */


#ifndef __C_IMAGECAPTURER_H__
#define __C_IMAGECAPTURER_H__


// System
#include <string>
#include <iostream>
#include <fstream>
#include <algorithm>

// ROS
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

// OpenCV
#include <cv.h>
#include <highgui.h>


namespace semrec {
  class CImageCapturer {
  private:
    ros::Subscriber m_subImage;
    bool m_bReceived;
    sensor_msgs::Image m_imgReceived;
    
    bool fileExists(std::string strFileName);
    void freeFilename(std::string& strFileName, std::string strWorkingDirectory);
    
    std::string m_strImagesTopic;
    ros::Publisher m_pubImages;
    
    double m_dTimeout; // In seconds
    
  public:
    CImageCapturer();
    ~CImageCapturer();
    
    bool captureFromTopic(std::string strTopicName, std::string &strFileName, std::string strWorkingDirectory, bool bUseFreeName = true);
    void imageCallback(const sensor_msgs::Image &imgData);
    
    void setTimeout(double dTimeout);
    void publishImages(std::string strImagesTopic);
  };
}


#endif /* __C_IMAGECAPTURER_H__ */

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
 * along with semrec_plugins.  If not, see <http://www.gnu.org/licenses/>.
 *************************************************************************/

/** \author Jan Winkler */


#include <semrec_plugins/imagecapturer/CImageCapturer.h>


namespace semrec {
  CImageCapturer::CImageCapturer() {
    m_strImagesTopic = "";
    m_dTimeout = 3;
  }
  
  CImageCapturer::~CImageCapturer() {
  }
  
  bool CImageCapturer::fileExists(std::string strFileName) {
    std::ifstream ifile(strFileName.c_str());
    
    if(ifile) {
      return true;
    }
    
    return false;
  }

  static inline bool isDepthImage(const std::string& encoding) {
    return encoding == sensor_msgs::image_encodings::TYPE_32FC1 ||
           encoding == sensor_msgs::image_encodings::TYPE_64FC1;
  }
  
  void CImageCapturer::freeFilename(std::string& strFileName, std::string strWorkingDirectory) {
    int nIndex = 0;
    std::string strBase = strWorkingDirectory + strFileName;
    std::string strFile = strFileName;
    
    while(this->fileExists(strBase)) {
      std::stringstream sts;
      sts << nIndex;
      sts << "_";
      sts << strFileName;
      
      nIndex++;
      
      strFile = sts.str();
      strBase = strWorkingDirectory + sts.str();
    }
    
    strFileName = strFile;
  }
  
  bool CImageCapturer::captureFromTopic(std::string strTopicName, std::string& strFileName, std::string strWorkingDirectory, bool bUseFreeName) {
    bool bReturnvalue = false;
    
    double dTimeout = m_dTimeout;
    bool bGoon = true;
    m_bReceived = false;
    
    ros::NodeHandle nh;
    m_subImage = nh.subscribe(strTopicName, 1, &CImageCapturer::imageCallback, this);
    
    while(bGoon) {
      dTimeout -= 0.1;
      ros::Duration(0.1).sleep();
      
      // NOTE(winkler): Is spinning actually necessary here? There is
      // another thread maintained by the ''ros'' plugin that does
      // continuous spinning. Check if this mechanism works without an
      // explicit ''spinOnce'' here. This could be a reason for images
      // not appearing on the publishing topic.
      
      ros::spinOnce();
      
      if(dTimeout <= 0 || m_bReceived) {
	bGoon = false;
      }
    }
    
    m_subImage.shutdown();
    
    if(m_bReceived) {
      cv_bridge::CvImagePtr cv_ptr;
      
      try {
	if(sensor_msgs::image_encodings::isColor(m_imgReceived.encoding)) {
	  cv_ptr = cv_bridge::toCvCopy(m_imgReceived, sensor_msgs::image_encodings::BGR8);
	} else if(isDepthImage(m_imgReceived.encoding)) {
	  cv_ptr = cv_bridge::toCvCopy(m_imgReceived);
	  cv::convertScaleAbs(cv_ptr->image, cv_ptr->image, 100.0, 0.0);
	  cv_ptr->encoding = sensor_msgs::image_encodings::MONO8;
	} else {
	  cv_ptr = cv_bridge::toCvCopy(m_imgReceived, sensor_msgs::image_encodings::MONO8);
	}
	
	cv::Mat imgMat = cv_ptr->image;
	std::string strUseFilename = "";
	
	if(bUseFreeName) {
	  this->freeFilename(strFileName, strWorkingDirectory);
	  strUseFilename = strWorkingDirectory + strFileName;
	} else {
	  strUseFilename = strWorkingDirectory + strFileName;
	}
	
	cv::imwrite(strUseFilename, imgMat);
	
	if(m_strImagesTopic != "") {
	  m_pubImages.publish(m_imgReceived);
	}
	
	bReturnvalue = true;
      } catch(cv_bridge::Exception& e) {
	// Timeout reached
      }
    }
    
    return bReturnvalue;
  }
  
  void CImageCapturer::imageCallback(const sensor_msgs::Image &imgData) {
    if(!m_bReceived) {
      m_bReceived = true;
      m_imgReceived = imgData;
    }
  }
  
  void CImageCapturer::setTimeout(double dTimeout) {
    m_dTimeout = dTimeout;
  }
  
  void CImageCapturer::publishImages(std::string strImagesTopic) {
    m_strImagesTopic = strImagesTopic;
    ros::NodeHandle nh;
    
    m_pubImages = nh.advertise<sensor_msgs::Image>(strImagesTopic, 1);
  }
}

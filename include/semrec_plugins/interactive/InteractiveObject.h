/** \author Jan Winkler */

#ifndef __INTERACTIVE_OBJECT_H__
#define __INTERACTIVE_OBJECT_H__


// System
#include <string>
#include <iostream>
#include <mutex>

// ROS
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <ros/ros.h>


namespace semrec {
  typedef struct {
    std::string strLabel;
    std::string strIdentifier;
    std::string strParameter;
    uint8_t unMenuEntryID;
  } InteractiveMenuEntry;
  
  typedef struct {
    std::string strObject;
    std::string strCommand;
    std::string strParameter;
  } InteractiveObjectCallbackResult;
  
  class InteractiveObject {
  private:
    visualization_msgs::Marker m_mkrMarker;
    interactive_markers::MenuHandler m_mhMenu;
    visualization_msgs::InteractiveMarker m_imMarker;
    interactive_markers::InteractiveMarkerServer* m_imsServer;
    visualization_msgs::InteractiveMarkerControl m_imcControl;
    std::list<InteractiveMenuEntry> m_lstMenuEntries;
    std::list<InteractiveObjectCallbackResult> m_lstCallbackResults;
    std::mutex m_mtxCallbackResults;
    
  public:
    InteractiveObject(std::string strName);
    ~InteractiveObject();
    
    bool insertIntoServer(interactive_markers::InteractiveMarkerServer* imsServer);
    bool removeFromServer();
    void clickCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
    
    void setMarker(visualization_msgs::Marker mkrMarker);
    void setSize(float fWidth, float fDepth, float fHeight);
    void setPose(std::string strFixedFrame, geometry_msgs::Pose posPose);
    void setPose(geometry_msgs::Pose posPose);
    
    std::string name();
    void addMenuEntry(std::string strLabel, std::string strIdentifier, std::string strParameter = "");
    void removeMenuEntry(std::string strIdentifier, std::string strParameter = "");
    void clearMenuEntries();
    
    std::list<InteractiveObjectCallbackResult> callbackResults();
  };
}


#endif /* __INTERACTIVE_OBJECT_H__ */

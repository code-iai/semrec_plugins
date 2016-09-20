/*************************************************************************
 * Copyright (c) 2016, Jan Winkler <jan.winkler.84@gmail.com>.
 * All rights reserved.
 * 
 * This file is part of semrec_plugins.
 * 
 * semrec_plugins is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
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


#ifndef __PLUGIN_INTERACTIVE_H__
#define __PLUGIN_INTERACTIVE_H__


#define PLUGIN_CLASS PluginInteractive


// System
#include <cstdlib>
#include <iostream>

// ROS
#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

// Designators
#include <designators/Designator.h>
#include <designator_integration_msgs/DesignatorCommunication.h>

// Private
#include <semrec/Types.h>
#include <semrec/ForwardDeclarations.h>
#include <semrec/Plugin.h>
#include <semrec_plugins/interactive/InteractiveObject.h>


namespace semrec {
  namespace plugins {
    class PLUGIN_CLASS : public Plugin {
    private:
      interactive_markers::InteractiveMarkerServer* m_imsServer;
      std::list<InteractiveObject*> m_lstInteractiveObjects;
      
    public:
      PLUGIN_CLASS();
      ~PLUGIN_CLASS();
      
      virtual Result init(int argc, char** argv);
      virtual Result deinit();
      
      virtual Result cycle();
      
      virtual void consumeEvent(Event evEvent);
      
      InteractiveObject* addInteractiveObject(InteractiveObject* ioAdd);
      InteractiveObject* addInteractiveObject(std::string strName);
      InteractiveObject* interactiveObjectForName(std::string strName);
      InteractiveObject* updatePoseForInteractiveObject(std::string strName, geometry_msgs::Pose posUpdate);
      bool removeInteractiveObject(std::string strName);
    };
  }
  
  extern "C" plugins::PLUGIN_CLASS* createInstance();
  extern "C" void destroyInstance(plugins::PLUGIN_CLASS* icDestroy);
}


#endif /* __PLUGIN_INTERACTIVE_H__ */

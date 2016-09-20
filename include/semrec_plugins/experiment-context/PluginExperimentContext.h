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


#ifndef __PLUGIN_EXPERIMENT_CONTEXT_H__
#define __PLUGIN_EXPERIMENT_CONTEXT_H__


#define PLUGIN_CLASS PluginExperimentContext


// System
#include <map>
#include <cstdlib>
#include <iostream>
#include <sys/stat.h>
#include <sys/types.h>

// ROS
#include <ros/ros.h>

// Private
#include <semrec/Types.h>
#include <semrec/ForwardDeclarations.h>
#include <semrec/Plugin.h>
#include <semrec/plugins/dotexporter/CExporterDot.h>
#include <semrec/CExporterFileoutput.h>


namespace semrec {
  namespace plugins {
    class PLUGIN_CLASS : public Plugin {
    private:
      std::map<std::string, MappedMetaData> m_mapValues;
      CExporterFileoutput* m_expFileExporter;
      ros::Publisher m_pubMetadata;
      
    public:
      PLUGIN_CLASS();
      ~PLUGIN_CLASS();
      
      virtual Result init(int argc, char** argv);
      virtual Result deinit();
      
      virtual Result cycle();
      
      virtual void consumeEvent(Event evEvent);
    };
  }
  
  extern "C" plugins::PLUGIN_CLASS* createInstance();
  extern "C" void destroyInstance(plugins::PLUGIN_CLASS* icDestroy);
}


#endif /* __PLUGIN_EXPERIMENT_CONTEXT_H__ */

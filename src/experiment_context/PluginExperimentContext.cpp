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


#include <semrec_plugins/experiment-context/PluginExperimentContext.h>


namespace semrec {
  namespace plugins {
    PLUGIN_CLASS::PLUGIN_CLASS() {
      m_expFileExporter = new CExporterFileoutput();
      
      this->setPluginVersion("0.3");
      this->addDependency("ros");
    }
    
    PLUGIN_CLASS::~PLUGIN_CLASS() {
      if(m_expFileExporter) {
	delete m_expFileExporter;
      }
    }
    
    Result PLUGIN_CLASS::init(int argc, char** argv) {
      this->setSubscribedToEvent("set-experiment-meta-data", true);
      this->setSubscribedToEvent("export-planlog", true);
      this->setSubscribedToEvent("experiment-start", true);
      this->setSubscribedToEvent("experiment-shutdown", true);
      this->setSubscribedToEvent("update-absolute-experiment-start-time", true);
      this->setSubscribedToEvent("update-absolute-experiment-end-time", true);
      
      ros::NodeHandle nh;
      m_pubMetadata = nh.advertise<designator_integration_msgs::Designator>("/logged_metadata", 1);
      
      return defaultResult();
    }
    
    Result PLUGIN_CLASS::deinit() {
      return defaultResult();
    }
    
    Result PLUGIN_CLASS::cycle() {
      Result resCycle = defaultResult();
      this->deployCycleData(resCycle);
      
      return resCycle;
    }
    
    void PLUGIN_CLASS::consumeEvent(Event evEvent) {
      if(evEvent.strEventName == "set-experiment-meta-data") {
	if(evEvent.cdDesignator) {
	  std::string strField = evEvent.cdDesignator->stringValue("field");
	  std::string strValue = evEvent.cdDesignator->stringValue("value");
	  std::string strType = evEvent.cdDesignator->stringValue("type");
	  
	  MappedMetaData::Type mtType = (strType == "resource" ? MappedMetaData::Resource : MappedMetaData::Property);
	  
	  if(strField != "") {
	    this->info("Set " + std::string(mtType == MappedMetaData::Resource ? "resource" : "property") + " '" + strField + "' to '" + strValue + "'");
	    m_mapValues[strField] = {mtType, strValue};
	  }
	}
      } else if(evEvent.strEventName == "export-planlog") {
	std::string strFormat = evEvent.cdDesignator->stringValue("format");
	transform(strFormat.begin(), strFormat.end(), strFormat.begin(), ::tolower);
	
	if(strFormat == "meta") {
	  Designator* cdMeta = new Designator();
	  cdMeta->setType(Designator::DesignatorType::ACTION);
	  
	  this->info("Experiment Context Plugin exporting meta-data");
	  
	  if(m_mapValues.find("time-end") == m_mapValues.end()) {
	    m_mapValues["time-end"] = {MappedMetaData::Property, this->getTimeStampStr()};
	  }
	  
	  ConfigSettings cfgsetCurrent = configSettings();
	  std::string strMetaFile = cfgsetCurrent.strExperimentDirectory + "metadata.xml";
	  
	  std::string strMetaXML = "";
	  strMetaXML += "<?xml version=\"1.0\" encoding=\"utf-8\"?>\n\n";
	  strMetaXML += "<meta-data>\n";
	  for(std::pair<std::string, MappedMetaData> prEntry : m_mapValues) {
	    strMetaXML += "  <" + prEntry.first + ">" + prEntry.second.strValue + "</" + prEntry.first + ">\n";
	    cdMeta->setValue(prEntry.first, prEntry.second.strValue);
	  }
	  strMetaXML += "</meta-data>\n";
	  
	  this->info("Published metadata");
	  m_pubMetadata.publish(cdMeta->serializeToMessage());
	  delete cdMeta;
	  
	  m_expFileExporter->writeToFile(strMetaXML, strMetaFile);
	  
	  this->info("Successfully exported meta-data to '" + strMetaFile + "'");
	}
      } else if(evEvent.strEventName == "experiment-start") {
	//m_mapValues["time-start"] = this->getTimeStampStr();
      } else if(evEvent.strEventName == "experiment-shutdown") {
	//m_mapValues["time-end"] = this->getTimeStampStr();
      } else if(evEvent.strEventName == "update-absolute-experiment-start-time") {
	if(evEvent.lstNodes.size() > 0) {
	  // Only the first node counts, as the first node represents
	  // the earliest time.
	  if(m_mapValues.find("time-start") == m_mapValues.end()) {
	    m_mapValues["time-start"] = {MappedMetaData::Property, evEvent.lstNodes.front()->metaInformation()->stringValue("time-start")};
	  }
	}
      } else if(evEvent.strEventName == "update-absolute-experiment-end-time") {
	if(evEvent.lstNodes.size() > 0) {
	  // Every end time overwrites any already existing value, as
	  // it always happens after.
	  m_mapValues["time-end"] = {MappedMetaData::Property, evEvent.lstNodes.front()->metaInformation()->stringValue("time-end")};
	}
      }
    }
  }
  
  extern "C" plugins::PLUGIN_CLASS* createInstance() {
    return new plugins::PLUGIN_CLASS();
  }
  
  extern "C" void destroyInstance(plugins::PLUGIN_CLASS* icDestroy) {
    delete icDestroy;
  }
}

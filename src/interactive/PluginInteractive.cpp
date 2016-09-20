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


#include <semrec_plugins/interactive/PluginInteractive.h>


namespace semrec {
  namespace plugins {
    PLUGIN_CLASS::PLUGIN_CLASS() {
      this->addDependency("ros");
      this->setPluginVersion("0.4");
      
      m_imsServer = NULL;
    }
    
    PLUGIN_CLASS::~PLUGIN_CLASS() {
      if(m_imsServer) {
	delete m_imsServer;
      }
    }
    
    Result PLUGIN_CLASS::init(int argc, char** argv) {
      Result resInit = defaultResult();
      
      // Initialize server
      m_imsServer = new interactive_markers::InteractiveMarkerServer("interactive_semrec", "", false);
      
      // Subscribe to internal events
      this->setSubscribedToEvent("symbolic-add-object", true);
      this->setSubscribedToEvent("symbolic-remove-object", true);
      this->setSubscribedToEvent("symbolic-update-object-pose", true);
      this->setSubscribedToEvent("symbolic-set-interactive-object-menu", true);
      
      return resInit;
    }
    
    InteractiveObject* PLUGIN_CLASS::updatePoseForInteractiveObject(std::string strName, geometry_msgs::Pose posUpdate) {
      InteractiveObject* ioUpdate = this->interactiveObjectForName(strName);
      
      if(!ioUpdate) {
	ioUpdate = this->addInteractiveObject(strName);
      }
      
      ioUpdate->setPose(posUpdate);
      
      return ioUpdate;
    }
    
    InteractiveObject* PLUGIN_CLASS::addInteractiveObject(std::string strName) {
      return this->addInteractiveObject(new InteractiveObject(strName));
    }
    
    InteractiveObject* PLUGIN_CLASS::addInteractiveObject(InteractiveObject* ioAdd) {
      m_lstInteractiveObjects.push_back(ioAdd);
      ioAdd->insertIntoServer(m_imsServer);
      
      return ioAdd;
    }
    
    InteractiveObject* PLUGIN_CLASS::interactiveObjectForName(std::string strName) {
      for(InteractiveObject* ioCurrent : m_lstInteractiveObjects) {
	if(ioCurrent->name() == strName) {
	  return ioCurrent;
	}
      }
      
      return NULL;
    }
    
    bool PLUGIN_CLASS::removeInteractiveObject(std::string strName) {
      for(std::list<InteractiveObject*>::iterator itIO = m_lstInteractiveObjects.begin();
	  itIO != m_lstInteractiveObjects.end();
	  itIO++) {
	if((*itIO)->name() == strName) {
	  (*itIO)->removeFromServer();
	  m_lstInteractiveObjects.erase(itIO);
	  
	  return true;
	}
      }
      
      return false;
    }
    
    Result PLUGIN_CLASS::deinit() {
      Result resReturn = defaultResult();
      
      for(InteractiveObject* ioDelete : m_lstInteractiveObjects) {
	delete ioDelete;
      }
      
      return resReturn;
    }
    
    Result PLUGIN_CLASS::cycle() {
      Result resCycle = defaultResult();
      
      for(InteractiveObject* ioCurrent : m_lstInteractiveObjects) {
	std::list<InteractiveObjectCallbackResult> lstCBResults = ioCurrent->callbackResults();
	
	for(InteractiveObjectCallbackResult iocrResult : lstCBResults) {
	  this->info("Interactive callback for object '" + iocrResult.strObject + "': '" + iocrResult.strCommand + "', '" + iocrResult.strParameter + "'");
	  
	  Event evCallback = defaultEvent("interactive-callback");
	  evCallback.cdDesignator = new Designator();
	  evCallback.cdDesignator->setType(Designator::DesignatorType::ACTION);
	  evCallback.cdDesignator->setValue("object", iocrResult.strObject);
	  evCallback.cdDesignator->setValue("command", iocrResult.strCommand);
	  evCallback.cdDesignator->setValue("parameter", iocrResult.strParameter);
	  
	  this->deployEvent(evCallback);
	}
      }
      
      this->deployCycleData(resCycle);
      
      return resCycle;
    }
    
    void PLUGIN_CLASS::consumeEvent(Event evEvent) {
      if(evEvent.strEventName == "symbolic-add-object") {
	if(evEvent.cdDesignator) {
	  std::string strObjectName = evEvent.cdDesignator->stringValue("name");
	  
	  if(strObjectName != "") {
	    InteractiveObject* ioNew = this->addInteractiveObject(strObjectName);
	    
	    bool bPoseGiven = false;
	    if(evEvent.cdDesignator->childForKey("pose") != NULL) {
	      ioNew->setPose(evEvent.cdDesignator->poseValue("pose"));
	      bPoseGiven = true;
	    }
	    
	    if(evEvent.cdDesignator->childForKey("pose-stamped") != NULL) {
	      geometry_msgs::PoseStamped psPose = evEvent.cdDesignator->poseStampedValue("pose-stamped");
	      ioNew->setPose(psPose.header.frame_id, psPose.pose);
	      bPoseGiven = true;
	    }
	    
	    if(!bPoseGiven) {
	      this->info("No pose given for object '" + strObjectName + "'.");
	    }
	    
	    float fWidth = 0.1;
	    float fDepth = 0.1;
	    float fHeight = 0.1;
	    
	    if(evEvent.cdDesignator->childForKey("width") != NULL &&
	       evEvent.cdDesignator->childForKey("depth") != NULL &&
	       evEvent.cdDesignator->childForKey("height") != NULL) {
	      fWidth = evEvent.cdDesignator->floatValue("width");
	      fDepth = evEvent.cdDesignator->floatValue("depth");
	      fHeight = evEvent.cdDesignator->floatValue("height");
	    } else {
	      this->info("No dimension (width, depth, height) given for object '" + strObjectName + "', assuming default (0.1, 0.1, 0.1).");
	    }
	    
	    ioNew->setSize(fWidth, fDepth, fHeight);
	    
	    if(evEvent.cdDesignator->childForKey("menu") != NULL) {
	      ioNew->clearMenuEntries();
	      
	      KeyValuePair* ckvpMenu = evEvent.cdDesignator->childForKey("menu");
	      std::list<std::string> lstMenuKeys = ckvpMenu->keys();
	      
	      for(std::string strKey : lstMenuKeys) {
		KeyValuePair* ckvpMenuEntry = ckvpMenu->childForKey(strKey);
		std::string strLabel = ckvpMenuEntry->stringValue("label");
		std::string strParameter = ckvpMenuEntry->stringValue("parameter");
		
		ioNew->addMenuEntry(strLabel, strKey, strParameter);
		this->info("Added menu entry '" + strKey + "': '" + strLabel + "'");
	      }
	    } else {
	      this->info("No menu given for object '" + strObjectName + "'.");
	    }
	    
	    this->info("Registered interactive object '" + strObjectName + "'.");
	  } else {
	    this->warn("No name given when adding interactive object!");
	  }
	} else {
	  this->warn("No designator given when adding interactive object!");
	}
      } else if(evEvent.strEventName == "symbolic-remove-object") {
	if(evEvent.cdDesignator) {
	  std::string strObjectName = evEvent.cdDesignator->stringValue("name");
	  
	  if(strObjectName != "") {
	    if(!this->removeInteractiveObject(strObjectName)) {
	      this->warn("Tried to unregister non-existing interactive object '" + strObjectName + "'.");
	    }
	  }
	}
      } else if(evEvent.strEventName == "symbolic-update-object-pose") {
	if(evEvent.cdDesignator) {
	  std::string strObjectName = evEvent.cdDesignator->stringValue("name");
	  
	  if(strObjectName != "") {
	    geometry_msgs::Pose psSet = evEvent.cdDesignator->poseValue("pose");
	    
	    this->updatePoseForInteractiveObject(strObjectName, psSet);
	  }
	}
      } else if(evEvent.strEventName == "symbolic-set-interactive-object-menu") {
	if(evEvent.cdDesignator) {
	  std::string strObjectName = evEvent.cdDesignator->stringValue("name");
	  
	  if(strObjectName != "") {
	    InteractiveObject* ioNew = this->interactiveObjectForName(strObjectName);
	    
	    if(ioNew) {
	      ioNew->clearMenuEntries();
	      
	      KeyValuePair* ckvpMenu = evEvent.cdDesignator->childForKey("menu");
	      std::list<std::string> lstMenuKeys = ckvpMenu->keys();
	      
	      for(std::string strKey : lstMenuKeys) {
		KeyValuePair* ckvpMenuEntry = ckvpMenu->childForKey(strKey);
		std::string strLabel = ckvpMenuEntry->stringValue("label");
		std::string strParameter = ckvpMenuEntry->stringValue("parameter");
		
		ioNew->addMenuEntry(strLabel, strKey, strParameter);
		this->info("Added menu entry '" + strKey + "': '" + strLabel + "'");
	      }
	    } else {
	      this->warn("Interactive object '" + strObjectName + "' not known when setting menu!");
	    }
	  } else {
	    this->warn("No name given when setting interactive object menu!");
	  }
	} else {
	  this->warn("No designator given when setting interactive object menu!");
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

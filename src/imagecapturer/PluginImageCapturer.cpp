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


#include <semrec_plugins/imagecapturer/PluginImageCapturer.h>


namespace semrec {
  namespace plugins {
    PLUGIN_CLASS::PLUGIN_CLASS() {
      m_icapImageCapturer = NULL;
      
      this->addDependency("ros");
      this->setPluginVersion("0.2");
    }
    
    PLUGIN_CLASS::~PLUGIN_CLASS() {
    }
    
    Result PLUGIN_CLASS::init(int argc, char** argv) {
      Result resInit = defaultResult();
      
      m_icapImageCapturer = new CImageCapturer();
      
      Designator* cdConfig = this->getIndividualConfig();
      m_icapImageCapturer->setTimeout(cdConfig->floatValue("timeout", 5.0f));
      m_icapImageCapturer->publishImages(cdConfig->stringValue("image-publishing-topic", "/logged_images"));
      
      this->setSubscribedToEvent("add-image-from-topic", true);
      
      return resInit;
    }
    
    Result PLUGIN_CLASS::deinit() {
      if(m_icapImageCapturer) {
	delete m_icapImageCapturer;
      }
      
      return defaultResult();
    }
    
    Result PLUGIN_CLASS::cycle() {
      Result resCycle = defaultResult();
      this->deployCycleData(resCycle);
      
      return resCycle;
    }
    
    void PLUGIN_CLASS::consumeEvent(Event evEvent) {
      if(evEvent.strEventName == "add-image-from-topic") {
	if(evEvent.cdDesignator) {
	  std::string strTopic = evEvent.cdDesignator->stringValue("origin");
	  std::string strFilename = evEvent.cdDesignator->stringValue("filename");
	  
	  if(strTopic != "") {
	    if(strFilename == "") {
	      strFilename = strTopic;
	      replace(strFilename.begin(), strFilename.end(), '/', '_');
	      strFilename += ".jpg";
	    }
	    
	    ConfigSettings cfgsetCurrent = configSettings();
	    
	    this->info("Waiting for image topic '" + strTopic + "'");
	    if(m_icapImageCapturer->captureFromTopic(strTopic, strFilename, cfgsetCurrent.strExperimentDirectory)) {
	      std::string strFilepath = cfgsetCurrent.strExperimentDirectory + strFilename;
	      this->info("Wrote image from topic '" + strTopic + "' to file '" + strFilepath + "'");
	      
	      Event evImage = eventInResponseTo(evEvent, "add-image-from-file");
	      evImage.cdDesignator = new Designator();
	      evImage.cdDesignator->setType(Designator::DesignatorType::ACTION);
	      evImage.cdDesignator->setValue("origin", strTopic);
	      // NOTE(winkler): We just use the filename here, not the
	      // global filepath. This is due to the fact that all
	      // images are stored relative to the generated .owl file
	      // (i.e. in the same directory). When moving all files
	      // somewhere else, global paths would make finding of
	      // files very difficult.
	      evImage.cdDesignator->setValue("filename", strFilename);
	      evImage.cdDesignator->setValue("absolute-path", strFilepath);
	      
	      m_mtxEventsStore.lock();
	      m_lstEvents.push_back(evImage);
	      m_mtxEventsStore.unlock();
	    } else {
	      this->warn("Failed to capture image from topic '" + strTopic + "' and write it to '" + cfgsetCurrent.strExperimentDirectory + strFilename + "'.");
	      
	      Event evImage = eventInResponseTo(evEvent, "cancel-open-request");
	      
	      m_mtxEventsStore.lock();
	      m_lstEvents.push_back(evImage);
	      m_mtxEventsStore.unlock();
	    }
	  } else {
	    this->warn("No topic was given when requesting to capture an image from a topic.");
	  }
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

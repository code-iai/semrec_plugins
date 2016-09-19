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


#include <semrec_plugins/console/PluginConsole.h>


namespace semrec {
  namespace plugins {
    PLUGIN_CLASS::PLUGIN_CLASS() {
      this->setPluginVersion("0.1");
      
      m_nScreenWidth = 1;
      m_nScreenHeight = 1;
      
      m_bFirstDisplay = true;
      m_bNeedsRedisplay = false;
      
      m_nBufferLineSize = 500;
    }
    
    PLUGIN_CLASS::~PLUGIN_CLASS() {
    }
    
    Result PLUGIN_CLASS::init(int argc, char** argv) {
      Result resInit = defaultResult();
      
      this->initCurses();
      
      this->setSubscribedToEvent("status-message", true);
      this->setSubscribedToEvent("resize-terminal-window", true);
      
      this->checkScreenSize();
      this->setNeedsRedisplay();
      
      return resInit;
    }
    
    void PLUGIN_CLASS::initCurses() {
      setlocale(LC_ALL, "");
      
      m_winMain = initscr();
      start_color();
      
      noecho();
      keypad(m_winMain, true);
      scrollok(m_winMain, false);
      timeout(0.01);
      curs_set(0);
      
      m_winLog = newwin(1, 1, 1, 1);
      
      this->registerColor("30", COLOR_BLACK);
      this->registerColor("31", COLOR_RED);
      this->registerColor("32", COLOR_GREEN);
      this->registerColor("33", COLOR_YELLOW);
      this->registerColor("34", COLOR_BLUE);
      this->registerColor("35", COLOR_MAGENTA);
      this->registerColor("36", COLOR_CYAN);
      this->registerColor("37", COLOR_WHITE);
    }
    
    void PLUGIN_CLASS::deinitCurses() {
      endwin();
    }
    
    void PLUGIN_CLASS::printInterface() {
      m_mtxStatusMessages.lock();
      
      if(m_bFirstDisplay) {
	clear();
	m_bFirstDisplay = false;
      }
      
      wclrtobot(stdscr);
      wclrtobot(m_winMain);
      wclrtobot(m_winLog);
      
      box(m_winMain, 0, 0);
      
      int nLogWidth = std::max(4, m_nScreenWidth - 2);
      int nLogHeight = std::max(1, m_nScreenHeight - 2);
      
      int nLinesUsed = 0;
      int nReverseIndex = 0;
      for(std::list<StatusMessage>::reverse_iterator itSMr = m_lstStatusMessages.rbegin();
	  itSMr != m_lstStatusMessages.rend(); itSMr++) {
	StatusMessage msgStatus = *itSMr;
	std::string strStatus = "[ " + msgStatus.strPrefix + " ] " + msgStatus.strMessage;
	int nLinesPerLine = 1;
	
	while(strStatus.length() > nLogWidth) {
	  strStatus = strStatus.substr(nLogWidth);
	  strStatus += "   ";
	  
	  nLinesPerLine++;
	}
	
	if(nLinesUsed + nLinesPerLine > nLogHeight) {
	  break;
	}
	
	nLinesUsed += nLinesPerLine;
	nReverseIndex++;
      }
      
      int nLine = 0;
      std::list<StatusMessage>::iterator itSM = m_lstStatusMessages.begin();
      int nDiff = m_lstStatusMessages.size() - nReverseIndex;
      
      if(nDiff > 0) {
	advance(itSM, nDiff);
      }
      
      for(; itSM != m_lstStatusMessages.end();
      	  ++itSM) {
      	StatusMessage msgStatus = *itSM;
      	short sColor = this->colorNumber(msgStatus.strColorCode);
	
      	if(sColor != -1) {
      	  wattron(m_winLog, COLOR_PAIR(sColor));
      	}
	
	std::string strPrint = "[ " + msgStatus.strPrefix + " ] " + msgStatus.strMessage;
	
	while(strPrint.length() > 0) {
	  move(nLine, 0);
	  wclrtoeol(m_winLog);
	  
	  int nLen = (strPrint.length() > nLogWidth ? nLogWidth : strPrint.length());
	  mvwaddstr(m_winLog, nLine, 0, strPrint.substr(0, nLen).c_str());
	  wclrtoeol(m_winLog);
	  strPrint = "   " + strPrint.substr(nLen);
	  
	  if(strPrint == "   ") {
	    strPrint = "";
	  }
	  
	  nLine++;
	}
	
      	if(sColor != -1) {
      	  wattroff(m_winLog, COLOR_PAIR(sColor));
      	}
      }
      
      wrefresh(m_winMain);
      wrefresh(m_winLog);
      
      m_mtxStatusMessages.unlock();
    }
    
    bool PLUGIN_CLASS::checkScreenSize() {
      int nScreenWidth = 0, nScreenHeight = 0;
      
      m_mtxStatusMessages.lock();
      wrefresh(stdscr);
      getmaxyx(stdscr, nScreenHeight, nScreenWidth);
      
      if(nScreenWidth != m_nScreenWidth || nScreenHeight != m_nScreenHeight) {
	// Screen size changed
	m_nScreenWidth = nScreenWidth;
	m_nScreenHeight = nScreenHeight;
	
	wresize(m_winLog, m_nScreenHeight - 2, m_nScreenWidth - 2);
	wresize(m_winMain, m_nScreenHeight, m_nScreenWidth);
	
	m_mtxStatusMessages.unlock();
	
	return true;
      }
      
      m_mtxStatusMessages.unlock();
      
      return false;
    }
    
    Result PLUGIN_CLASS::deinit() {
      deinitCurses();
      
      return defaultResult();
    }
    
    Result PLUGIN_CLASS::cycle() {
      Result resCycle = defaultResult();
      
      if(this->needsRedisplay()) {
	this->printInterface();
      }
      
      this->deployCycleData(resCycle);
      
      return resCycle;
    }
    
    void PLUGIN_CLASS::consumeEvent(Event evEvent) {
      if(evEvent.strEventName == "status-message") {
	m_mtxStatusMessages.lock();
	m_lstStatusMessages.push_back(evEvent.msgStatusMessage);
	
	while(m_lstStatusMessages.size() > m_nBufferLineSize) {
	  m_lstStatusMessages.pop_front();
	}
	
	m_mtxStatusMessages.unlock();
	
	this->setNeedsRedisplay();
      } else if(evEvent.strEventName == "resize-terminal-window") {
	if(this->checkScreenSize()) {
	  this->setNeedsRedisplay();
	}
      }
    }
    
    void PLUGIN_CLASS::registerColor(std::string strColorCode, short sColor) {
      short sPair = m_mapColors.size() + 1;
      init_pair(sPair, sColor, COLOR_BLACK);
      m_mapColors[strColorCode] = sPair;
    }
    
    short PLUGIN_CLASS::colorNumber(std::string strColorCode) {
      std::map<std::string, short>::iterator itEntry = m_mapColors.find(strColorCode);
      
      if(itEntry != m_mapColors.end()) {
	return (*itEntry).second;
      }
      
      return -1;
    }
    
    void PLUGIN_CLASS::setNeedsRedisplay() {
      m_mtxRedisplay.lock();
      m_bNeedsRedisplay = true;
      m_mtxRedisplay.unlock();
    }
    
    bool PLUGIN_CLASS::needsRedisplay() {
      m_mtxRedisplay.lock();
      bool bRedisplay = m_bNeedsRedisplay;
      m_bNeedsRedisplay = false;
      m_mtxRedisplay.unlock();
      
      return bRedisplay;
    }
  }
  
  extern "C" plugins::PLUGIN_CLASS* createInstance() {
    return new plugins::PLUGIN_CLASS();
  }
  
  extern "C" void destroyInstance(plugins::PLUGIN_CLASS* icDestroy) {
    delete icDestroy;
  }
}

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


#ifndef __PLUGIN_CONSOLE_H__
#define __PLUGIN_CONSOLE_H__


#define PLUGIN_CLASS PluginConsole


// System
#include <cstdlib>
#include <iostream>
#include <sys/stat.h>
#include <sys/types.h>
#include <ncurses.h>
#include <mutex>
#include <algorithm>

// Private
#include <semrec/Types.h>
#include <semrec/ForwardDeclarations.h>
#include <semrec/Plugin.h>


namespace semrec {
  namespace plugins {
    class PLUGIN_CLASS : public Plugin {
    private:
      int m_nScreenHeight;
      int m_nScreenWidth;
      std::map<std::string, short> m_mapColors;
      bool m_bNeedsRedisplay;
      std::mutex m_mtxRedisplay;
      bool m_bFirstDisplay;
      std::list<StatusMessage> m_lstStatusMessages;
      std::mutex m_mtxStatusMessages;
      WINDOW* m_winMain;
      WINDOW* m_winLog;
      int m_nBufferLineSize;
      
    public:
      PLUGIN_CLASS();
      ~PLUGIN_CLASS();
      
      virtual Result init(int argc, char** argv);
      void initCurses();
      void deinitCurses();
      
      bool checkScreenSize();
      void printInterface();
      
      virtual Result deinit();
      
      virtual Result cycle();
      
      virtual void consumeEvent(Event evEvent);
      
      void registerColor(std::string strColorCode, short sColor);
      short colorNumber(std::string strColorCode);
      
      void setNeedsRedisplay();
      bool needsRedisplay();
    };
  }
  
  extern "C" plugins::PLUGIN_CLASS* createInstance();
  extern "C" void destroyInstance(plugins::PLUGIN_CLASS* icDestroy);
}


#endif /* __PLUGIN_CONSOLE_H__ */

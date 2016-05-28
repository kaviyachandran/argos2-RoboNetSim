/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

/**
 * @file argos2/simulator/factories/query_factories.h
 *
 * @author Carlo Pinciroli - <cpinciro@ulb.ac.be>
 */

#include "query_factories.h"
#include <argos2/simulator/visualizations/render.h>
#include <argos2/simulator/physics_engines/physics_engine.h>
#include <argos2/simulator/space/entities/entity.h>
#include <argos2/simulator/actuators/simulated_actuator.h>
#include <argos2/simulator/sensors/simulated_sensor.h>

namespace argos {

   /****************************************/
   /****************************************/

   void QueryShowActuatorList() {
      std::map < std::string, std::string, std::less<std::string> >&
         mapShortDescriptions = GetActuatorPlugin()->ShortDescriptionMap;
      LOG << "AVAILABLE ACTUATORS" << std::endl << std::endl;
      for(std::map < std::string, std::string, std::less<std::string> >::iterator it = mapShortDescriptions.begin();
          it != mapShortDescriptions.end(); ++it) {
         LOG << "   [ " << it->first << " ]" << std::endl;
         LOG << "      " << it->second << std::endl << std::endl;
      }
   }

   /****************************************/
   /****************************************/

   void QueryShowSensorList() {
      std::map < std::string, std::string, std::less<std::string> >&
         mapShortDescriptions = GetSensorPlugin()->ShortDescriptionMap;
      LOG << "AVAILABLE SENSORS" << std::endl << std::endl;
      for(std::map < std::string, std::string, std::less<std::string> >::iterator it = mapShortDescriptions.begin();
          it != mapShortDescriptions.end(); ++it) {
         LOG << "   [ " << it->first << " ]" << std::endl;
         LOG << "      " << it->second << std::endl << std::endl;
      }
   }

   /****************************************/
   /****************************************/

   void QueryShowPhysicsEngineList() {
      std::map < std::string, std::string, std::less<std::string> >&
         mapShortDescriptions = GetPhysicsEnginePlugin()->ShortDescriptionMap;
      LOG << "AVAILABLE PHYSICS ENGINES" << std::endl << std::endl;
      for(std::map < std::string, std::string, std::less<std::string> >::iterator it = mapShortDescriptions.begin();
          it != mapShortDescriptions.end(); ++it) {
         LOG << "   [ " << it->first << " ]" << std::endl;
         LOG << "      " << it->second << std::endl << std::endl;
      }
   }

   /****************************************/
   /****************************************/

   void QueryShowVisualizationList() {
      std::map < std::string, std::string, std::less<std::string> >&
         mapShortDescriptions = GetRendererPlugin()->ShortDescriptionMap;
      LOG << "AVAILABLE VISUALIZATIONS" << std::endl << std::endl;
      for(std::map < std::string, std::string, std::less<std::string> >::iterator it = mapShortDescriptions.begin();
          it != mapShortDescriptions.end(); ++it) {
         LOG << "   [ " << it->first << " ]" << std::endl;
         LOG << "      " << it->second << std::endl << std::endl;
      }
   }

   /****************************************/
   /****************************************/

   void QueryShowEntityList() {
      std::map < std::string, std::string, std::less<std::string> >&
         mapShortDescriptions = GetEntityPlugin()->ShortDescriptionMap;
      LOG << "AVAILABLE ENTITIES" << std::endl << std::endl;
      for(std::map < std::string, std::string, std::less<std::string> >::iterator it = mapShortDescriptions.begin();
          it != mapShortDescriptions.end(); ++it) {
         LOG << "   [ " << it->first << " ]" << std::endl;
         LOG << "      " << it->second << std::endl << std::endl;
      }
   }

   /****************************************/
   /****************************************/

   void QuerySearchPlugins(const std::string& str_query,
                           const std::map<std::string, std::string, std::less<std::string> >& map_status,
                           const std::map<std::string, std::string, std::less<std::string> >& map_short_descriptions,
                           const std::map<std::string, std::string, std::less<std::string> >& map_authors,
                           const std::map<std::string, std::string, std::less<std::string> >& map_long_descriptions,
                           std::vector<std::string>& vec_names,
                           std::vector<std::string>& vec_status,
                           std::vector<std::string>& vec_short_descriptions,
                           std::vector<std::string>& vec_authors,
                           std::vector<std::string>& vec_long_descriptions) {
      /* Cycle through the map of short descriptions */
      for(std::map<std::string, std::string, std::less<std::string> >::const_iterator it = map_short_descriptions.begin();
          it != map_short_descriptions.end(); ++it) {
         /* If the current plugin name contains the passed query */
         if(it->first.find(str_query) != std::string::npos) {
            const std::string& strName = it->first;
            /* Add the name to the vector of names */
            vec_names.push_back(strName);
            /* Add the short description to the vector of short descriptions */
            vec_short_descriptions.push_back(it->second);
            /* Add the author to the list of authors */
            vec_authors.push_back(map_authors.find(strName)->second);
            /* Add the long description to the vector of long descriptions */
            vec_long_descriptions.push_back(map_long_descriptions.find(strName)->second);
            /* Add the status to the list of status */
            vec_status.push_back(map_status.find(strName)->second);
         }
      }
   }

   /****************************************/
   /****************************************/

   void QueryShowPluginDescription(const std::string& str_query) {
      LOG << "Plugins matching \"" << str_query << "\":" << std::endl << std::endl;
      /* Buffers to contain the search results */
      std::vector<std::string> vecNames;
      std::vector<std::string> vecStatus;
      std::vector<std::string> vecShortDescriptions;
      std::vector<std::string> vecAuthors;
      std::vector<std::string> vecLongDescriptions;
      /* Search among the actuators */
      QuerySearchPlugins(
         str_query,
         GetActuatorPlugin()->StatusMap,
         GetActuatorPlugin()->ShortDescriptionMap,
         GetActuatorPlugin()->AuthorMap,
         GetActuatorPlugin()->LongDescriptionMap,
         vecNames,
         vecStatus,
         vecShortDescriptions,
         vecAuthors,
         vecLongDescriptions
         );
      /* Search among the sensors */
      QuerySearchPlugins(
         str_query,
         GetSensorPlugin()->StatusMap,
         GetSensorPlugin()->ShortDescriptionMap,
         GetSensorPlugin()->AuthorMap,
         GetSensorPlugin()->LongDescriptionMap,
         vecNames,
         vecStatus,
         vecShortDescriptions,
         vecAuthors,
         vecLongDescriptions
         );
      /* Search among the physics engines */
      QuerySearchPlugins(
         str_query,
         GetPhysicsEnginePlugin()->StatusMap,
         GetPhysicsEnginePlugin()->ShortDescriptionMap,
         GetPhysicsEnginePlugin()->AuthorMap,
         GetPhysicsEnginePlugin()->LongDescriptionMap,
         vecNames,
         vecStatus,
         vecShortDescriptions,
         vecAuthors,
         vecLongDescriptions
         );
      /* Search among the visualizations */
      QuerySearchPlugins(
         str_query,
         GetRendererPlugin()->StatusMap,
         GetRendererPlugin()->ShortDescriptionMap,
         GetRendererPlugin()->AuthorMap,
         GetRendererPlugin()->LongDescriptionMap,
         vecNames,
         vecStatus,
         vecShortDescriptions,
         vecAuthors,
         vecLongDescriptions
         );
      /* Search among the entities */
      QuerySearchPlugins(
         str_query,
         GetEntityPlugin()->StatusMap,
         GetEntityPlugin()->ShortDescriptionMap,
         GetEntityPlugin()->AuthorMap,
         GetEntityPlugin()->LongDescriptionMap,
         vecNames,
         vecStatus,
         vecShortDescriptions,
         vecAuthors,
         vecLongDescriptions
         );
      /* Print the result */
      if(vecNames.empty()) {
         LOG << "   None found." << std::endl << std::endl;
      }
      else {
         LOG << "==============================================================================" << std::endl << std::endl;
         for(UInt32 i = 0; i < vecNames.size(); ++i) {
            LOG << "[ " << vecNames[i] << " ] " << std::endl;
            LOG << vecShortDescriptions[i] << std::endl;
            LOG << "by " << vecAuthors[i] << std::endl;
            LOG << "Status: " << vecStatus[i] << std::endl << std::endl;
            LOG << vecLongDescriptions[i] << std::endl;
            LOG << "==============================================================================" << std::endl << std::endl;
         }
      }
   }

   /****************************************/
   /****************************************/

   void QueryPlugins(const std::string& str_query) {
      if(str_query == "actuators") {
         QueryShowActuatorList();
      } else if(str_query == "sensors") {
         QueryShowSensorList();
      } else if(str_query == "physics_engines") {
         QueryShowPhysicsEngineList();
      } else if(str_query == "visualizations") {
         QueryShowVisualizationList();
      } else if(str_query == "entities") {
         QueryShowEntityList();
      } else if(str_query == "all") {
         QueryShowActuatorList();
         QueryShowSensorList();
         QueryShowPhysicsEngineList();
         QueryShowVisualizationList();
         QueryShowEntityList();
      } else {
         QueryShowPluginDescription(str_query);
      }
   }

   /****************************************/
   /****************************************/

}

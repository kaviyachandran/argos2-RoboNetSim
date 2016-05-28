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

#ifndef QUERY_FACTORIES_H
#define QUERY_FACTORIES_H

#include <map>
#include <vector>
#include <string>

namespace argos {

   void QueryPlugins(const std::string& str_query);

   void QueryShowPluginDescription(const std::string& str_query);

   void QuerySearchPlugins(const std::string& str_query,
                           const std::map<std::string, std::string, std::less<std::string> >& map_status,
                           const std::map<std::string, std::string, std::less<std::string> >& map_short_descriptions,
                           const std::map<std::string, std::string, std::less<std::string> >& map_authors,
                           const std::map<std::string, std::string, std::less<std::string> >& map_long_descriptions,
                           std::vector<std::string>& vec_names,
                           std::vector<std::string>& vec_status,
                           std::vector<std::string>& vec_short_descriptions,
                           std::vector<std::string>& vec_authors,
                           std::vector<std::string>& vec_long_descriptions);

   void QueryShowEntityList();

   void QueryShowVisualizationList();

   void QueryShowPhysicsEngineList();

   void QueryShowSensorList();

   void QueryShowActuatorList();

}

#endif


/* -*- Mode: C++ -*-
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
 * @file <argos2/simulator/sensors/booth/booth_proximity_sensor.cpp>
 *
 * @author @author Manuele Brambilla - <mbrambilla@iridia.ulb.ac.be>
 */

#include <argos2/common/utility/argos_random.h>
#include <argos2/simulator/space/entities/epuck_entity.h>
#include <argos2/simulator/space/entities/embodied_entity.h>
#include <argos2/simulator/simulator.h>

#include "booth_presence_sensor.h"

namespace argos {

   /****************************************/
   /****************************************/

   static const Real PRESENCE_SQUARE_DISTANCE_THRESHOLD = 0.001; // ~3^2 cm

   /****************************************/
   /****************************************/

   CBoothPresenceSensor::CBoothPresenceSensor() :
      m_cSpace(CSimulator::GetInstance().GetSpace()),
      m_pcEmbodiedEntity(NULL){}

   /****************************************/
   /****************************************/

   void CBoothPresenceSensor::Init(TConfigurationNode& t_tree) {
   }

   /****************************************/
   /****************************************/

   void CBoothPresenceSensor::Reset() {
	   m_bPresence = false;
   }

   /****************************************/
   /****************************************/

   void CBoothPresenceSensor::Update() {

	   m_bPresence = false;
	   /* we consider only e-pucks, all the rest is ignored.
	    * This implementation does not use the hash space, since it's quite simple.
	    * Moreover it should be faster for small group of robots (under 100) which is the standard case */
	   CSpace::TAnyEntityMap& tEPuckEntityMap = m_cSpace.GetEntitiesByType("epuck_entity");
	   for(CSpace::TAnyEntityMap::iterator it = tEPuckEntityMap.begin(); it != tEPuckEntityMap.end(); ++it) {
		   // Get a reference to the individual e-puck
		   CEPuckEntity& cEPuck = *(any_cast<CEPuckEntity*>(it->second));
		   // if the epuck center is close to the center of the booth, then return true.
		   CVector3 cEPuckPosition = cEPuck.GetEmbodiedEntity().GetPosition();
		   //LOG << SquareDistance(cEPuckPosition,cBoothPosition) << std::endl;
		   if (SquareDistance(cEPuckPosition,m_cBoothPosition) < PRESENCE_SQUARE_DISTANCE_THRESHOLD){
			   m_bPresence = true;
		   }
	   }
   }

   /****************************************/
   /****************************************/

   REGISTER_SENSOR(CBoothPresenceSensor,
                   "booth_presence", "default",
                   "The booth presence sensor",
                   "Manuele Brambilla [mbrambilla@iridia.ulb.ac.be]",
                   "This sensor accesses the booth presence sensor. For a complete\n"
                   "description of its usage, refer to the common interface.\n"
                   "REQUIRED XML CONFIGURATION\n\n"
                   "  <controllers>\n"
                   "    ...\n"
                   "    <my_controller ...>\n"
                   "      ...\n"
                   "      <sensors>\n"
                   "        ...\n"
                   "        <booth_presence implementation=\"default\" />\n"
                   "        ...\n"
                   "      </sensors>\n"
                   "      ...\n"
                   "    </my_controller>\n"
                   "    ...\n"
                   "  </controllers>\n\n"
                   "OPTIONAL XML CONFIGURATION\n\n"
                   "None\n",
                   "Under development"
		  );

}

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
 * @file argos2/common/utility/configuration/base_configurable_resource.h
 *
 * @brief This file provides the definition of the base configurable resource interface.
 *
 * @author Carlo Pinciroli - <cpinciro@ulb.ac.be>
 */

#ifndef CBASE_CONFIGURABLE_RESOURCE_H
#define CBASE_CONFIGURABLE_RESOURCE_H

namespace argos {
   class CBaseConfigurableResource;
}

#include <argos2/common/utility/configuration/argos_configuration.h>

namespace argos {

   /**
    * This class is the base of all XML-configurable ARGoS interface.
    */
   class CBaseConfigurableResource {

   public:

      /**
       * Class destructor.
       */
      virtual ~CBaseConfigurableResource() {}

      /**
       * Initializes the resource.
       * @param t_tree the base of the XML configuration tree to parse
       * @throw CARGoSException if an error occurs
       */
      virtual void Init(TConfigurationNode& t_tree) = 0;

      /**
       * Undoes whatever was done by Init().
       */
      virtual void Destroy() = 0;

   };

}

#endif

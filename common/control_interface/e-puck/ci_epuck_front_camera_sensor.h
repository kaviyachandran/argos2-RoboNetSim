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
 * @file common/control_interface/epuck/ci_epuck_front_camera_sensor.h
 *
 * @brief This file provides the definition of the e-puck front camera sensor.
 * Which is a modified copy/paste of the FootBot ceiling camera sensor done by Carlo Pinciroli - <cpinciro@ulb.ac.be>.
 *
 * @author Mikael Lenaertz - <miklenae@ulb.ac.be>
 */

#ifndef CI_EPUCK_FRONT_CAMERA_SENSOR_H
#define CI_EPUCK_FRONT_CAMERA_SENSOR_H

namespace argos {
	class CCI_EpuckFrontCameraSensor;
}

#include <argos2/common/control_interface/ci_camera_sensor.h>

namespace argos {

	/**
	 * This class provides the main interface to the front camera on the e-puck.
	 *
	 * @see CCI_CameraSensor
	 */
	class CCI_EpuckFrontCameraSensor: virtual public CCI_CameraSensor {

		public:
			/**
			 * Constructor
			 */
			CCI_EpuckFrontCameraSensor() :
				CCI_CameraSensor() {
			}
			/**
			 * Destructor
			 */
			virtual ~CCI_EpuckFrontCameraSensor() {
			}
	};

}/* namespace argos */

#endif

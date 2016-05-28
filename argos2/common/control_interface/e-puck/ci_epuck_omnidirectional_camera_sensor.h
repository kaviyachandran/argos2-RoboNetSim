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
 * @file common/control_interface/epuck/ci_epuck_omnidirectional_camera_sensor.h
 *
 * @brief This file provides the definition of the e-puck omnidirectional camera sensor.
 * Which is a modified copy/paste of the FootBot omnidirectional camera sensor done by Carlo Pinciroli - <cpinciro@ulb.ac.be>.
 *
 * @author Mikael Lenaertz - <miklenae@ulb.ac.be>
 */

#ifndef CI_EPUCK_OMNIDIRECTIONAL_CAMERA_SENSOR_H
#define CI_EPUCK_OMNIDIRECTIONAL_CAMERA_SENSOR_H

namespace argos {
	class CCI_EPuckOmnidirectionalCameraSensor;
}

#include <argos2/common/control_interface/ci_camera_sensor.h>

namespace argos {

	/**
	 * This class provides the main interface to the omnidirectional camera on the e-puck.
	 *
	 * @see CCI_CameraSensor
	 */
	class CCI_EPuckOmnidirectionalCameraSensor: virtual public CCI_CameraSensor {

		public:
			/**
			 * Constructor
			 */
			CCI_EPuckOmnidirectionalCameraSensor() {}

			/**
			 * Destructor
			 */
			virtual ~CCI_EPuckOmnidirectionalCameraSensor() {}

	};

} /* namespace argos */

#endif

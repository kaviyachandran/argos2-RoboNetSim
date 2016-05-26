/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */
/**
 * @file ci_eyebot_pan_and_tilt_camera.h
 *
 * @date Jun 24, 2010
 * @author Alessandro Stranieri
 *
 * This file defines the common interface for the Pan and Tilt camera installed
 * on the eyebot.
 */

#ifndef CI_EYEBOT_PAN_AND_TILT_CAMERA_H
#define CI_EYEBOT_PAN_AND_TILT_CAMERA_H

namespace argos {
class CCI_EyeBotPanAndTiltCameraSensor;
}

#include <argos2/common/control_interface/ci_camera_sensor.h>
#include <argos2/common/utility/math/vector2.h>

namespace argos {

/**
 * This class defines the common interface to the Pan and Tilt camera installed
 * on the Eyebot.
 *
 * @see CCI_CameraSensor
 */
class CCI_EyeBotPanAndTiltCameraSensor: public CCI_CameraSensor {

public:

	CCI_EyeBotPanAndTiltCameraSensor() {}
	virtual ~CCI_EyeBotPanAndTiltCameraSensor() {}

};

}

#endif

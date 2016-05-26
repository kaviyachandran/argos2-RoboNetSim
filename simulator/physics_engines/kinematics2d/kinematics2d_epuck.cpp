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
 * @file <argos2/simulator/physics_engines/kinematics2d_epuck.cpp>
 *
 * @author Vito Trianni - <vito.trianni@istc.cnr.it>
 */

#include "kinematics2d_epuck.h"
#include <argos2/simulator/simulator.h>
#include <argos2/simulator/space/space.h>
#include <argos2/simulator/physics_engines/kinematics2d/kinematics2d_engine.h>
#include <argos2/simulator/space/entities/embodied_entity.h>
namespace argos {

    /****************************************/
    /****************************************/

    static const Real EPUCK_RADIUS = 0.035f;
    static const Real EPUCK_INTERWHEEL_DISTANCE = 0.053f;
    static const Real EPUCK_HEIGHT = 0.086f; // to be checked!

    enum EPUCK_WHEELS {

        EPUCK_LEFT_WHEEL = 0,
        EPUCK_RIGHT_WHEEL = 1
    };

    /****************************************/

    /****************************************/

    CKinematics2DEPuck::CKinematics2DEPuck(CKinematics2DEngine& c_engine,
            CEPuckEntity& c_epuck) :
    CKinematics2DCollisionCircle(c_engine, c_epuck.GetEmbodiedEntity(), EPUCK_RADIUS),
    m_cEPuckEntity(c_epuck),
    m_cWheeledEntity(c_epuck.GetWheeledEntity()) {

        m_fCurrentWheelVelocityFromSensor[EPUCK_LEFT_WHEEL] = 0.0f;
        m_fCurrentWheelVelocityFromSensor[EPUCK_RIGHT_WHEEL] = 0.0f;
    }

    /****************************************/

    /****************************************/

    void CKinematics2DEPuck::UpdateEntityStatus() {
        /* update entity position and orientation */
        CKinematics2DEntity::UpdateEntityStatus();

        /* Update foot-bot components */
        m_cEPuckEntity.UpdateComponents();
    }

    /****************************************/

    /****************************************/

    void CKinematics2DEPuck::UpdateFromEntityStatus() {
        /* Store previous position and orientation for collision avoidance */
        CKinematics2DEntity::UpdateFromEntityStatus();

        /* Get wheel speeds from entity */
        m_cWheeledEntity.GetSpeed(m_fCurrentWheelVelocityFromSensor);

        /* Do we want to move? */
        if ((m_fCurrentWheelVelocityFromSensor[EPUCK_LEFT_WHEEL] != 0.0f) ||
                (m_fCurrentWheelVelocityFromSensor[EPUCK_RIGHT_WHEEL] != 0.0f)) {
            /* Yeah, we do */

            /**
             *
             * THE DIFFERENTIAL STEERING SYSTEM
             *
             * The differential drive motion can be simulated according to
             * the following equations:
             *
             * 	dx/dt = [(Vl + Vr)/2] * r * cos( a )
             * 	dy/dt = [(Vl + Vr)/2] * r * sin( a )                 (1),
             * 	da/dt = [Vr - Vl] * r / L
             *
             * where x,y are the absolute coordinates in the world reference
             * frame, Vr and Vl are the angular speed of the left and right
             * wheels, r is the radius of the wheels and L is the distance
             * between the centers of the wheels.
             *
             * Integrating (1) we obtain:
             *
             * 	x(t) = x(t-1) + dt * [(Vl + Vr)/2] * r * cos( a )
             * 	y(t) = y(t-1) + dt * [(Vl + Vr)/2] * r * sin( a )    (2),
             * 	a(t) = a(t-1) + dt * [Vr - Vl] * r / L
             *
             **/

            /* Update position and heading according to the physics equations */
            m_cPosition += (CVector2((m_fCurrentWheelVelocityFromSensor[EPUCK_RIGHT_WHEEL] + m_fCurrentWheelVelocityFromSensor[EPUCK_LEFT_WHEEL]) * CPhysicsEngine::GetSimulationClockTick() / 2.0, 0).Rotate(m_cOrientation));
            m_cOrientation += CRadians((m_fCurrentWheelVelocityFromSensor[EPUCK_RIGHT_WHEEL] - m_fCurrentWheelVelocityFromSensor[EPUCK_LEFT_WHEEL]) /
                    EPUCK_INTERWHEEL_DISTANCE * CPhysicsEngine::GetSimulationClockTick());
        }
    }

    /****************************************/
    /****************************************/

    /****************************************/

    /****************************************/

    bool CKinematics2DEPuck::CheckIntersectionWithRay(Real& f_distance, const CRay& c_ray) const {

        if (CKinematics2DCollisionCircle::CheckIntersectionWithRay(f_distance, c_ray)) {
            CVector3 cIntersectionPoint;
            c_ray.GetPoint(cIntersectionPoint, f_distance);
            if ((cIntersectionPoint.GetZ() >= GetEmbodiedEntity().GetPosition().GetZ()) &&
                    (cIntersectionPoint.GetZ() <= GetEmbodiedEntity().GetPosition().GetZ() + EPUCK_HEIGHT)) {
      //          LOG<<"ECCO!!!TRUE\n";
                return true;
            }
        }
        //
    //    LOG<<"ECCO!!!FALSE\n";
        //
        return false;
    }





}

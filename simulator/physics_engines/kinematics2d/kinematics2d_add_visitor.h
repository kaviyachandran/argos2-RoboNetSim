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
 * @file <argos2/simulator/physics_engines/kinematics2d/kinematics2d_add_visitor.h>
 *
 * @author Vito Trianni - <vito.trianni@istc.cnr.it>
 */

#ifndef KINEMATICS2D_ADD_VISITOR_H
#define KINEMATICS2D_ADD_VISITOR_H

namespace argos {
   class CEntity;
   class CKinematics2DEngine;
}

#include <argos2/simulator/space/entities/entity_visitor.h>
#include <argos2/common/utility/configuration/argos_exception.h>

namespace argos {

   class CKinematics2DAddVisitor : public CEntityVisitor {

   public:

      CKinematics2DAddVisitor(CKinematics2DEngine& c_engine) :
         m_cEngine(c_engine) {}
      virtual ~CKinematics2DAddVisitor() {}

      virtual void Visit(CBoothEntity& c_entity) {THROW_ARGOSEXCEPTION("adding floor entity not implemented");}
      virtual void Visit(CBoxEntity& c_entity);
      virtual void Visit(CCylinderEntity& c_entity);
      virtual void Visit(CTileEntity& c_entity) {}
      virtual void Visit(CBluebotEntity& c_entity) {}
      virtual void Visit(CFloorEntity& c_entity) {THROW_ARGOSEXCEPTION("adding floor entity not implemented");}
      virtual void Visit(CFootBotEntity& c_entity);
      virtual void Visit(CEyeBotEntity& c_entity);
      virtual void Visit(CEPuckEntity& c_entity);
      virtual void Visit(CLightEntity& c_entity) {THROW_ARGOSEXCEPTION("adding light entity not implemented");}
      virtual void Visit(CWiFiMediumEntity& c_entity) {THROW_ARGOSEXCEPTION("adding wifi medium entity not implemented");}

   private:

      CKinematics2DEngine& m_cEngine;
       
   };
}

#endif


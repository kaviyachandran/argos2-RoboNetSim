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
 * @file <simulator/space/entities/tile_entity.h>
 *
 * @author Carlo Pinciroli - <cpinciro@ulb.ac.be>
 * @author Dhananjay Ipparthi - <ipparthi@gmail.com>
 */

#ifndef TILE_ENTITY_H
#define TILE_ENTITY_H

namespace argos {
   class CTileEntity;
}

#include <argos2/simulator/space/entities/embodied_entity.h>
#include <argos2/common/utility/math/matrix3x3.h>
#include <argos2/common/utility/math/range.h>

namespace argos {

   class CTileEntity : public CEmbodiedEntity {

   public:

      CTileEntity();
      virtual ~CTileEntity();

      virtual void Init(TConfigurationNode& t_tree);

      inline const CVector3& GetSize() const {
         return m_cSize;
      }

      inline void SetSize(const CVector3& c_size) {
         m_cSize = c_size;
      }

      inline Real GetMass() const {
         return m_fMass;
      }

      inline void SetMass(Real f_mass) {
         m_fMass = f_mass;
      }

      inline const CRange<Real>& GetPushRange() const {
         return m_cPushRange;
      }

      inline void SetPushRange(const CRange<Real>& c_range) {
         m_cPushRange = c_range;
      }

      inline virtual std::string GetTypeDescription() const {
         return "tile_entity";
      }

      inline virtual void Accept(CEntityVisitor& visitor) {
         visitor.Visit(*this);
      }

      virtual void CalculateBoundingBox();

   private:

      CVector3 m_cSize;
      Real m_fMass;
      CMatrix3x3 m_cOrientationMatrix;
      CRange<Real>  m_cPushRange;

   };

}

#endif

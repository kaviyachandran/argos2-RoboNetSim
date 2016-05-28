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
 * @file argos2/common/utility/math/matrix2x2.cpp
 *
 * The matrix goes like this:
 *
 * | 0 1 |
 * | 2 3 |
 *
 * @author Carlo Pinciroli - <cpinciro@ulb.ac.be>
 * @author Vito Trianni - <vtrianni@ulb.ac.be>
 */

#include "matrix2x2.h"
#include "vector2.h"

namespace argos {

   /****************************************/
   /****************************************/

   CMatrix2x2 CMatrix2x2::IDENTITY;

   /****************************************/
   /****************************************/

   CVector2 CMatrix2x2::operator[](UInt32 un_row) const {
      ARGOS_ASSERT(un_row < 2,
                   "Matrix index out of bounds: un_row = " <<
                   un_row);
      UInt32 unRowBase = un_row * 2;
      return CVector2(m_fValues[unRowBase    ],
                      m_fValues[unRowBase + 1]);
   }

   /****************************************/
   /****************************************/

   CMatrix2x2& CMatrix2x2::FromAngle(const CRadians& c_angle) {
      /* This code is adapted from the Bullet source code */
      Real cos_angle = Cos(c_angle);
      Real sin_angle = Sin(c_angle);

      /* Set values */
      m_fValues[0] =  cos_angle;
      m_fValues[1] = -sin_angle;
      m_fValues[2] =  sin_angle;
      m_fValues[3] =  cos_angle;
      return *this;
   }

   /****************************************/
   /****************************************/

   CVector2 CMatrix2x2::operator*(const CVector2& c_vector) const {
      CVector2 cResult(m_fValues[0]*c_vector.GetX() + m_fValues[1]*c_vector.GetY(),
		       m_fValues[2]*c_vector.GetX() + m_fValues[3]*c_vector.GetY());
      return cResult;
   }

   /****************************************/
   /****************************************/

}

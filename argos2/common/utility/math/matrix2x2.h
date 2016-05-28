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
 * @file argos2/common/utility/math/matrix2x2.h
 *
 * The matrix goes like this:
 *
 * | 0 1 |
 * | 2 3 |
 *
 * @author Carlo Pinciroli - <cpinciro@ulb.ac.be>
 * @author Vito Trianni - <vtrianni@ulb.ac.be>
 */

#ifndef MATRIX2X2_H
#define MATRIX2X2_H

namespace argos {
   class CMatrix2x2;
   class CRadians;
   class CVector2;
}

#include <argos2/common/utility/math/general.h>
#include <argos2/common/utility/math/angles.h>
#include <cmath>

namespace argos {

   class CMatrix2x2 {

   public:

      static CMatrix2x2 IDENTITY;

   public:

      CMatrix2x2() {
         m_fValues[0] = 1.0f; m_fValues[1] = 0.0f; 
         m_fValues[2] = 0.0f; m_fValues[3] = 1.0f; 
      }

      CMatrix2x2(const Real* f_values) {
         Set(f_values);
      }

      CMatrix2x2(const Real& f_value0, const Real& f_value1, 
		 const Real& f_value2, const Real& f_value3 ) {
         Set(f_value0, f_value1, 
	     f_value2, f_value3);
      }

      CMatrix2x2(const CRadians& c_angle) {
         FromAngle(c_angle);
      }

      CMatrix2x2(const CMatrix2x2& c_matrix) {
         Set(c_matrix.m_fValues);
      }

      CMatrix2x2& operator=(const CMatrix2x2& c_matrix) {
         if(this != &c_matrix) {
            Set(c_matrix.m_fValues);
         }
         return *this;
      }

      inline void Set(const Real* f_values) {
         m_fValues[0] = f_values[0]; m_fValues[1] = f_values[1]; 
	 m_fValues[2] = f_values[2]; m_fValues[3] = f_values[3]; 
      }

      inline void Set(const Real& f_value0, const Real& f_value1, 
		      const Real& f_value2, const Real& f_value3) {
         m_fValues[0] = f_value0; m_fValues[1] = f_value1; 
	 m_fValues[2] = f_value2; m_fValues[3] = f_value3;
      }

      inline Real operator()(UInt32 un_row,
                             UInt32 un_col) const {
         ARGOS_ASSERT(un_row < 2 && un_col < 2,
                      "Matrix index out of bounds: un_row = " <<
                      un_row <<
                      ", un_col = " <<
                      un_col);
         return m_fValues[un_row * 2 + un_col];
      }

      inline Real& operator()(UInt32 un_row,
                              UInt32 un_col) {
         ARGOS_ASSERT(un_row < 2 && un_col < 2,
                      "Matrix index out of bounds: un_row = " <<
                      un_row <<
                      ", un_col = " <<
                      un_col);
         return m_fValues[un_row * 2 + un_col];
      }

      inline Real operator()(UInt32 un_idx) const {
         ARGOS_ASSERT(un_idx < 4,
                      "Matrix index out of bounds: un_idx = " <<
                      un_idx);
         return m_fValues[un_idx];
      }

      inline Real& operator()(UInt32 un_idx) {
         ARGOS_ASSERT(un_idx < 4,
                      "Matrix index out of bounds: un_idx = " <<
                      un_idx);
         return m_fValues[un_idx];
      }

      CVector2 operator[](UInt32 un_row) const;

      CMatrix2x2& FromAngle(const CRadians& c_angle);

      inline bool operator==(const CMatrix2x2& c_matrix) const {
         return
            m_fValues[0] == c_matrix.m_fValues[0] &&
            m_fValues[1] == c_matrix.m_fValues[1] &&
            m_fValues[2] == c_matrix.m_fValues[2] &&
            m_fValues[3] == c_matrix.m_fValues[3];
      }
      
      inline CMatrix2x2& operator+=(const CMatrix2x2& c_matrix) {
         m_fValues[0] += c_matrix.m_fValues[0];
         m_fValues[1] += c_matrix.m_fValues[1];
         m_fValues[2] += c_matrix.m_fValues[2];
         m_fValues[3] += c_matrix.m_fValues[3];
         return *this;
      }

      inline CMatrix2x2& operator-=(const CMatrix2x2& c_matrix) {
         m_fValues[0] -= c_matrix.m_fValues[0];
         m_fValues[1] -= c_matrix.m_fValues[1];
         m_fValues[2] -= c_matrix.m_fValues[2];
         m_fValues[3] -= c_matrix.m_fValues[3];
         return *this;
      }

      inline CMatrix2x2& operator*=(const CMatrix2x2& c_matrix) {
         Real fNewValues[9];
         /* Calculate new values */
         fNewValues[0] =
            m_fValues[0] * c_matrix.m_fValues[0] + 
            m_fValues[1] * c_matrix.m_fValues[2];
         fNewValues[1] =
            m_fValues[0] * c_matrix.m_fValues[1] + 
            m_fValues[1] * c_matrix.m_fValues[3];
         fNewValues[2] =
            m_fValues[2] * c_matrix.m_fValues[0] + 
            m_fValues[3] * c_matrix.m_fValues[2];
         fNewValues[3] =
            m_fValues[2] * c_matrix.m_fValues[1] + 
            m_fValues[3] * c_matrix.m_fValues[3];
         /* Copy them into m_fValues */
         Set(fNewValues);
         return *this;
      }

      inline CMatrix2x2& operator*=(Real f_scale) {
         m_fValues[0] *= f_scale;
         m_fValues[1] *= f_scale;
         m_fValues[2] *= f_scale;
         m_fValues[3] *= f_scale;
         return *this;
      }

      inline CMatrix2x2 operator+(const CMatrix2x2& c_matrix) const {
         CMatrix2x2 cResult = (*this);
         cResult += c_matrix;
         return cResult;
      }

      inline CMatrix2x2 operator-(const CMatrix2x2& c_matrix) const {
         CMatrix2x2 cResult = (*this);
         cResult -= c_matrix;
         return cResult;
      }

      inline CMatrix2x2 operator*(const CMatrix2x2& c_matrix) const {
         CMatrix2x2 cResult = (*this);
         cResult *= c_matrix;
         return cResult;
      }

      CVector2 operator*(const CVector2& c_vector) const;

      inline CMatrix2x2 operator*(Real f_scale) const {
         CMatrix2x2 cResult = (*this);
         cResult *= f_scale;
         return cResult;
      }

      friend std::ostream& operator<<(std::ostream& c_os,
                                      const CMatrix2x2& c_matrix) {
         c_os << "| " << c_matrix(0) << " " << c_matrix(1) << " |" << "<br>"
              << "| " << c_matrix(2) << " " << c_matrix(3) << " |" << "<br>";
         return c_os;
      }

   private:

      Real m_fValues[4];

   };

}

#endif

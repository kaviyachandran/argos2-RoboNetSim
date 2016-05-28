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
 * @file argos2/common/utility/configuration/memento.h
 *
 * @brief This file provides the definition of the memento interface.
 *
 * The <a href="http://en.wikipedia.org/wiki/Memento_pattern">Memento pattern</a>
 * is a software design pattern that provides the ability to restore an object to
 * its previous state.
 *
 * @author Carlo Pinciroli - <cpinciro@ulb.ac.be>
 */

#ifndef MEMENTO_H
#define MEMENTO_H

namespace argos {
   class CMemento;
}

#include <argos2/common/utility/datatypes/byte_array.h>

namespace argos {

   /**
    * The interface inherited by all classes whose state must be saved and restored.
    */
   class CMemento {

   public:

      /**
       * Class destructor.
       */
      virtual ~CMemento() {}

      /**
       * Saves the state of the object to the given buffer.
       * Data is added to the buffer.
       * @param c_buffer the target buffer
       */
      virtual void SaveState(CByteArray& c_buffer) = 0;

      /**
       * Restores the state of the object from the given buffer.
       * The read data is removed from the buffer.
       * @param c_buffer the source buffer
       */
      virtual void LoadState(CByteArray& c_buffer) = 0;

   };

}

#endif

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
 * @file argos2/common/utility/logging/argos_log.h
 *
 * @brief This file redirects compilation to the single-thread or the thread-safe version of the ARGoS log.
 *
 * @author Carlo Pinciroli - <cpinciro@ulb.ac.be>
 */

#ifdef CROSSCOMPILING
#include <argos2/common/utility/logging/argos_log_no_threads.h>
#else
# include <argos2/common/utility/logging/argos_log_thread_safe.h>
#endif

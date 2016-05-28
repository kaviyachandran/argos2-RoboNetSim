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
 * @file argos2/simulator/argos_command_line_arg_parser.h
 *
 * @author Carlo Pinciroli - <cpinciro@ulb.ac.be>
 */

#ifndef ARGOS_COMMAND_LINE_ARG_PARSER_H
#define ARGOS_COMMAND_LINE_ARG_PARSER_H

namespace argos {
   class CARGoSCommandLineArgParser;
}

#include <argos2/common/utility/configuration/command_line_arg_parser.h>
#include <argos2/simulator/simulator.h>

namespace argos {

   class CARGoSCommandLineArgParser : public CCommandLineArgParser {

   public:

      enum EAction {
         ACTION_UNKNOWN = 0,
         ACTION_SHOW_HELP,
         ACTION_RUN_EXPERIMENT,
         ACTION_QUERY
      };

   public:

      CARGoSCommandLineArgParser(CSimulator& c_simulator);
      virtual ~CARGoSCommandLineArgParser() {}

      virtual void PrintUsage(CARGoSLog& c_log);

      virtual void Parse(SInt32 n_argc,
                         char** ppch_argv);

      inline EAction GetAction() {
         return m_eAction;
      }

      inline const std::string& GetExperimentConfigFile() {
         return m_strExperimentConfigFile;
      }

      inline const std::string& GetQuery() {
         return m_strQuery;
      }

      inline bool IsLogColored() {
         return ! m_bNonColoredLog;
      }

      inline bool IsHelpWanted() {
         return m_bHelpWanted;
      }

   private:

      CSimulator& m_cSimulator;
      EAction m_eAction;
      std::string m_strExperimentConfigFile;
      std::string m_strQuery;
      bool m_bNonColoredLog;
      bool m_bHelpWanted;

   };

}

#endif

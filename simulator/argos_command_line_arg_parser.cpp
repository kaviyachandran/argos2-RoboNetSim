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
 * @file argos2/simulator/argos_command_line_arg_parser.cpp
 *
 * @author Carlo Pinciroli - <cpinciro@ulb.ac.be>
 */

#include "argos_command_line_arg_parser.h"

namespace argos {

   /****************************************/
   /****************************************/

   CARGoSCommandLineArgParser::CARGoSCommandLineArgParser(CSimulator& c_simulator) :
      m_cSimulator(c_simulator),
      m_eAction(ACTION_UNKNOWN) {
      AddFlag(
         'h',
         "help",
         "display this usage information",
         m_bHelpWanted
         );
      AddFlag(
         'n',
         "no-color",
         "do not use colored output [OPTIONAL]",
         m_bNonColoredLog
         );
      AddArgument<std::string>(
         'c',
         "config-file",
         "the experiment XML configuration file",
         m_strExperimentConfigFile
         );
      AddArgument<std::string>(
         'q',
         "query",
         "query the available plugins",
         m_strQuery
         );
   }

   /****************************************/
   /****************************************/

   void CARGoSCommandLineArgParser::Parse(SInt32 n_argc,
                                          char** ppch_argv) {
      CCommandLineArgParser::Parse(n_argc, ppch_argv);
      /* Configure LOG/LOGERR coloring */
      if(m_bNonColoredLog) {
         LOG.DisableColoredOutput();
         LOGERR.DisableColoredOutput();
      }

      /* Check that either -h, -c or -q was passed (strictly one of them) */
      if(m_strExperimentConfigFile == "" &&
         m_strQuery == ""                &&
         ! m_bHelpWanted) {
         THROW_ARGOSEXCEPTION("No --help, --config-file or --query options specified.");
      }
      if((m_strExperimentConfigFile != "" && m_strQuery != "") ||
         (m_strExperimentConfigFile != "" && m_bHelpWanted) ||
         (m_strQuery != "" && m_bHelpWanted)) {
         THROW_ARGOSEXCEPTION("Options --help, --config-file and --query are mutually exclusive.");
      }

      if(m_strExperimentConfigFile != "") {
         m_eAction = ACTION_RUN_EXPERIMENT;
      }

      if(m_strQuery != "") {
         m_eAction = ACTION_QUERY;
      }

      if(m_bHelpWanted) {
         m_eAction = ACTION_SHOW_HELP;
      }

   }

   /****************************************/
   /****************************************/

   void CARGoSCommandLineArgParser::PrintUsage(CARGoSLog& c_log) {
      c_log << "Usage: argos [OPTIONS]" << std::endl;
      c_log << "The ARGOS simulator, the official simulator of the Swarmanoid Project." << std::endl;
      c_log << "Current version: 2.0" << std::endl;
      c_log << std::endl;
      c_log << "   -h        | --help                  display this usage information" << std::endl;
      c_log << "   -c FILE   | --config-file FILE      the experiment XML configuration file" << std::endl;
      c_log << "   -q QUERY  | --query QUERY           query the available plugins." << std::endl;
      c_log << "   -n        | --no-color FILE         do not use colored output [OPTIONAL]" << std::endl << std::endl;
      c_log << "The options --config-file and --query are mutually exclusive. Either you use" << std::endl;
      c_log << "the first, and thus you run an experiment, or you use the second to query the" << std::endl;
      c_log << "plugins." << std::endl << std::endl;
      c_log << "EXAMPLES" << std::endl << std::endl;
      c_log << "To run an experiment, type:" << std::endl << std::endl;
      c_log << "   ./argos -c /path/to/myconfig.xml" << std::endl << std::endl;
      c_log << "To query the plugins, type:" << std::endl << std::endl;
      c_log << "   ./argos -q QUERY" << std::endl << std::endl;
      c_log << "where QUERY can have the following values:" << std::endl << std::endl;
      c_log << "   all                    print a list of all the available plugins" << std::endl;
      c_log << "   actuators              print a list of all the available actuators" << std::endl;
      c_log << "   sensors                print a list of all the available sensors" << std::endl;
      c_log << "   physics_engines        print a list of all the available physics engines" << std::endl;
      c_log << "   visualizations         print a list of all the available visualizations" << std::endl;
      c_log << "   entities               print a list of all the available entities" << std::endl << std::endl;
      c_log << "Alternatively, QUERY can be the name of a specific plugin as returned by the" << std::endl;
      c_log << "above commands. In this case, you get a complete description of the matching" << std::endl;
      c_log << "plugins." << std::endl << std::endl;
   }

   /****************************************/
   /****************************************/

}

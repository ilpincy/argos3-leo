#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/core/utility/configuration/command_line_arg_parser.h>
#include <argos3/core/utility/logging/argos_log.h>
#include <argos3/plugins/robots/leo/real_robot/real_leo.h>

using namespace argos;

int main(int argc, char* argv[]) {
   /*
    * Parse the command line
    */
   std::string strARGoSFName;
   std::string strControllerId;
   CCommandLineArgParser cCLAP;
   cCLAP.AddArgument<std::string>(
      'c',
      "--config-file",
      "the experiment XML configuration file",
      strARGoSFName);
   cCLAP.AddArgument<std::string>(
      'i',
      "--controller-id",
      "the id of the controller to run",
      strControllerId);
   try {
      cCLAP.Parse(argc, argv);
      DEBUG("Parsed the parameters\n");
      DEBUG("  strARGoSFName = '%s'\n", strARGoSFName.c_str());
      DEBUG("  strControllerId = '%s'\n", strControllerId.c_str());
      /*
       * Initialize the robot
       */
      if(strControllerId == "") {
         THROW_ARGOSEXCEPTION("Missing controller id, please add -i <id>");
      }
      CRealLeo* pcRobot = new CRealLeo();
      pcRobot->Init(strARGoSFName, strControllerId);
      /*
       * Perform the main loop
       */
      pcRobot->Execute();
   }
   catch(CARGoSException& ex) {
      /* A fatal error occurred: dispose of data, print error and exit */
      LOGERR << ex.what() << std::endl;
      LOG.Flush();
      LOGERR.Flush();
      return 1;
   }
   /* All done (should never get here) */
   return 0;
}

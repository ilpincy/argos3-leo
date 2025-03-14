#ifndef LEO_TEST_CONTROLLER_H
#define LEO_TEST_CONTROLLER_H

/* Definition of the CCI_Controller class. */
#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/plugins/robots/leo/control_interface/ci_leo_posetwist_actuator.h>
#include <argos3/plugins/robots/leo/control_interface/ci_leo_odometry_sensor.h>
#include <argos3/plugins/robots/leo/control_interface/ci_leo_ar_tag_sensor.h>
#include <argos3/plugins/robots/leo/control_interface/ci_leo_navigation_actuator.h>
#include <argos3/plugins/robots/leo/control_interface/ci_leo_localization_sensor.h>

using namespace argos;

class CLeoTestController : public CCI_Controller {

public:
   /* Class constructor. */
   CLeoTestController();

   /* Class destructor. */
   virtual ~CLeoTestController() {}
   /*
    * This function initializes the controller.
    * The 't_node' variable points to the <parameters> section in the XML
    * file in the <controllers><footbot_diffusion_controller> section.
    */
   virtual void Init(TConfigurationNode& t_node);

   /*
    * This function is called once every time step.
    * The length of the time step is set in the XML file.
    */
   virtual void ControlStep();

   /*
    * This function resets the controller to its state right after the
    * Init().
    * It is called when you press the reset button in the GUI.
    * In this example controller there is no need for resetting anything,
    * so the function could have been omitted. It's here just for
    * completeness.
    */
   virtual void Reset() {}

   /*
    * Called to cleanup what done by Init() when the experiment finishes.
    * In this example controller there is no need for clean anything up,
    * so the function could have been omitted. It's here just for
    * completeness.
    */
   virtual void Destroy() {}

   private:

    CCI_LeoPoseTwistActuator* leoPoseTwistActuator;
    CCI_LeoOdometrySensor* leoOdometrySensor;
    CCI_LeoArTagSensor* leoArTagSensor;

    CCI_LeoNavigationActuator* leoNavigationActuator;
    CCI_LeoLocalizationSensor* leoLocalizationSensor;

};

#endif

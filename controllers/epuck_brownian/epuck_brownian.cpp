/* Include the controller definition */
#include "epuck_brownian.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/core/utility/logging/argos_log.h>

#include <cmath>
#include <string>
#include <sstream>
#include <fstream>
#include <iostream>

/****************************************/
/****************************************/

CEPuckBrownian::CEPuckBrownian() :
   m_pcWheels(NULL),
   m_pcProximity(NULL),
   m_fWheelVelocity(2.5f) {}

/****************************************/
/****************************************/

void CEPuckBrownian::Init(TConfigurationNode& t_node) {
   /*
    * Get sensor/actuator handles
    *
    * The passed string (ex. "differential_steering") corresponds to the
    * XML tag of the device whose handle we want to have. For a list of
    * allowed values, type at the command prompt:
    *
    * $ argos3 -q actuators
    *
    * to have a list of all the possible actuators, or
    *
    * $ argos3 -q sensors
    *
    * to have a list of all the possible sensors.
    *
    * NOTE: ARGoS creates and initializes actuators and sensors
    * internally, on the basis of the lists provided the configuration
    * file at the <controllers><epuck_brownian><actuators> and
    * <controllers><epuck_brownian><sensors> sections. If you forgot to
    * list a device in the XML and then you request it here, an error
    * occurs.
    */
   m_pcWheels    = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
   m_pcProximity = GetSensor  <CCI_ProximitySensor             >("proximity"    );
   m_pcPosition  = GetSensor  <CCI_PositioningSensor           >("positioning");
   m_pcLight     = GetSensor  <CCI_LightSensor                 >("light");
   m_pcRABA      = GetActuator<CCI_RangeAndBearingActuator     >("range_and_bearing");
   m_pcRABS      = GetSensor  <CCI_RangeAndBearingSensor       >("range_and_bearing");

   /*
    * Parse the configuration file
    *
    * The user defines this part. Here, the algorithm accepts three
    * parameters and it's nice to put them in the config file so we don't
    * have to recompile if we want to try other settings.
    */
   GetNodeAttributeOrDefault(t_node, "velocity", m_fWheelVelocity, m_fWheelVelocity);

   std::cout << "Init() called" << std::endl;

   goal_x = 4.4;
   goal_y = 1.3;

   ticks = 0;
   
   m_pcRABA->SetData(0, FUNCTIONING);
}

/****************************************/
/****************************************/

void CEPuckBrownian::ControlStep()
{
  // TODO: Implement ControlStep().
}

void CEPuckBrownian::Destroy() {
  std::cout << "Destroy() called" << std::endl;
}

/****************************************/
/****************************************/

/*
 * This statement notifies ARGoS of the existence of the controller.
 * It binds the class passed as first argument to the string passed as
 * second argument.
 * The string is then usable in the configuration file to refer to this
 * controller.
 * When ARGoS reads that string in the configuration file, it knows which
 * controller class to instantiate.
 * See also the configuration files for an example of how this is used.
 */
REGISTER_CONTROLLER(CEPuckBrownian, "epuck_brownian_controller")

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

   goal_x = 0.0;
   goal_y = 0.0;

   ticks = 0;
   state_ticks = 0;
   tick_wander = 15 * 10;
   tick_turn = (int) (((double) rand() / RAND_MAX) * 5 * 10);

   goal_found = false;
   m_pcRABA->SetData(0, GOAL_NOT_FOUND);
}

/****************************************/
/****************************************/

void CEPuckBrownian::ControlStep()
{
  if (!goal_found)
  {
    double x = m_pcPosition->GetReading().Position.GetX();
    double y = m_pcPosition->GetReading().Position.GetY();

    if (sqrt((x - goal_x) * (x - goal_x) + (y - goal_y) * (y - goal_y)) < 0.05)
    {
      /* Goal found - stop moving and notify the other robots that it has
         found the goal. */
      m_pcWheels->SetLinearVelocity(0, 0);
      m_pcRABA->SetData(0, GOAL_FOUND);
      goal_found = true;
      argos::LOG << "Goal discovered!" << std::endl;
      return;
    }

    /* Otherwise, check to see if at least one other robot has found the goal
       already.  If so, move toward the closest such robot. */
    bool other_goal_found = false;
    double range = 0.0;
    double horiz_bearing = 0.0;

    const CCI_RangeAndBearingSensor::TReadings& tPackets = m_pcRABS->GetReadings();

    for (size_t i = 0; i < tPackets.size(); i++)
    {
      if (tPackets[i].Data[0] == GOAL_FOUND)
      {
        if (!other_goal_found)
        {
          range = tPackets[i].Range;
          horiz_bearing = tPackets[i].HorizontalBearing.GetValue();
          other_goal_found = true;
        }
        else if (tPackets[i].Range < range)
        {
          range = tPackets[i].Range;
          horiz_bearing = tPackets[i].HorizontalBearing.GetValue();
        }
      }
    }

    if (other_goal_found)
    {
      /* First, check to see how closely we are facing the robot at the goal -
         if need be, turn until facing it. */

      if (fabs(horiz_bearing) > 0.1)
      {
        m_pcWheels->SetLinearVelocity(m_fWheelVelocity, 0);
      }
      else
      {
        /* Move toward the robot at the goal. */
        m_pcWheels->SetLinearVelocity(m_fWheelVelocity, m_fWheelVelocity);
      }
    }
    else
    {
      if (state_ticks < tick_wander)
      {
        /* Simply move straight. */
        m_pcWheels->SetLinearVelocity(m_fWheelVelocity, m_fWheelVelocity);
      }
      else if (state_ticks < (tick_wander + tick_turn))
      {
        /* Turn to a random orientation. */
        m_pcWheels->SetLinearVelocity(m_fWheelVelocity, 0.0);
      }
      else
      {
        state_ticks = 0;
        tick_turn = (int) (((double) rand() / RAND_MAX) * 5 * 10);
      }

      ticks++;
      state_ticks++;
    }
  }
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

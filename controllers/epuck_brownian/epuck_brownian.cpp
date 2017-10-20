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
  GetNodeAttribute(t_node, "avoid_radius_init", avoid_radius_init);
  GetNodeAttribute(t_node, "avoid_radius_light", avoid_radius_light);
  GetNodeAttribute(t_node, "omega_ticks", omega_ticks);
  GetNodeAttribute(t_node, "ticks_to_failure", ticks_to_failure);

  std::cout << "Init() called" << std::endl;

  /*
   * Based on the ID string of this robot (itself set via the argos experiment
   * file) set the failure type.  This will take effect after ticks_to_failure
   * ticks have elapsed in the simulation.
   * 
   * If the first character in the ID string is an "f", this robot is perfectly
   * functional, and will experience no failure after ticks_to_failure ticks
   * have elapsed.
   *
   * If the first character in the ID string is a "p", this robot will
   * experience "power failure", i.e., no functional actuators or sensors.  In
   * the simulation, after ticks_to_failure ticks have elapsed, this robot will
   * become stationary and will not be detected by other robots when they
   * compute the center of the swarm (due to the "emitter" being turned off).
   * The stationary robot will still be avoided by other robots using their
   * "proximity sensors", here analogous to the range-and-bearing sensors, as
   * usual.
   */

  std::string id = GetId();

  switch (id[0])
  {
    case 'f' : failure_type = FUNCTIONING;
               break;
    case 'p' : failure_type = POWER_FAILURE;
               break;
    default  : std::cout << "ERROR: UNKNOWN FAILURE TYPE";
  }

  ticks_since_last_avoidance = 0;
  ticks_since_start = 0;
  
  /*
   * Until ticks_to_failure seconds elapse, function normally, and let the other
   * robots know that we are functioning normally.
   */

  current_type = FUNCTIONING;

  m_pcRABA->SetData(0, FUNCTIONING);
}

/****************************************/
/****************************************/

void CEPuckBrownian::ControlStep()
{
  /*
   * Has the necessary amount of time passed for this robot to "fail"?
   */

  if (ticks_since_start == ticks_to_failure)
  {
    /*
     * If so, set this robot's current failure type to the corresponding failure
     * type, and broadcast this robot's current type to the other robots using
     * the range and bearing actuator (which will be used in FunctioningStep()
     * to ignore robots that have experienced power failure when estimating the
     * center of the swarm, due to the fact that their infrared emitters are
     * deactivated).
     */

    m_pcRABA->SetData(0, failure_type);
    current_type = failure_type;
  }

  /*
   * Based on the current failure type of the robot, call the step function
   * corresponding to that failure type, so that malfunctioning robots can
   * have completely unique behavior to that of functioning robots.
   */

  switch (current_type)
  {
    case FUNCTIONING   : FunctioningStep();
                         break;
    case POWER_FAILURE : PowerFailureStep();
                         break;
    default            : std::cout << "ERROR: UNKNOWN FAILURE TYPE";
  }

  ticks_since_start++;
}

/****************************************/
/****************************************/

void CEPuckBrownian::FunctioningStep()
{
  /*
   * First, determine the radius of avoidance of our current robot, which
   * depends on whether or not we detect light from the beacon source.
   */

  double avoid_radius = avoid_radius_init;
  double PI = 3.14159265358979323;

  /*
   * Read in data from the light sensors - if any of our readings have value
   * greater than 0, light is detected.
   */

  const std::vector<Real>& light_readings = m_pcLight->GetReadings();

  bool light_detected = false;

  for (size_t i = 0; i < light_readings.size(); i++)
  {
    if (light_readings[i] > 0.0)
    {
      light_detected = true;
    }
  }

  /*
   * If light is detected, set our avoidance radius higher.
   */

  if (light_detected)
  {
    avoid_radius = avoid_radius_light;
  }

  /* 
   * Next, check to see if there is a robot in front of us.  Do so using the
   * range-and-bearing sensor rather than the proximity sensor, so as to allow
   * for a larger short-term range than that of the e-puck proximity sensor -
   * whose range is not adjustable.
   */

  const CCI_RangeAndBearingSensor::TReadings& proximity_readings = m_pcRABS->GetReadings();

  bool robot_found = false;
  double min_range;
  double min_hbearing;

  for (size_t i = 0; i < proximity_readings.size(); i++)
  {
    /*
     * Is this robot within our avoidance radius?
     */

    double range = proximity_readings[i].Range;
    if (range < avoid_radius)
    {
      /*
       * Is this robot in front of us?
       */

      double hbearing = proximity_readings[i].HorizontalBearing.GetValue();
      if (-PI / 2 < hbearing && hbearing < PI / 2)
      {
        /*
         * Is this robot closer to us than any other robot we've found so far
         * meeting these criteria?
         */

        if (!robot_found || range < min_range)
        {
          /*
           * This is the closest robot in front of us within our avoidance
           * radius.  Remember its bearing with respect to us.
           */

          min_range = range;
          min_hbearing = hbearing;
          robot_found = true;
        }
      }
    }
  }

  /*
   * If there is a robot meeting the above criteria, turn to avoid the closest
   * such one.
   */

  if (robot_found)
  {
    /*
     * Perform an avoidance maneuver.
     */

    if (min_hbearing < 0.0)
    {
      /*
       * The robot is to our right - turn left.
       */

      m_pcWheels->SetLinearVelocity(0.0, m_fWheelVelocity);
    }
    else
    {
      /*
       * The robot is to our left - turn right.
       */

      m_pcWheels->SetLinearVelocity(m_fWheelVelocity, 0.0);
    }

    ticks_since_last_avoidance = 0;
  }
  else if (ticks_since_last_avoidance > omega_ticks)
  {
    /*
     * We need to move toward the approximate center of the swarm.  We can do so
     * by finding the average horizontal bearing of the position of the other
     * robots from the position of our current robot using the range-and-bearing
     * sensor.  This average bearing points approximately toward the swarm's
     * center.
     */

    double center_bearing = 0.0;
    int n = 0;
    
    for (size_t i = 0; i < proximity_readings.size(); i++)
    {
      /*
       * Ignore those robots whose power has failed - they do not emit light,
       * and so we should not be able to identify them in our calculation of
       * the average.
       */

      if (proximity_readings[i].Data[0] != POWER_FAILURE)
      {
        center_bearing += proximity_readings[i].HorizontalBearing.GetValue();
        n++;
      }
    }

    center_bearing /= n;

    /*
     * If we are not facing the center of the swarm, turn toward it.  Otherwise,
     * move forward.
     */

    if (fabs(center_bearing) > 0.1)
    {
      if (center_bearing < 0.0)
      {
        /*
         * The center is to our right - turn right.
         */

        m_pcWheels->SetLinearVelocity(m_fWheelVelocity, 0.0);
      }
      else
      {
        /*
         * The center is to our left - turn left.
         */

        m_pcWheels->SetLinearVelocity(0.0, m_fWheelVelocity);
      }
    }
    else
    {
      m_pcWheels->SetLinearVelocity(m_fWheelVelocity, m_fWheelVelocity);
    }
  }
  else
  {
    /* 
     * We don't have an obstacle in front of us and we do not yet need to turn
     * toward the center of the swarm - simply move forward.
     */

    m_pcWheels->SetLinearVelocity(m_fWheelVelocity, m_fWheelVelocity);
    ticks_since_last_avoidance++;
  }
}

/****************************************/
/****************************************/

void CEPuckBrownian::PowerFailureStep()
{
  /*
   * This robot has experienced power failure, and so does not move.  Simply
   * remain stationary.
   */

  m_pcWheels->SetLinearVelocity(0.0, 0.0);
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

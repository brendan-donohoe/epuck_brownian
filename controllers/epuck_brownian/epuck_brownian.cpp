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

  std::cout << "Init() called" << std::endl;

  ticks_since_last_avoidance = 0;
  
  m_pcRABA->SetData(0, FUNCTIONING);
}

/****************************************/
/****************************************/

void CEPuckBrownian::ControlStep()
{
  double avoid_radius = avoid_radius_init;
  double PI = 3.1415926535;

  const std::vector<Real>& light_readings = m_pcLight->GetReadings();

  bool light_detected = false;

  for (size_t i = 0; i < light_readings.size(); i++)
  {
    if (light_readings[i] > 0.0)
    {
      light_detected = true;
    }
  }

  if (light_detected)
  {
    avoid_radius = avoid_radius_light;
  }

  const CCI_RangeAndBearingSensor::TReadings& proximity_readings = m_pcRABS->GetReadings();

  bool robot_found = false;
  double min_range;
  double min_hbearing;

  for (size_t i = 0; i < proximity_readings.size(); i++)
  {
    double range = proximity_readings[i].Range;
    if (range < avoid_radius)
    {
      double hbearing = proximity_readings[i].HorizontalBearing.GetValue();
      if (-PI / 2 < hbearing && hbearing < PI / 2)
      {
        if (!robot_found || range < min_range)
        {
          min_range = range;
          min_hbearing = hbearing;
          robot_found = true;
        }
      }
    }
  }

  if (robot_found)
  {
    if (min_hbearing < 0.0)
    {
      m_pcWheels->SetLinearVelocity(0.0, m_fWheelVelocity);
    }
    else
    {
      m_pcWheels->SetLinearVelocity(m_fWheelVelocity, 0.0);
    }

    ticks_since_last_avoidance = 0;
  }
  else if (ticks_since_last_avoidance > omega_ticks)
  {
    double total = 0.0;
    int n = 0;
    
    for (size_t i = 0; i < proximity_readings.size(); i++)
    {
      if (proximity_readings[i].Data[0] != POWER_FAILURE)
      {
        total += proximity_readings[i].HorizontalBearing.GetValue();
        n++;
      }
    }

    total /= n;

    if (fabs(total) > 0.1)
    {
      if (total < 0.0)
      {
        m_pcWheels->SetLinearVelocity(m_fWheelVelocity, 0.0);
      }
      else
      {
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
    m_pcWheels->SetLinearVelocity(m_fWheelVelocity, m_fWheelVelocity);
    ticks_since_last_avoidance++;
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

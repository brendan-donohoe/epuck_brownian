/*
 * AUTHOR: Carlo Pinciroli <cpinciro@ulb.ac.be>
 *
 * An example controller for obstacle avoidance with the e-puck.
 *
 * The controller uses the proximity sensor to detect obstacles and the
 * wheels to move the robot around.
 *
 * This controller is meant to be used with the XML files:
 *    experiments/epuck_brownian.argos
 */

#ifndef EPUCK_BROWNIAN_H
#define EPUCK_BROWNIAN_H

/*
 * Include some necessary headers.
 */
/* Definition of the CCI_Controller class. */
#include <argos3/core/control_interface/ci_controller.h>
/* Definition of the differential steering actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
/* Definition of proximity sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_proximity_sensor.h>
/* Definition of positioning sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>
/* Definition of light sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_light_sensor.h>
/* Definition of range-and-bearing actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_actuator.h>
/* Definition of range-and-bearing sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>

/*
 * All the ARGoS stuff in the 'argos' namespace.
 * With this statement, you save typing argos:: every time.
 */
using namespace argos;

/*
 * A controller is simply an implementation of the CCI_Controller class.
 */
class CEPuckBrownian : public CCI_Controller {

public:

   /* Class constructor. */
   CEPuckBrownian();

   /* Class destructor. */
   virtual ~CEPuckBrownian() {}

   /*
    * This function initializes the controller.
    * The 't_node' variable points to the <parameters> section in the XML
    * file in the <controllers><epuck_brownian_controller> section.
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
   virtual void Destroy();

private:

  /* Pointer to the differential steering actuator */
  CCI_DifferentialSteeringActuator* m_pcWheels;
  /* Pointer to the e-puck proximity sensor */
  CCI_ProximitySensor* m_pcProximity;
  /* Pointer to the positioning sensor */
  CCI_PositioningSensor* m_pcPosition;
  /* Pointer to the light sensor */
  CCI_LightSensor* m_pcLight;
  /* Pointer to the range-and-bearing actuator */
  CCI_RangeAndBearingActuator* m_pcRABA;
  /* Pointer to the range-and-bearing sensor */
  CCI_RangeAndBearingSensor* m_pcRABS;

  /*
   * The following variables are used as parameters for the
   * algorithm. You can set their value in the <parameters> section
   * of the XML configuration file, under the
   * <controllers><epuck_brownian_controller> section.
   */
  /* Wheel speed. */
  Real m_fWheelVelocity;

  enum EMalfunctionType
  {
    FUNCTIONING = 0,
    POWER_FAILURE,
    SENSOR_FAILURE,
    MOTOR_FAILURE
  } m_eMalfunctionType;

  double avoid_radius_init;
  double avoid_radius_light;

  int omega_ticks;

  int ticks_since_last_avoidance;
};

#endif

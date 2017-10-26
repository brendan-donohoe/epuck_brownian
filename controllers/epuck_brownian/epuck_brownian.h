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
#include <iostream>
#include <fstream>
/*
 * All the ARGoS stuff in the 'argos' namespace.
 * With this statement, you save typing argos:: every time.
 */
using namespace argos;
using namespace std;

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

  /* The failure cases for each robot. */
  enum EMalfunctionType
  {
    FUNCTIONING = 0,
    POWER_FAILURE,
    SENSOR_FAILURE,
    MOTOR_FAILURE
  } m_eMalfunctionType;

private:

  /* Step function called when robot is in a functional state. */
  void FunctioningStep();

  /* Step function called when robot has experienced power failure. */
  void PowerFailureStep();

  void SensorFailureStep();

  void MotorFailureStep();

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

  int failure_type;
  int ticks_since_last_avoidance;
  int ticks_since_start;
  int current_type;
  int random_turn;
  int tick_wander;
  bool printed_result;
  int counter;


  /*
   * The following variables are used as parameters for the
   * algorithm. You can set their value in the <parameters> section
   * of the XML configuration file, under the
   * <controllers><epuck_brownian_controller> section.
   */
  /* Wheel speed. */
  Real m_fWheelVelocity;

  /* Avoidance radius if beacon sensor is NOT activated. */
  double avoid_radius_init;
  /* Avoidance radius if beacon sensor is activated. */
  double avoid_radius_light;

  /* This value determines either the left or right motor is not functioning */
  int randomMotorFailure;

  /* Time a robot can go without undergoing an avoidance maneuver before
     returning to the center of the swarm. */
  int omega_ticks;
  /* Time from the start of the simulation until a robot experiences its
     corresponding "failure".  Unused for a functional robot. */
  int ticks_to_failure;

  /* The x-coordinate of the light beacon. */
  double light_x;
  /* The y-coordinate of the light beacon. */
  double light_y;
};

#endif

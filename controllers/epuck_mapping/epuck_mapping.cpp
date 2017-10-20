/* Include the controller definition */
#include "epuck_mapping.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/core/utility/logging/argos_log.h>

#include <cmath>
#include <string>
#include <sstream>

/****************************************/
/****************************************/

CEPuckMapping::CEPuckMapping() :
   m_pcWheels(NULL),
   m_pcProximity(NULL),
   m_fWheelVelocity(2.5f) {}

/****************************************/
/****************************************/

void CEPuckMapping::Init(TConfigurationNode& t_node) {
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
    * file at the <controllers><epuck_mapping><actuators> and
    * <controllers><epuck_mapping><sensors> sections. If you forgot to
    * list a device in the XML and then you request it here, an error
    * occurs.
    */
   m_pcWheels    = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
   m_pcProximity = GetSensor  <CCI_ProximitySensor             >("proximity"    );
   m_pcPosSens = GetSensor  <CCI_PositioningSensor        >("positioning"       );
   /*
    * Parse the configuration file
    *
    * The user defines this part. Here, the algorithm accepts three
    * parameters and it's nice to put them in the config file so we don't
    * have to recompile if we want to try other settings.
    */
   GetNodeAttributeOrDefault(t_node, "velocity", m_fWheelVelocity, m_fWheelVelocity);

   std::cout << "Init() called" << std::endl;
}

/****************************************/
/****************************************/

void CEPuckMapping::ControlStep() {
   /* Get the highest reading in front of the robot, which corresponds to the closest object */
   Real fMaxReadVal = m_pcProximity->GetReadings()[0];
   UInt32 unMaxReadIdx = 0;
   if(fMaxReadVal < m_pcProximity->GetReadings()[1]) {
      fMaxReadVal = m_pcProximity->GetReadings()[1];
      unMaxReadIdx = 1;
   }
   if(fMaxReadVal < m_pcProximity->GetReadings()[7]) {
      fMaxReadVal = m_pcProximity->GetReadings()[7];
      unMaxReadIdx = 7;
   }
   if(fMaxReadVal < m_pcProximity->GetReadings()[6]) {
      fMaxReadVal = m_pcProximity->GetReadings()[6];
      unMaxReadIdx = 6;
   }
   /* Do we have an obstacle in front? */
   if(fMaxReadVal > 0.0f) {
     /* Yes, we do: avoid it */
     if(unMaxReadIdx == 0 || unMaxReadIdx == 1) {
       /* The obstacle is on the left, turn right */
       m_pcWheels->SetLinearVelocity(m_fWheelVelocity, 0.0f);
     }
     else {
       /* The obstacle is on the left, turn right */
       m_pcWheels->SetLinearVelocity(0.0f, m_fWheelVelocity);
     }
   }
   else {
     /* No, we don't: go straight */
      m_pcWheels->SetLinearVelocity(m_fWheelVelocity, m_fWheelVelocity);
   }

   float x = m_pcPosSens->GetReading().Position.GetX();
   float y = m_pcPosSens->GetReading().Position.GetY();
   int grid_x = 20+round(x*10);
   int grid_y = 20+round(y*10);
   
   argos::LOG << "Recording freespace at <" << x << ", " <<  y << "> to grid <" << grid_x  << ", " << grid_y << ">" << std::endl;

   map[grid_x][grid_y]=true;

}

void CEPuckMapping::Destroy() {
  std::cout << "Destroy() called" << std::endl;
  std::stringstream ss;
  ss << GetId();
  std::fstream file_stream("Map"+ss.str()+".txt", std::ios::out | std::ios::trunc);
  //WriteMap(std::cout, map);
  WriteMap(file_stream, map);
}

template<typename T, int height, int width>
std::ostream& CEPuckMapping::WriteMap(std::ostream& os, T (&map)[height][width])
{
    for (int i = 0; i < height; ++i)
    {
        for (int j = 0; j < width; ++j)
        {
            os << map[i][j]<<", ";
        }
        os<<"\n";
    }
    return os;
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
REGISTER_CONTROLLER(CEPuckMapping, "epuck_mapping_controller")

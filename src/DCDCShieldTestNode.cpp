//###############################################################
//# Copyright (C) 2019, Evocortex GmbH, All rights reserved.    #
//# Further regulations can be found in LICENSE file.           #
//###############################################################

/**
 * @file DCDCShieldTestNode.cpp
 * @author MBA (info@evocortex.com)
 *
 * @brief DCDC Shield Test Node
 *
 * @version 1.0
 * @date 2019-10-30
 *
 * @copyright Copyright (c) 2019 Evocortex GmbH
 *
 */

/* Includes ----------------------------------------------------------------------*/
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>

#include <evo_dcdc_shield_interface/DCDCShield.h>
#include <evo_mbed/tools/Logging.h>
#include <evo_mbed/tools/com/ComServer.h>

#include <csignal>

using namespace evo_mbed;
/*--------------------------------------------------------------------------------*/

int main(int argc, char* argv[])
{
   ros::init(argc, argv, "evo_dcdc_shield_interface");
   ros::NodeHandle nh("~");

   std::shared_ptr<ComServer> com_server =
       std::shared_ptr<ComServer>(new ComServer(true));

   if(RES_OK != com_server->init("can_motor", 200))
   {
      return -1;
   }

   std::shared_ptr<evo_mbed::DCDCShield> shield =
       std::shared_ptr<evo_mbed::DCDCShield>(
           new DCDCShield(10, com_server, 5.0, true));

   if(!shield->init())
   {
      return -2;
   }

   if(!shield->setComTimeoutMS(500))
      return -8;

   // Setup channel 2
   if(!shield->setChn2PWMFrequencyHz(50000))
      return -3;
   if(!shield->setChn2Status(true))
      return -4;

   // Publisher
   ros::Publisher pub_bat = nh.advertise<std_msgs::Float32>("battery_voltage", 1);

   const float update_rate = 30.0;
   ros::Rate loop_rate(update_rate);

   float duty_cycle = 0.0f;
   float timer      = 0.0f;
   while(ros::ok())
   {
      duty_cycle = 50.0f + 50.0f * sinf(timer + 4.8f);

      if(!shield->setChn2DutyCycle(duty_cycle))
         return -5;

      std_msgs::Float32 f32;
      f32.data = shield->getBatteryVoltage();

      pub_bat.publish(f32);

      ros::spinOnce();
      loop_rate.sleep();
      timer += 1.0f / update_rate;
   }

   return 0;
}

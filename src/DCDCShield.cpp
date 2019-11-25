//###############################################################
//# Copyright (C) 2019, Evocortex GmbH, All rights reserved.    #
//# Further regulations can be found in LICENSE file.           #
//###############################################################

/**
 * @file MotorShield.cpp
 * @author MBA (info@evocortex.com)
 *
 * @brief Source of Motor Shield
 *
 * @version 1.0
 * @date 2019-04-17
 *
 * @copyright Copyright (c) 2019
 *
 */

/* Includes ----------------------------------------------------------------------*/
#include <evo_dcdc_shield_interface/DCDCShield.h>
#include <evo_mbed/tools/Logging.h>

#include <iomanip>
/*--------------------------------------------------------------------------------*/

using namespace evo_mbed;

/* Public Class Functions --------------------------------------------------------*/

DCDCShield::DCDCShield(const uint8_t node_id, std::shared_ptr<ComServer> com_server,
                       const double update_rate_hz, const bool logging) :
    _com_server(com_server),
    _com_node_id(node_id), _update_rate_hz(update_rate_hz), _logging(logging)
{
   if(_logging)
   {
      _log_module += "[" + std::to_string(node_id) + "]";
   }

   _run_update        = false;
   _dcdc_shield_state = DCDC_SHIELD_STS_OK;
}

DCDCShield::~DCDCShield(void)
{
   release();
}

const bool DCDCShield::init(void)
{
   if(_is_initialized)
   {
      LOG_ERROR("Class is already initialzed");
      return false;
   }

   if(!_com_server)
   {
      LOG_ERROR("Pointer to _com_server is null");
      return false;
   }

   if(_com_node_id < 1 && _com_node_id > 127)
   {
      LOG_ERROR("Node ID is not valid! [1;127]");
      return false;
   }

   if(_update_rate_hz <= 0.1)
   {
      LOG_ERROR("Update rate has to be >= 0.1 (" << _update_rate_hz << ")");
      return false;
   }

   if(RES_OK != _com_server->registerNode(_com_node_id))
   {
      LOG_ERROR("Failed to register node to communciation server!");
      return false;
   }

   if(!readConstObject(_do_device_type))
      return false;

   // Check type
   if(2u != (uint8_t) _do_device_type)
   {
      LOG_ERROR("DCDC Shield error: Type of Node is '" << +(uint8_t) _do_device_type
                                                       << "' which is not a"
                                                       << " DCDC Shield (=2)!");
      return false;
   }

   if(!readConstObject(_do_fw_version))
      return false;
   if(!readConstObject(_do_com_version))
      return false;

   // Check communication version -> Check if com version fits
   // the supported stack
   if(DCDC_SHIELD_COM_VER != (float) (_do_com_version))
   {
      LOG_ERROR("DCDC Shield reports communication version '"
                << (float) (_do_com_version) << "' but only version '"
                << DCDC_SHIELD_COM_VER << "' is supported!");
      return false;
   }

   if(!readConstObject(_do_fw_build_date))
      return false;

   // Read general data
   if(!readConstObject(_do_com_timeout))
      return false;
   if(!readConstObject(_do_set_ch2_pwm_freq))
      return false;
   if(!readConstObject(_do_set_ch2_sts))
      return false;
   if(!readConstObject(_do_set_ch2_pwm_freq))
      return false;

   // Create update thread
   _update_thread = std::make_unique<std::thread>(&DCDCShield::updateHandler, this);
   auto timer_ms  = 0u;
   while(timer_ms < 10u && !_run_update)
   {
      timer_ms++;
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
   }

   if(!_run_update)
   {
      LOG_ERROR("Failed to start update thread!");
      return false;
   }

   LOG_INFO("Initialized DCDC-Shield: "
            << " FW-Ver: " << (float) (_do_fw_version)
            << " FW-Build: " << std::setprecision(8) << (float) (_do_fw_build_date)
            << " COM-Ver: " << std::setprecision(5) << (float) (_do_com_version));

   LOG_INFO("DC/DC-Shield[Timeout]: " << +(uint16_t) _do_com_timeout << " ms");
   LOG_INFO("DC/DC-Shield[PWM-CHN2]: Sts: "
            << +(bool) _do_set_ch2_sts
            << " PWM-Freq (Hz): " << +(uint32_t) _do_set_ch2_pwm_freq
            << " Duty-Cycle (%): " << (float) _do_set_ch2_duty_cycle);

   _dcdc_shield_state      = DCDC_SHIELD_STS_OK;
   _timeout_error_cnt      = 0u;
   _timeout_error_cnt_prev = 0u;

   _is_initialized = true;

   return true;
}

void DCDCShield::release(void)
{
   if(!_is_initialized)
      return;

   if(_run_update)
   {
      _run_update = false;
      _update_thread->join();
   }

   _is_initialized = false;
}

const bool DCDCShield::setComTimeoutMS(const uint16_t timeout_ms)
{
   if(!_is_initialized)
   {
      LOG_ERROR("Class is not initialized!");
      return false;
   }

   _do_com_timeout = timeout_ms;

   return writeDataObject(_do_com_timeout, "Communication Timeout");
}

const bool DCDCShield::setChn1PWMFrequencyHz(const uint32_t frequency_hz)
{
   if(!_is_initialized)
   {
      LOG_ERROR("Class is not initialized!");
      return false;
   }

   _do_set_ch1_pwm_freq = frequency_hz;

   return writeDataObject(_do_set_ch1_pwm_freq, "Channel1 PWM Frequency");
}

const bool DCDCShield::setChn1Status(const bool enable)
{
   if(!_is_initialized)
   {
      LOG_ERROR("Class is not initialized!");
      return false;
   }

   _do_set_ch1_sts = enable;

   return writeDataObject(_do_set_ch1_sts, "Channel1 Status");
}

const bool DCDCShield::setChn1DutyCycle(const float duty_cycle_perc)
{
   if(!_is_initialized)
   {
      LOG_ERROR("Class is not initialized!");
      return false;
   }

   _do_set_ch1_duty_cycle = duty_cycle_perc;

   return writeDataObject(_do_set_ch1_duty_cycle, "Channel1 Duty Cycle");
}

const bool DCDCShield::setChn2PWMFrequencyHz(const uint32_t frequency_hz)
{
   if(!_is_initialized)
   {
      LOG_ERROR("Class is not initialized!");
      return false;
   }

   _do_set_ch2_pwm_freq = frequency_hz;

   return writeDataObject(_do_set_ch2_pwm_freq, "Channel2 PWM Frequency");
}

const bool DCDCShield::setChn2Status(const bool enable)
{
   if(!_is_initialized)
   {
      LOG_ERROR("Class is not initialized!");
      return false;
   }

   _do_set_ch2_sts = enable;

   return writeDataObject(_do_set_ch2_sts, "Channel2 Status");
}

const bool DCDCShield::setChn2DutyCycle(const float duty_cycle_perc)
{
   if(!_is_initialized)
   {
      LOG_ERROR("Class is not initialized!");
      return false;
   }

   _do_set_ch2_duty_cycle = duty_cycle_perc;

   return writeDataObject(_do_set_ch2_duty_cycle, "Channel2 Duty Cycle");
}

const float DCDCShield::getComID(void) const
{
   return _com_node_id;
}

const bool DCDCShield::isInitialized(void) const
{
   return _is_initialized;
}

const float DCDCShield::getBatteryVoltage(void)
{
   return static_cast<float>(_do_battery_voltage);
}

/* !Public Class Functions -------------------------------------------------------*/

/* Private Class Functions -------------------------------------------------------*/

void DCDCShield::updateHandler(void)
{
   _run_update = true;

   const double delay_sec = fabs(1.0 / _update_rate_hz);

   auto time_start = std::chrono::high_resolution_clock::now();

   while(_run_update)
   {

      std::chrono::duration<double> delta_sec =
          std::chrono::high_resolution_clock::now() - time_start;

      if(delta_sec.count() >= delay_sec)
      {

         // TODO Update stuff here
         checkShieldStatus();

         // Read battery voltage
         readConstObject(_do_battery_voltage);

         time_start = std::chrono::high_resolution_clock::now();
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(1));
   }
}

const bool DCDCShield::readConstObject(ComDataObject& object)
{
   std::lock_guard<std::mutex> guard(_com_mutex);

   ComMsgErrorCodes error_code;

   const Result com_result =
       _com_server->readDataObject(_com_node_id, object, error_code, 0u, 4u);

   switch(com_result)
   {
   case RES_OK:
   {
      if(COM_MSG_ERR_NONE != error_code)
         return false;

      return true;
   }
   break;

   case RES_TIMEOUT:
   {
      _timeout_error_cnt++;
      return false;
   }
   break;

   default:
   {
      // General error
      _dcdc_shield_state = DCDC_SHIELD_STS_ERR;
      return false;
   }
   break;
   }

   return false;
}

const bool DCDCShield::writeDataObject(ComDataObject& object, const std::string name)
{
   std::lock_guard<std::mutex> guard(_com_mutex);

   ComMsgErrorCodes error_code = COM_MSG_ERR_NONE;

   const std::string log_info =
       " (Object-ID: " + std::to_string(object.getID()) +
       ", Raw-Value: " + std::to_string(object.getRawValue()) + ", Desc: " + name +
       ")";

   // Write data with timeout threshold = default and 2 retries
   const Result com_result =
       _com_server->writeDataObject(_com_node_id, object, error_code, 0, 2u);

   switch(com_result)
   {
   case RES_OK:
   {

      if(COM_MSG_ERR_NONE == error_code)
      {
         object.setDataReaded();
         return true;
      }

      if(_logging)
      {
         switch(error_code)
         {
         case COM_MSG_ERR_INVLD_CMD:
         {
            LOG_ERROR("Failed to write object: Invalid command" << log_info);
         }
         break;
         case COM_MSG_ERR_READ_ONLY:
         {
            LOG_ERROR("Failed to write object: Read-Only" << log_info);
         }
         break;
         case COM_MSG_ERR_OBJCT_INVLD:
         {
            LOG_ERROR("Failed to write object: Object unknown" << log_info);
         }
         break;
         case COM_MSG_ERR_INVLD_DATA_TYPE:
         {
            LOG_ERROR("Failed to write object: Invalid data type" << log_info);
         }
         break;
         case COM_MSG_ERR_VALUE_RANGE_EXCD:
         {
            LOG_ERROR("Failed to write object: Value out of range" << log_info);
         }
         break;
         case COM_MSG_ERR_COND_NOT_MET:
         {
            LOG_ERROR("Failed to write object: Conditions not met to write"
                      << log_info);
         }
         break;
         }
      }

      if(RES_OK !=
         _com_server->readDataObject(_com_node_id, object, error_code, 0u, 2u))
         LOG_ERROR("Failed to read data from device" << log_info);

      return false;
   }
   break;

   case RES_TIMEOUT:
   {
      _timeout_error_cnt++;

      return false;
   }
   break;

   default:
   {
      // General error
      _dcdc_shield_state = DCDC_SHIELD_STS_ERR;
      return false;
   }
   break;
   }

   return false;
}

void DCDCShield::checkShieldStatus(void)
{
   // Check if error is present -> only healable by reset
   if(DCDC_SHIELD_STS_ERR == _dcdc_shield_state)
   {
      return;
   }

   // Store timeout errors
   const unsigned int timeout_error_cnt = _timeout_error_cnt;
   const int timeout_errors = timeout_error_cnt - _timeout_error_cnt_prev;

   // If more than 2 timeout errors occured in the last cycle
   // switch to timeout error
   if(timeout_errors != 0)
   {
      _dcdc_shield_state = DCDC_SHIELD_TIMEOUT;
      LOG_INFO("Errors: " << +timeout_errors);
   }
   else
   {
      _dcdc_shield_state = DCDC_SHIELD_STS_OK;
   }

   _timeout_error_cnt_prev = timeout_error_cnt;
}

/* !Private Class Functions ------------------------------------------------------*/

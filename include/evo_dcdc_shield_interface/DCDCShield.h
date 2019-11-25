//###############################################################
//# Copyright (C) 2019, Evocortex GmbH, All rights reserved.    #
//# Further regulations can be found in LICENSE file.           #
//###############################################################

/**
 * @file DCDCShield.h
 * @author MBA (info@evocortex.com)
 *
 * @brief DCDC Shield Representation
 *
 * @version 1.0
 * @date 2019-11-19
 *
 * @copyright Copyright (c) 2019 Evocortex GmbH
 *
 */

#ifndef EVO_DCDC_SHIELD_H_
#define EVO_DCDC_SHIELD_H_

/* Includes ----------------------------------------------------------------------*/
#include <evo_mbed/Utils.h>
#include <evo_mbed/tools/com/ComServer.h>
/*--------------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------------*/
/** @addtogroup evocortex
 * @{
 */

namespace evo_mbed {

/*--------------------------------------------------------------------------------*/
/** @addtogroup evocortex_DCDCShield
 * @{
 */

/** \brief Supported communication version */
constexpr float DCDC_SHIELD_COM_VER = 1.0f;

/**
 * @brief States of the motor shield
 */
enum DCDCShieldState : uint8_t
{
   DCDC_SHIELD_STS_ERR  = 0u, //!< General error
   DCDC_SHIELD_STS_OK   = 1u, //!< Ok
   DCDC_SHIELD_SYNC_ERR = 2u, //!< Config on host and shield out of sync
   DCDC_SHIELD_TIMEOUT  = 3u  //!< Timeout (e.g. Shield is off due to emergency stop)
};

/**
 * @brief Communication Object IDs of DCDC Shield
 */
enum DCDCShieldObjects : uint16_t
{
   DCDCSO_DEV_TYPE = 10001u, //!< Device type (DCDC-Shield, Motorshield, ...)
   DCDCSO_FW_VER,            //!< Firmware Version of motor controller
   DCDCSO_FW_COM_VER,        //!< Communication stack version
   DCDCSO_FW_BUILD_DATE,     //!< Build date of the firmware

   /* General settings */
   DCDCSO_COM_TIMEOUT = 10101u, //!< Communication timeout in ms

   /* DC/DC Data */
   DCDCSO_BAT_VOLT = 11001u, //!< Battery voltage in volts

   /* PWM Channels */
   DCDCSO_CH1_PWM_SET_STS = 12001u, //!< Enable disable PWM channel
   DCDCSO_CH1_PWM_FREQ_HZ,          //!< PWM frequency in Hz
   DCDCSO_CH1_PWM_DUTY_CYCLE,       //!< PWM duty cycle in percentage
   DCDCSO_CH2_PWM_SET_STS,          //!< Enable disable PWM channel
   DCDCSO_CH2_PWM_FREQ_HZ,          //!< PWM frequency in Hz
   DCDCSO_CH2_PWM_DUTY_CYCLE,       //!< PWM duty cycle in percentage

   DCDCSO_SIZE = 12u //!< Number of objects
};

/**
 * @brief DCDC Shield Representation
 *
 */
class DCDCShield
{
 public:
   /**
    * @brief Default constructor of a dcdc shield object
    *
    * @param node_id Communication ID of the motor shield [1;127]
    * @param com_server Pointer to communication server
    * @param update_rate Update rate of the async thread in hz
    * @param logging true Enable logging output (default=false)
    */
   DCDCShield(const uint8_t node_id, std::shared_ptr<ComServer> com_server,
              const double update_rate_hz = 5.0, const bool logging = false);

   /**
    * @brief Destructor of dcdc shield object
    */
   ~DCDCShield(void);

   /**
    * @brief Initializes the dcdc shield
    *        Checks if dcdc shield is reachable, if communication
    *        version is supported and starts async update thread.
    *
    * @return true Success
    * @return false Error
    */
   const bool init(void);

   /**
    * @brief Releases the object stops threads and releases
    *        memory
    */
   void release(void);

   /**
    * @brief Maximum communication timeout before DC/DC shields
    *        disables all PWM channels
    *
    * @param timeout_ms Timeout in milliseconds
    *
    * @return true Success
    * @return false Failed
    */
   const bool setComTimeoutMS(const uint16_t timeout_ms);

   /**
    * @brief Set the PWM frequency of the channel 1 in Hz
    *
    * @param frequency_hz [1; 100000] Hz
    *
    * @return true Success
    * @return false Failed
    */
   const bool setChn1PWMFrequencyHz(const uint32_t frequency_hz);

   /**
    * @brief Set the status of channel 1
    *
    * @param enable true: Enables the channel, false: Disables the channel
    *
    * @return true Success
    * @return false Failed
    */
   const bool setChn1Status(const bool enable);

   /**
    * @brief Set the duty cycle of the channel 1 pwm signal
    *
    * @param duty_cycle_perc [0.0;100.0] Duty cycle in percent
    *
    * @return true Success
    * @return false Failed
    */
   const bool setChn1DutyCycle(const float duty_cycle_perc);

   /**
    * @brief Set the PWM frequency of the channel 2 in Hz
    *
    * @param frequency_hz [1; 100000] Hz
    *
    * @return true Success
    * @return false Failed
    */
   const bool setChn2PWMFrequencyHz(const uint32_t frequency_hz);

   /**
    * @brief Set the status of channel 2
    *
    * @param enable true: Enables the channel, false: Disables the channel
    *
    * @return true Success
    * @return false Failed
    */
   const bool setChn2Status(const bool enable);

   /**
    * @brief Set the duty cycle of the channel 2 pwm signal
    *
    * @param duty_cycle_perc [0.0;100.0] Duty cycle in percent
    *
    * @return true Success
    * @return false Failed
    */
   const bool setChn2DutyCycle(const float duty_cycle_perc);

   /* Getters */
   const float getComID(void) const;
   const DCDCShieldState getState(void) const;

   const float getBatteryVoltage(void);

   /** \brief Check if class is initialized */
   const bool isInitialized(void) const;

 private:
   /**
    * @brief Updates the dcdc shield
    */
   void updateHandler(void);

   /**
    * @brief Reads a constant data object
    *
    * @param object Object to read
    *
    * @return true Reading data was successful
    * @return false Failed reading data
    */
   const bool readConstObject(ComDataObject& object);

   /**
    * @brief Writes a data object via can
    *
    * @param object Object to write
    * @param name Name of the object for logging
    *
    * @return true Successfully written value
    * @return false Error during writting
    */
   const bool writeDataObject(ComDataObject& object, const std::string name);

   /**
    * @brief Checks the current shield status
    */
   void checkShieldStatus(void);

   /** \brief Used communication server */
   std::shared_ptr<ComServer> _com_server;

   /** \brief Node ID of the client */
   const unsigned int _com_node_id = 0u;

   /** \brief Update rate of the async data in hz */
   const double _update_rate_hz = 30.0f;

   /** \brief Update thread for asnyc tx/rx */
   std::unique_ptr<std::thread> _update_thread;

   /** \brief Set to false to stop update thread */
   std::atomic<bool> _run_update;

   /** \brief Mutex for synchronization of communication access */
   std::mutex _com_mutex;

   /** \brief Locks checksum calculation during config update */
   std::mutex _config_mutex;

   /** \brief State of the dcdc shield */
   std::atomic<DCDCShieldState> _dcdc_shield_state;

   /** \brief Count of timeout errors occured */
   std::atomic<unsigned int> _timeout_error_cnt;

   /** \brief Old timeout error count */
   unsigned int _timeout_error_cnt_prev;

   /* DCDC shield read only objects */
   ComDataObject _do_device_type =
       ComDataObject(DCDCSO_DEV_TYPE, false, uint8_t(0u));
   ComDataObject _do_fw_version  = ComDataObject(DCDCSO_FW_VER, false, 0.0f);
   ComDataObject _do_com_version = ComDataObject(DCDCSO_FW_COM_VER, false, 0.0f);
   ComDataObject _do_fw_build_date =
       ComDataObject(DCDCSO_FW_BUILD_DATE, false, 0.0f);

   /* General settings */
   ComDataObject _do_com_timeout =
       ComDataObject(DCDCSO_COM_TIMEOUT, true, uint16_t(0u));

   /* DCDC shield read only data */
   ComDataObject _do_battery_voltage = ComDataObject(DCDCSO_BAT_VOLT, false, 0.0f);

   /* DCDC shield settings */
   ComDataObject _do_set_ch1_pwm_freq =
       ComDataObject(DCDCSO_CH1_PWM_FREQ_HZ, true, uint32_t(0));
   ComDataObject _do_set_ch1_sts =
       ComDataObject(DCDCSO_CH1_PWM_SET_STS, true, false);
   ComDataObject _do_set_ch1_duty_cycle =
       ComDataObject(DCDCSO_CH1_PWM_DUTY_CYCLE, true, 0.0f);
   ComDataObject _do_set_ch2_pwm_freq =
       ComDataObject(DCDCSO_CH2_PWM_FREQ_HZ, true, uint32_t(0));
   ComDataObject _do_set_ch2_sts =
       ComDataObject(DCDCSO_CH2_PWM_SET_STS, true, false);
   ComDataObject _do_set_ch2_duty_cycle =
       ComDataObject(DCDCSO_CH2_PWM_DUTY_CYCLE, true, 0.0f);

   /** \brief Logging option: set to true to enable logging */
   const bool _logging = false;

   /** \brief Logging module name */
   std::string _log_module = "DCDCShield";

   /** \brief True class is initialized */
   bool _is_initialized = false;
};

/**
 * @}
 */ // evocortex_DCDCShield
/*--------------------------------------------------------------------------------*/

}; // namespace evo_mbed

/**
 * @}
 */ // evocortex
/*--------------------------------------------------------------------------------*/

#endif /* EVO_DCDC_SHIELD_H_ */

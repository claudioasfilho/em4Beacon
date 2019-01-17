/***************************************************************************//**
 * @file
 * @brief Silicon Labs iBeacon Demo Application
 * This application is intended to be used with the iOS and Android Silicon Labs
 * app for demonstration purposes
 *******************************************************************************
 * # License
 * <b>Copyright 2018 Silicon Laboratories Inc. www.silabs.com</b>
 * Written by Claudio Filho
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/

#include "init_mcu.h"
#include "init_board.h"
#include "init_app.h"
#include "ble-configuration.h"
#include "board_features.h"
#include "sleep.h"

/* BG stack headers */
#include "bg_types.h"
#include "native_gecko.h"
#include "gatt_db.h"
#include "infrastructure.h"
#include "em_cryotimer.h"

/* libraries containing default gecko configuration values */
#include "em_emu.h"
#include "em_cmu.h"
#include "em_rmu.h"

/* Device initialization header */
#include "hal-config.h"

#ifdef FEATURE_BOARD_DETECTED
#if defined(HAL_CONFIG)
#include "bsphalconfig.h"
#else
#include "bspconfig.h"
#endif
#endif

#define EM4WU_PIN          BSP_BUTTON1_PIN
#define EM4WU_PORT         BSP_BUTTON1_PORT
#define EM4WU_EM4WUEN_NUM   (1)                       // PB1 is EM4WUEN pin 1
#define EM4WU_EM4WUEN_MASK  (1 << EM4WU_EM4WUEN_NUM)

#define EM4_RSTCAUSE_MASK	RMU_RSTCAUSE_EM4RST

/***********************************************************************************************//**
 * @addtogroup Application
 * @{
 **************************************************************************************************/

/***********************************************************************************************//**
 * @addtogroup app
 * @{
 **************************************************************************************************/

#ifndef MAX_CONNECTIONS
#define MAX_CONNECTIONS 4
#endif
uint8_t bluetooth_stack_heap[DEFAULT_BLUETOOTH_HEAP(MAX_CONNECTIONS)];

/* Gecko configuration parameters (see gecko_configuration.h) */
static const gecko_configuration_t config = {
  .config_flags = 0,
#if defined(FEATURE_LFXO)
  .sleep.flags = SLEEP_FLAGS_DEEP_SLEEP_ENABLE,
#else
  .sleep.flags = 0,
#endif // LFXO
  .bluetooth.max_connections = MAX_CONNECTIONS,
  .bluetooth.heap = bluetooth_stack_heap,
  .bluetooth.sleep_clock_accuracy = 100, // ppm
  .bluetooth.heap_size = sizeof(bluetooth_stack_heap),
  .gattdb = &bg_gattdb_data,
#if (HAL_PA_ENABLE)
  .pa.config_enable = 1, // Set this to be a valid PA config
#if defined(FEATURE_PA_INPUT_FROM_VBAT)
  .pa.input = GECKO_RADIO_PA_INPUT_VBAT, // Configure PA input to VBAT
#else
  .pa.input = GECKO_RADIO_PA_INPUT_DCDC,
#endif // defined(FEATURE_PA_INPUT_FROM_VBAT)
#endif // (HAL_PA_ENABLE)
};

/**************************************************************************//**
 * @brief GPIO initialization
 *****************************************************************************/
void initGPIO(void)
{
  // Configure GPIO pins
  CMU_ClockEnable(cmuClock_GPIO, true);

  // Configure PB1 as input and EM4 wake-up source
  GPIO_PinModeSet(EM4WU_PORT, EM4WU_PIN, gpioModeInputPullFilter, 1);
  GPIO_EM4EnablePinWakeup(EM4WU_EM4WUEN_MASK << _GPIO_EM4WUEN_EM4WUEN_SHIFT, 0);

  // Configure LED0 and LED1 as output
 // GPIO_PinModeSet(BSP_GPIO_LED0_PORT, BSP_GPIO_LED0_PIN, gpioModePushPull, 0);
//  GPIO_PinModeSet(BSP_GPIO_LED1_PORT, BSP_GPIO_LED1_PIN, gpioModePushPull, 0);
}


/**************************************************************************//**
* @brief Routine for entering into EM4S
*****************************************************************************/
void enterIntoEM4S(void)
{
	EMU_EM4Init_TypeDef em4Init = EMU_EM4INIT_DEFAULT;
   // em4Init.retainUlfrco = true;
	em4Init.pinRetentionMode = emuPinRetentionEm4Exit;
    EMU_EM4Init(&em4Init);
    SLEEP_ForceSleepInEM4();
}


/**************************************************************************//**
* @brief Cryotimer Interrupt handler
*****************************************************************************/
void CRYOTIMER_IRQHandler(void)
{
  /* Read and clear interrupt source */
  CRYOTIMER_IntClear(CRYOTIMER_IF_PERIOD);
}


/**************************************************************************//**
* @brief Setup Cryotimer.
*****************************************************************************/
void cryoSetup(void)
{
  CRYOTIMER_Init_TypeDef cryoInit = CRYOTIMER_INIT_DEFAULT;

  /* Enable CRYO clock */
  CMU_ClockEnable(cmuClock_CRYOTIMER, true);

  cryoInit.enable = false;
  cryoInit.em4Wakeup = true;
  cryoInit.osc = cryotimerOscULFRCO;
  cryoInit.period = cryotimerPeriod_512k; //Change the period to get a different wake-up time. This in milliseconds

  CRYOTIMER_Init(&cryoInit);

  /* Enable required interrupt */
  CRYOTIMER_IntEnable(CRYOTIMER_IF_PERIOD);

  /* Enable CRYOTIMER interrupt */
  CRYOTIMER_IntClear(CRYOTIMER_IF_PERIOD);

  NVIC_ClearPendingIRQ(CRYOTIMER_IRQn);
  NVIC_EnableIRQ(CRYOTIMER_IRQn);
}

#define EDDYSTONE_DATA_LEN           (30)
enum {IBEACON_HANDLER, EDDYSTONE_HANDLER};
static uint8_t eddystone_data[EDDYSTONE_DATA_LEN] = {

  0x03,          //Length of service list
  0x03,          //service list
  0xAA, 0xFE,    //Eddystone ID
  0x10,          //length of service data
  0x16,          //service data
  0xAA,  0xFE,   //Eddystone ID
  0x10,          //frame type Eddyston-URL
  0x00,          // tx power
  0x00,          //http://www.
  's','i','l','a','b','s','.','c','o','m'

};


/**
 * @brief Function for creating a custom advertisement package
 *
 * The function builds the advertisement package according to Apple iBeacon specifications,
 * configures this as the device advertisement data and starts broadcasting.
 */
void bcnSetupAdvBeaconing(void)
{

#if 0
  /* This function sets up a custom advertisement package according to iBeacon specifications.
   * The advertisement package is 30 bytes long. See the iBeacon specification for further details.
   */

  static struct {
    uint8_t flagsLen;     /* Length of the Flags field. */
    uint8_t flagsType;    /* Type of the Flags field. */
    uint8_t flags;        /* Flags field. */
    uint8_t mandataLen;   /* Length of the Manufacturer Data field. */
    uint8_t mandataType;  /* Type of the Manufacturer Data field. */
    uint8_t compId[2];    /* Company ID field. */
    uint8_t beacType[2];  /* Beacon Type field. */
    uint8_t uuid[16];     /* 128-bit Universally Unique Identifier (UUID). The UUID is an identifier for the company using the beacon*/
    uint8_t majNum[2];    /* Beacon major number. Used to group related beacons. */
    uint8_t minNum[2];    /* Beacon minor number. Used to specify individual beacons within a group.*/
    uint8_t txPower;      /* The Beacon's measured RSSI at 1 meter distance in dBm. See the iBeacon specification for measurement guidelines. */
  }
  bcnBeaconAdvData
    = {
    /* Flag bits - See Bluetooth 4.0 Core Specification , Volume 3, Appendix C, 18.1 for more details on flags. */
    2,  /* length  */
    0x01, /* type */
    0x04 | 0x02, /* Flags: LE General Discoverable Mode, BR/EDR is disabled. */

    /* Manufacturer specific data */
    26,  /* length of field*/
    0xFF, /* type of field */

    /* The first two data octets shall contain a company identifier code from
     * the Assigned Numbers - Company Identifiers document */
    /* 0x004C = Apple */
    { UINT16_TO_BYTES(0x004C) },

    /* Beacon type */
    /* 0x0215 is iBeacon */
    { UINT16_TO_BYTE1(0x0215), UINT16_TO_BYTE0(0x0215) },

    /* 128 bit / 16 byte UUID */
    { 0xE2, 0xC5, 0x6D, 0xB5, 0xDF, 0xFB, 0x48, 0xD2, \
      0xB0, 0x60, 0xD0, 0xF5, 0xA7, 0x10, 0x96, 0xE0 },

    /* Beacon major number */
    /* Set to 34987 and converted to correct format */
    { UINT16_TO_BYTE1(34987), UINT16_TO_BYTE0(34987) },

    /* Beacon minor number */
    /* Set as 1025 and converted to correct format */
    { UINT16_TO_BYTE1(1025), UINT16_TO_BYTE0(1025) },

    /* The Beacon's measured RSSI at 1 meter distance in dBm */
    /* 0xC3 is -61dBm */
    0xC3
    };

  //
  uint8_t len = sizeof(bcnBeaconAdvData);
  uint8_t *pData = (uint8_t*)(&bcnBeaconAdvData);

#endif






  /* Set 0 dBm Transmit Power */
  gecko_cmd_system_set_tx_power(0);

  /* Set custom advertising data */
 // gecko_cmd_le_gap_bt5_set_adv_data(0, 0, len, pData);
  gecko_cmd_le_gap_bt5_set_adv_data(EDDYSTONE_HANDLER, 0,30, eddystone_data);

  /* Send only 5 advertisement packets */
  gecko_cmd_le_gap_set_advertise_timing(0, 160, 160, 160,5);

  /* Start advertising in user mode and enable connections */
  gecko_cmd_le_gap_start_advertising(0, le_gap_user_data, le_gap_non_connectable);
}

/**
 * @brief  Main function
 */
int main(void)
{
  // Initialize device
  initMcu();
  // Initialize board
  initBoard();
  // Initialize application
  initApp();


  // Initializations
  initGPIO();

  // Get the last Reset Cause
  uint32_t rstCause = RMU_ResetCauseGet();
  RMU_ResetCauseClear();

  // If the last Reset was due to leaving EM4, toggle LEDs. Else, enter EM4
  if (rstCause == EM4_RSTCAUSE_MASK)
  {
	  // Initialize stack
	   gecko_init(&config);

	   //cryoSetup();

	   while (1) {
	     struct gecko_cmd_packet* evt;

	     // Check for stack event.
	     evt = gecko_wait_event();

	     // Run application and event handler.
	     switch (BGLIB_MSG_ID(evt->header)) {
	       // This boot event is generated when the system boots up after reset.
	       // Do not call any stack commands before receiving the boot event.
	       case gecko_evt_system_boot_id:
	         // Initialize iBeacon ADV data
	         bcnSetupAdvBeaconing();


	         break;

	       case gecko_evt_le_gap_adv_timeout_id:
	         	  /* Start CRYOTIMER and go to EM4S */
	         	  // CRYOTIMER_Enable(true);
	         	  enterIntoEM4S();
	        break;

	       default:
	         break;
	     }
	   }
  }
  else
  {
    for (volatile uint32_t delay = 0; delay < 0xFFF; delay++);
    EMU_EnterEM4();
  }

}

/** @} (end addtogroup app) */
/** @} (end addtogroup Application) */

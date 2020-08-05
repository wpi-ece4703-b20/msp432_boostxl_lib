#ifndef DAC8311_H_
#define DAC8311_H_

// SETTINGS ARE FOR MSP432P401R

#include <stdint.h>

#define SYSTEM_CLK      48000000
#define DAC8311_SPI_CLK 24000000

#define DAC8311_MOSI_PORT   GPIO_PORT_P1
#define DAC8311_MOSI_PIN    GPIO_PIN6
#define DAC8311_MOSI_PIN_FUNCTION            GPIO_PRIMARY_MODULE_FUNCTION

#define DAC8311_SCLK_PORT   GPIO_PORT_P1
#define DAC8311_SCLK_PIN    GPIO_PIN5
#define DAC8311_SCLK_PIN_FUNCTION            GPIO_PRIMARY_MODULE_FUNCTION

#define DAC8311_SYNC_PORT   GPIO_PORT_P5
#define DAC8311_SYNC_PIN    GPIO_PIN2

// Definition of USCI base address to be used for SPI communication
#define DAC8311_EUSCI_BASE          EUSCI_B0_BASE

// Defines to put DAC8311 in various modes
#define DAC8311_NORMAL_MODE         (0x0000)
#define DAC8311_OUTPUT_1K_TO_GND    (0x4000)
#define DAC8311_OUTPUT_100K_TO_GND  (0x8000)
#define DAC8311_OUTPUT_HIGHZ        (0xC000)

//******************************************************************************
//
//! \brief  Initializes the SPI module
//!
//! Current example uses USCI_B0
//!
//! \param  none
//!
//! \return none
//
//******************************************************************************
void DAC8311_init(void);

//******************************************************************************
//
//! \brief  Updates the DAC output and forces the device in Normal Mode
//!
//! This sets the DAC8311 dac output value. DAC8311 is a 14-bit DAC.
//!
//! \param  value   Accepts a 16-bit value and clears bit-15 and bit-16 as this
//!                 forces the DAC8311 into a Normal Mode
//!
//! \return none
//
//******************************************************************************
void DAC8311_updateDacOut(uint16_t value);

//******************************************************************************
//
//! \brief  Sets the DAC into a low power mode
//!
//! DAC8311 can be set into 3 different low power modes. See datasheet for
//! more information. By calling this function, it clears the DAC8311 register
//! values.
//!
//! \param  mode   Sets DAC into 3 differet low power modes
//!                Valid values are:
//!                    - \b DAC8311_OUTPUT_1K_TO_GND
//!                    - \b DAC8311_OUTPUT_100K_TO_GND
//!                    - \b DAC8311_OUTPUT_HIGHZ
//!
//! \return none
//
//******************************************************************************
void DAC8311_setLowPowerMode(uint16_t mode);

#endif /* DAC8311_H_ */

/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/

// AK4558
// I2C sub address
#define AK4558_I2C_POWER (0x00)
#define AK4558_I2C_PLL (0x01)
#define AK4558_I2C_DAC (0x02)
#define AK4558_I2C_CONTROL1 (0x03)
#define AK4558_I2C_CONTROL2 (0x04)
#define AK4558_I2C_MODE (0x05)
#define AK4558_I2C_FILTER (0x06)
#define AK4558_I2C_HPF (0x07)
#define AK4558_I2C_LOUT (0x08)
#define AK4558_I2C_ROUT (0x09)

// PLL 
#define EXT_POWERDOWN (0b0)
#define PLL_POWERUP (0b1)
#define BICK64FS (0b0010)
#define BICK32FS (0b0011)

// POWER 
#define ADC_ON_DAC_ON (0b11111)
#define ADC_ON_DAC_OFF (0b11001)
#define ADC_OFF_DAC_ON (0b00111)
#define ADC_OFF_DAC_OFF (0b00001)

// Audio format
#define STEREO (0b00)
#define I2S_32BIT (0b111)
#define I2S_16BIT (0b011)
#define I2S_24BIT (0b011)
#define ATTENUATIONTIME (0b00)
#define SOFTMUTE_ON (1u)
#define SOFTMUTE_OFF (0u)

// Sample rate
#define FS48_441 (0b0100)
#define FS96_882 (0b0110)
#define FS192_1762 (0b1000)
#define LOPS_NORMALOPERATION (0b010)
#define LOPS_POWERSAVE (0b011u)

// Filter
#define FILTER_SHORTDELAY_SHARPROLLOFF (0x09)

// DAC Volume
#define DACVOLUME_0dB (0xFF)

/* [] END OF FILE */

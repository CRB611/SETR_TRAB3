/**
 * @file tc74.h
 * @brief I²C Interface to the TC47 temperature Sensor
 *
 * This module has the functions to initialize the TC74 sensor,
 * read the temperature through the I²C bus, using the Zephyr DeviceTree
 */

#ifndef TC74_H
#define TC74_H

 #include <zephyr/drivers/i2c.h>

/**
 * @def TC74_CMD_RTR
 * @brief Internal TC47 register with the actual temperature.
 *
 * Must be sent before the readinf to set the reading pointer in 
 * the temperature register.
 */
#define TC74_CMD_RTR  0x00

/**
 * @def TC74_CMD_RWCR
 * @brief TC47 configuration register.
 *
 * Allows reading or writing the sensor's configuration register.
 */
#define TC74_CMD_RWCR 0x01

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialises the TC74 sensor.
 *
 * Checks if the I²C bus is ready to communication and send the command
 * @p TC74_CMD_RTR  to assure that the reading pointer is in the temperature 
 * register.
 *
 * @return 0 if success, negative value (<0) in error.
 */
int tc74_init(void);

/**
 * @brief Reads the current TC74 sensor temperature.
 *
 * Executes the reading of 1 byte from the temperature register and 
 * converts it directly to Celsius Degrees (integer value).
 *
 * @param[out] out_temp Pointer to the variable where the read temperature (°C)
 *                      will be stored.
 * @return 0 if success, negative value (<0) in error.
 */
int tc74_read(uint8_t *out_temp);

#ifdef __cplusplus
}
#endif

#endif /* TC74_H */

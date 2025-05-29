#ifndef _HAL_GPIO_H_
#define _HAL_GPIO_H_

#include <pigpio.h>

#define HAL_GPIO_INT_EDGE_FALLING   FALLING_EDGE
#define HAL_GPIO_INT_EDGE_RISING    RISING_EDGE
#define HAL_GPIO_INT_EDGE_BOTH      EITHER_EDGE
#define HAL_GPIO_INT_EDGE_SETUP     0 // Placeholder for WiringPi compatibility

#define HAL_GPIO_DRDY  3

/**
 * @brief Initializes the GPIO utilities
 *
 * @param none
 *
 * @return Error code
 */
int HAL_GPIO_Init(void);

/**
 * @brief Sets up pin interrupt callback function on certain condition
 *
 * @param [in] pin: GPIO pin number (BCM numbering)
 * @param [in] edge_type: GPIO edge type to trigger interrupt
 * @param [in] cb: Callback function pointer to be called when interrupt occurs
 *
 * @return Error code
 */
int HAL_GPIO_SetupCb(int pin, int edge_type, void (*cb)(int, int, uint32_t));

/**
 * @brief Removes pin interrupt callback
 *
 * @param [in] pin: GPIO pin number (BCM numbering)
 *
 * @return Error code
 */
int HAL_GPIO_SetupCbRemove(int pin);

/**
 * @brief Reads the GPIO pin
 *
 * @param [in] pin: GPIO pin to be read (BCM numbering)
 *
 * @return GPIO value
 */
int HAL_GPIO_PinRead(int pin);

#endif //_HAL_GPIO_H_
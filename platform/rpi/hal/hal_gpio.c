#include <pigpio.h>
#include "hal.h"

/**
 * @brief Initializes the GPIO utilities
 *
 * @param none
 *
 * @return Error code
 */
int HAL_GPIO_Init(void)
{
// Disable pigpio's internal signal handling (bitmask 0 disables all)
   int cfg = gpioCfgGetInternals();
   cfg |= PI_CFG_NOSIGHANDLER;  // (1<<10)
    if (gpioCfgSetInternals(cfg) < 0)
    {
        HAL_Print("hal: *** ERROR *** GPIO: Failed to configure pigpio internals\n");
        return HAL_ERR;
    }

// Set the port to 8881 before initialization
    if (gpioCfgSocketPort(8881) < 0)
    {
        HAL_Print("hal: *** ERROR *** GPIO: Failed to configure socket port 8881\n");
        return HAL_ERR;
    }

    if (gpioInitialise() < 0)
    {
        HAL_Print("hal: *** ERROR *** GPIO: Unable to initialize pigpio...\n");
        return HAL_ERR;
    }
    return HAL_OK;
}

/**
 * @brief Sets up pin interrupt callback function on certain condition
 *
 * @param [in] pin: GPIO pin number (BCM numbering)
 * @param [in] edge_type: GPIO edge type to trigger interrupt
 * @param [in] cb: Callback function pointer to be called when interrupt occurs
 *
 * @return Error code
 */
int HAL_GPIO_SetupCb(int pin, int edge_type, void (*cb)(int, int, uint32_t))
{
    if (HAL_GPIO_Init() != HAL_OK)
        return HAL_ERR;

    if (gpioSetMode(pin, PI_INPUT) < 0)
    {
        HAL_Print("hal: *** ERROR *** GPIO: Unable to set pin %d as input...\n", pin);
        return HAL_ERR;
    }

    // Register callback with correct signature and 0 timeout
    if (gpioSetISRFunc(pin, edge_type, 0, (gpioISRFunc_t)cb) < 0)
    {
        HAL_Print("hal: *** ERROR *** GPIO: Unable to setup ISR for pin %d...\n", pin);
        return HAL_ERR;
    }

    HAL_Print("hal: ISR setup for pin %d with edge %d\n", pin, edge_type);
    return HAL_OK;
}

int HAL_GPIO_SetupCbRemove(int pin)
{
    // Remove ISR by setting NULL callback
    if (gpioSetISRFunc(pin, 0, 0, NULL) < 0)
    {
        HAL_Print("hal: *** ERROR *** GPIO: Unable to remove ISR for pin %d...\n", pin);
        return HAL_ERR;
    }
    HAL_Print("hal: ISR removed for pin %d\n", pin);
    return HAL_OK;
}

/**
 * @brief Reads the GPIO pin
 *
 * @param [in] pin: GPIO pin to be read (BCM numbering)
 *
 * @return GPIO value
 */
int HAL_GPIO_PinRead(int pin)
{
    if (HAL_GPIO_Init() != HAL_OK)
        return -1;

    return gpioRead(pin);
}
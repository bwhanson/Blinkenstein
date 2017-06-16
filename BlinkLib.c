/*! \file 
    \brief 
    
    
*/

#include "BlinkLib.h"
#include "ws2812b_drive.h"
#include "i2s_ws2812b_drive.h"

#include "nrf_rtc.h"

#include "hardware.h"

#define NUM_LEDS 30

static rgb_led_t led_1_array[NUM_LEDS];
static rgb_led_t led_2_array[NUM_LEDS];

void initializeLeds(void)
{
	ws2812b_drive_set_blank(led_1_array, NUM_LEDS);
	ws2812b_drive_set_blank(led_2_array, NUM_LEDS);
	i2s_ws2812b_drive_xfer(led_1_array, NUM_LEDS, LED_DATA_1_PIN);
	i2s_ws2812b_drive_xfer(led_2_array, NUM_LEDS, LED_DATA_2_PIN);
}

void renderLeds(void)
{
	led_1_array[0].red = 255;
	
	i2s_ws2812b_drive_xfer(led_1_array, NUM_LEDS, LED_DATA_1_PIN);
}


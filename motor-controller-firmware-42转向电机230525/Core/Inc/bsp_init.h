#ifndef _BSP_INIT_
#define _BSP_INIT_

#include "common.h"

/* Button bsp init dependency function */
uint8_t read_button_gpio(uint8_t button_id);

void bsp_tmc_init(void);

void bsp_button_init(void);

void bsp_oled_init(void);

#endif 

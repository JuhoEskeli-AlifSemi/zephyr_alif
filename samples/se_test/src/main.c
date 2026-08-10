/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <zephyr/kernel.h>
#include <se_service.h>

int main(void)
{
	printf("Hello World! %s\n", CONFIG_BOARD);
	
	uint8_t buffer[4] = {0};
	int res = se_service_get_rnd_num(&buffer[0], 4);
	if(res) {
		printk("Failed to get random number: %d\n", res);
		return res;
	}
	else {
		printk("Random number: 0x%02x%02x%02x%02x\n", buffer[0], buffer[1], buffer[2], buffer[3]);
	}
	
	return 0;
}

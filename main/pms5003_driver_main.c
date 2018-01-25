#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "pms5003.h"

void app_main()
{
	pms_setup();

	for(;;)
	{
		pms_measurement reading;
		pms_make_measurement(&reading);
		pms_print_measurement(&reading);

		fflush(stdout);
		vTaskDelay(10*1000);
	}
}

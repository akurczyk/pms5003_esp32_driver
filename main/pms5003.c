#include "pms5003.h"

void pms_setup()
{
	// Initializing UART interface
	uart_config_t uart_config = {
		.baud_rate = 9600,
		.data_bits = UART_DATA_8_BITS,
		.parity	= UART_PARITY_DISABLE,
		.stop_bits = UART_STOP_BITS_1,
		.flow_ctrl = UART_HW_FLOWCTRL_DISABLE
	};

	uart_param_config(UART_NUM_1, &uart_config);
	uart_set_pin(UART_NUM_1, PMS_TXD, PMS_RXD, PMS_RTS, PMS_CTS);
	uart_driver_install(UART_NUM_1, PMS_UART_BUF_SIZE * 2, 0, 0, NULL, 0);

	// Initializing GPIO pins
	gpio_config_t io_conf;
	io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
	io_conf.mode = GPIO_MODE_OUTPUT;
	io_conf.pin_bit_mask = (1ULL << PMS_SET) | (1ULL << PMS_RESET) | (1ULL << PMS_MODE);
	io_conf.pull_down_en = 0;
	io_conf.pull_up_en = 0;
	gpio_config(&io_conf);

	gpio_set_level(PMS_SET, 0);
	gpio_set_level(PMS_RESET, 1);
	gpio_set_level(PMS_MODE, 1);
}

void pms_make_measurement(pms_measurement* reading)
{
	uint8_t* data = (uint8_t*) malloc(PMS_UART_BUF_SIZE);

	gpio_set_level(PMS_SET, 1);
	vTaskDelay(1000); // Wait for a second...
	uart_flush(UART_NUM_1);

	pms_measurement current_reading;
	pms_clear_measurement(reading);

	int measurements_number = 0;
	while(measurements_number < 10)
	{
		int len = uart_read_bytes(UART_NUM_1, data, PMS_UART_BUF_SIZE, 1);
		if(len > 0 && pms_process_data(len, data, &current_reading))
		{
			pms_add_measurements(reading, &current_reading);
			measurements_number++;
		}
	}
	pms_div_measurement(reading, 10);

	gpio_set_level(PMS_SET, 0);
}

int pms_process_data(int len, uint8_t* data, pms_measurement* reading)
{
	// Length test
	if(len != 32) return 0;

	// Start of frame delimiter test
	if(data[0] != 0x42 || data[1] != 0x4D || data[2] != 0x00 || data[3] != 0x1C) return 0;

	// Checksum test
	int checksum = 0, checksum_h, checksum_l;
	for(int i=0; i<30; i++) checksum += data[i];
	checksum_h = (checksum >> 8) & 0xFF;
	checksum_l = checksum & 0xFF;
	if(data[30] != checksum_h || data[31] != checksum_l) return 0;

	// Reading data
	reading->pm1_0_std = (data[4]  << 8) + data[5];
	reading->pm2_5_std = (data[6]  << 8) + data[7];
	reading->pm10_std  = (data[8]  << 8) + data[9];
	reading->pm1_0_atm = (data[10] << 8) + data[11];
	reading->pm2_5_atm = (data[12] << 8) + data[13];
	reading->pm10_atm  = (data[14] << 8) + data[15];

	return 1;
}

void pms_print_measurement(pms_measurement* reading)
{
	printf("PM1.0 concentration in standard material:       %d ug/m^3\r\n", reading->pm1_0_std);
	printf("PM2.5 concentration in standard material:       %d ug/m^3\r\n", reading->pm2_5_std);
	printf("PM10  concentration in standard material:       %d ug/m^3\r\n", reading->pm10_std);
	printf("PM1.0 concentration in atmospheric environment: %d ug/m^3\r\n", reading->pm1_0_atm);
	printf("PM2.5 concentration in atmospheric environment: %d ug/m^3\r\n", reading->pm2_5_atm);
	printf("PM10  concentration in atmospheric environment: %d ug/m^3\r\n", reading->pm10_atm);
	printf("\r\n");
}

void pms_clear_measurement(pms_measurement* m)
{
	m->pm1_0_std = 0;
	m->pm2_5_std = 0;
	m->pm10_std  = 0;
	m->pm1_0_atm = 0;
	m->pm2_5_atm = 0;
	m->pm10_atm  = 0;
}

void pms_add_measurements(pms_measurement* m1, pms_measurement* m2)
{
	m1->pm1_0_std += m2->pm1_0_std;
	m1->pm2_5_std += m2->pm2_5_std;
	m1->pm10_std  += m2->pm10_std;
	m1->pm1_0_atm += m2->pm1_0_atm;
	m1->pm2_5_atm += m2->pm2_5_atm;
	m1->pm10_atm  += m2->pm10_atm;
}

void pms_div_measurement(pms_measurement* m, int d)
{
	m->pm1_0_std /= d;
	m->pm2_5_std /= d;
	m->pm10_std  /= d;
	m->pm1_0_atm /= d;
	m->pm2_5_atm /= d;
	m->pm10_atm  /= d;
}

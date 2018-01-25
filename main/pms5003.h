#include "driver/gpio.h"
#include "driver/uart.h"

#define PMS_SET		GPIO_NUM_13
#define PMS_RESET	GPIO_NUM_27
#define PMS_MODE	GPIO_NUM_26
#define PMS_PWM		GPIO_NUM_25
#define PMS_RXD		GPIO_NUM_14
#define PMS_TXD		GPIO_NUM_12
#define PMS_RTS		UART_PIN_NO_CHANGE
#define PMS_CTS		UART_PIN_NO_CHANGE
#define PMS_UART_BUF_SIZE	128

typedef struct {
	int pm1_0_std;
	int pm2_5_std;
	int pm10_std;
	int pm1_0_atm;
	int pm2_5_atm;
	int pm10_atm;
} pms_measurement;

void pms_setup();
void pms_make_measurement(pms_measurement* reading);
int pms_process_data(int len, uint8_t* data, pms_measurement* reading);
void pms_print_measurement(pms_measurement* reading);
void pms_clear_measurement(pms_measurement* m);
void pms_add_measurements(pms_measurement* m1, pms_measurement* m2);
void pms_div_measurement(pms_measurement* m, int d);

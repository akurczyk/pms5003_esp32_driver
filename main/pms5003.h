#include "driver/gpio.h"
#include "driver/uart.h"

typedef struct {
    gpio_num_t set_pin;
    gpio_num_t reset_pin;
    gpio_num_t mode_pin;
    gpio_num_t rxd_pin;
    gpio_num_t txd_pin;
    gpio_num_t rts_pin;
    gpio_num_t cts_pin;
    uart_port_t uart_instance;
    int uart_buffer_size;
} pms5003_config_t;

typedef struct {
    int pm1_0_std;
    int pm2_5_std;
    int pm10_std;
    int pm1_0_atm;
    int pm2_5_atm;
    int pm10_atm;
} pms5003_measurement_t;

void pms5003_setup(pms5003_config_t* inst);
void pms5003_make_measurement(pms5003_config_t* inst, pms5003_measurement_t* reading);
int pms5003_process_data(int len, uint8_t* data, pms5003_measurement_t* reading);
void pms5003_print_measurement(pms5003_measurement_t* reading);
void pms5003_clear_measurement(pms5003_measurement_t* m);
void pms5003_add_measurements(pms5003_measurement_t* m1, pms5003_measurement_t* m2);
void pms5003_div_measurement(pms5003_measurement_t* m, int d);

#ifndef _GPOI
#define _GPOI

void gpio_init(void);
bool is_button_press(void);
void led_data_rx(void);
void led_data_tx(void);
void led_data_off(void);
void set_slave_state_led(bool state);

#endif // _GPOI


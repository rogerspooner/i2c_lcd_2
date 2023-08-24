#include "esp_err.h"
/* Exported functions from keyboard_scan.h */
esp_err_t keyboard_scan(void);
esp_err_t keyboard_init(void);
void  reset_keys_down(int i);

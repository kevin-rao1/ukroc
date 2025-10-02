// For the Pi Pico. 
// Uses the LDR to sense the landing light of the DJI Mavic Air 2 to trigger the servo. 
// Tune BRIGHT_THRESHOLD for your LDR. 

#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/adc.h"

// Pins
#define SERVO_PWM_GPIO    16      // Must be PWM Pin (all GPIOs are PWM)
#define LDR_ADC_GPIO      26      // Must be ADC Pin

// PWM timings for MG90S
#define SERVO_MIN_US     1000     // fully closed
#define SERVO_MAX_US     2000     // fully open
#define SERVO_PERIOD_US  20000    // 50 Hz PWM freq - 20 ms period

// LDR things
#define BRIGHT_THRESHOLD  3400    // 0-4095 from the ADC. Tune.
#define SAMPLE_COUNT         16   // moving average samples
#define BRIGHT_HOLDOFF_MS   150   // require brightness for this long before triggering

static uint slice_num;

// Maps PWM Level to pulse width so it is the pulse width in us.
static inline void servo_write_us(uint16_t pulse_us) {
    if (pulse_us < SERVO_MIN_US) pulse_us = SERVO_MIN_US;
    if (pulse_us > SERVO_MAX_US) pulse_us = SERVO_MAX_US;
    pwm_set_gpio_level(SERVO_PWM_GPIO, pulse_us);
}

static void servo_init_pwm_1us_ticks(void) {
    gpio_set_function(SERVO_PWM_GPIO, GPIO_FUNC_PWM);
    slice_num = pwm_gpio_to_slice_num(SERVO_PWM_GPIO);
    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv(&cfg, 125.0f);           // 1MHz tick
    pwm_config_set_wrap(&cfg, SERVO_PERIOD_US - 1);// sets wrap to count 20000us
    pwm_init(slice_num, &cfg, true);
}

static uint16_t read_ldr_avg(void) {
    uint32_t acc = 0;
    for (int i = 0; i < SAMPLE_COUNT; ++i) {
        acc += adc_read();   // 12-bit
    }
    return (uint16_t)(acc / SAMPLE_COUNT);
}

int main() {
    // Init stuff
    stdio_init_all();
    servo_init_pwm_1us_ticks();
    adc_init();
    adc_gpio_init(LDR_ADC_GPIO);
    adc_select_input(0);
    servo_write_us(SERVO_MIN_US);
    bool opened = false;
    absolute_time_t bright_since = nil_time;

    // loop
    while (true) {
        uint16_t adc_raw = read_ldr_avg();

        if (!opened) {
            // Detect landing light
            if (adc_raw >= BRIGHT_THRESHOLD) {
                if (is_nil_time(bright_since)) {
                    bright_since = get_absolute_time();
                }
                // open servo
                if (absolute_time_diff_us(bright_since, get_absolute_time()) >= BRIGHT_HOLDOFF_MS * 1000) {
                    servo_write_us(SERVO_MAX_US);
                    opened = true;
                }
            } else {
                bright_since = nil_time; // reset timer if returns to below threshold
            }
        }

        sleep_ms(10);
    }
}

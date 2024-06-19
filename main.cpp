#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "hardware/dma.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/i2c.h"
#include "pico/i2c_slave.h"

#include "pulse_analyzer.pio.h"

// PIN SETUP FINAL PCB
#define PWM 0
#define REF_PWM 2
#define DIG_MEAS 3
#define SDA 4
#define SCL 5
#define LED_PIN 25

uint BASE_CLK_FREQ = 125000000;
float LATEST_DUTY;
bool SHORTED;

void setup_pwm() {
    // Tell GPIO 0 and 1 they are allocated to the PWM
    gpio_set_function(PWM, GPIO_FUNC_PWM);
    gpio_set_function(REF_PWM, GPIO_FUNC_PWM);

    // Find out which PWM slice is connected to GPIO 0 (it's slice 0)
    uint slice_num = pwm_gpio_to_slice_num(PWM);
    uint ref_slice_num = pwm_gpio_to_slice_num(REF_PWM);

    uint PWM_FREQ = 2000;
    uint PWM_DUTY = 2;

    uint REF_PWM_FREQ = 100000;
    uint REF_PWM_DUTY = 10;
    
    uint wrap_value = BASE_CLK_FREQ / PWM_FREQ; // Calculate wrap value
    uint ref_wrap_value = BASE_CLK_FREQ / REF_PWM_FREQ; // Calculate wrap value

    pwm_set_wrap(slice_num, wrap_value - 1); // Set wrap value (subtract 1 because it's 0-indexed)
    pwm_set_wrap(ref_slice_num, ref_wrap_value - 1); // Set wrap value (subtract 1 because it's 0-indexed)

    // Set channel 0A and 1A output high for a proportion of the period
    pwm_set_chan_level(slice_num, 0, wrap_value * PWM_DUTY / 100);
    pwm_set_chan_level(ref_slice_num, 0, ref_wrap_value * REF_PWM_DUTY / 100);

    // Set the PWM running
    pwm_set_enabled(slice_num, true);
    pwm_set_enabled(ref_slice_num, true);
    /// \end::setup_pwm[]
}

void change_pwm(uint duty, uint freq) {
    uint slice_num = pwm_gpio_to_slice_num(0);
    uint wrap_value = BASE_CLK_FREQ / freq;
    pwm_set_chan_level(slice_num, PWM, wrap_value * duty / 100);
}

void change_ref_pwm(uint duty, uint freq) {
    uint slice_num = pwm_gpio_to_slice_num(2);
    uint wrap_value = BASE_CLK_FREQ / freq;
    pwm_set_chan_level(slice_num, REF_PWM, wrap_value * duty / 100);
}

class pulse_analyzer
{
public:
    // constructor
    // input = pin that receives the PWM pulses.
    pulse_analyzer()
    {
        // pio 0 is used
        pio = pio0;
        // state machine 0
        sm = 0;
        // configure the used pins
        pio_gpio_init(pio, DIG_MEAS);
        // load the pio program into the pio memory
        uint offset = pio_add_program(pio, &pulse_analyzer_program);
        // make a sm config
        pio_sm_config c = pulse_analyzer_program_get_default_config(offset);
        // set the 'jmp' pin
        sm_config_set_jmp_pin(&c, DIG_MEAS);
        // set the 'wait' pin (uses 'in' pins)
        sm_config_set_in_pins(&c, DIG_MEAS);
        // set shift direction
        sm_config_set_in_shift(&c, false, false, 0);
        // init the pio sm with the config
        pio_sm_init(pio, sm, offset, &c);
        // enable the sm
        pio_sm_set_enabled(pio, sm, true);
    }

    // read_dutycycle (between 0 and 1)
    float read_dutycycle(void)
    {
        read();
        printf("Pulse width: %d\n", pulsewidth);
        printf("Period: %d\n", period);
        printf("Duty cycle: %f\n\n", ((float)pulsewidth / (float)period));
        return ((float)pulsewidth / (float)period);
    }

private:
    // read the period and pulsewidth
    void read(void)
    {
        // clear the FIFO: do a new measurement
        pio_sm_clear_fifos(pio, sm);
        // wait for the FIFO to contain two data items: pulsewidth and period
        while (pio_sm_get_rx_fifo_level(pio, sm) < 2)
            ;
        // read pulse width from the FIFO
        pulsewidth = pio_sm_get(pio, sm);
        // read period from the FIFO
        period = pio_sm_get(pio, sm) + pulsewidth;
        // the measurements are taken with 2 clock cycles per timer tick
        pulsewidth = 2 * pulsewidth;
        // calculate the period in clock cycles:
        period = 2 * period;
    }
    // the pio instance
    PIO pio;
    // the state machine
    uint sm;
    // data about the PWM input measured in pio clock cycles
    uint32_t pulsewidth, period;
};

static const uint I2C_SLAVE_ADDRESS = 0x17;
static const uint I2C_BAUDRATE = 400000; // 400 kHz

static const uint I2C_SLAVE_SDA_PIN = PICO_DEFAULT_I2C_SDA_PIN; // 4
static const uint I2C_SLAVE_SCL_PIN = PICO_DEFAULT_I2C_SCL_PIN; // 5

static void i2c_slave_handler(i2c_inst_t *i2c, i2c_slave_event_t event) {
    uint8_t data_send = (uint8_t) (SHORTED);
    switch (event) {
    case I2C_SLAVE_RECEIVE: // master has written some data
        printf("Received: %i\n", i2c_read_byte_raw(i2c));
        // Nothing is yet being done with the received data

    case I2C_SLAVE_REQUEST: // master is requesting data
        printf("Requested\n Sending %d\n", data_send);
        i2c_write_byte_raw(i2c, data_send);
        break;
    case I2C_SLAVE_FINISH: // master has signalled Stop / Restart
        break;
    default:
        break;
    }
}

static void setup_slave() {
    gpio_init(I2C_SLAVE_SDA_PIN);
    gpio_set_function(I2C_SLAVE_SDA_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SLAVE_SDA_PIN);

    gpio_init(I2C_SLAVE_SCL_PIN);
    gpio_set_function(I2C_SLAVE_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SLAVE_SCL_PIN);

    i2c_init(i2c0, I2C_BAUDRATE);
    // configure I2C0 for slave mode
    i2c_slave_init(i2c0, I2C_SLAVE_ADDRESS, &i2c_slave_handler);
}

int main(){

    stdio_init_all(); 
    setup_pwm();
    setup_slave();

    pulse_analyzer pulse_analyzer_instance; // Instantiate an object of the pulse_analyzer class

    // initialise GPIO (Green LED connected to pin 25)
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 0);

    while (1)
    {        
        // Read the duty cycle of the PWM signal
        LATEST_DUTY = pulse_analyzer_instance.read_dutycycle();
        if (LATEST_DUTY > 0.005) {
            SHORTED = true;
            gpio_put(LED_PIN, 1);
        } else {
            SHORTED = false;
            gpio_put(LED_PIN, 0);
        }
        
    }
  
}
#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/dma.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/i2c.h"
#include "pico/i2c_slave.h"

#include "pulse_analyzer.pio.h"

// set this to determine samtple rate
// 96     = 500,000 Hz
// 960   = 50,000 Hz
// 9600  = 5,000 Hz
#define CLOCK_DIV 96
#define FSAMP 500000

// Channel 0 is GPIO26
#define CAPTURE_CHANNEL 0
#define LED_PIN 25

#define SAMPLES 1000

#define PWM_DUTY_CYCLE 50

// globals
dma_channel_config cfg;
uint dma_chan;
float freqs[SAMPLES];

void setup_pwm() {
    /// \tag::setup_pwm[]

    // Tell GPIO 0 and 1 they are allocated to the PWM
    gpio_set_function(0, GPIO_FUNC_PWM);
    gpio_set_function(1, GPIO_FUNC_PWM);

    // Find out which PWM slice is connected to GPIO 0 (it's slice 0)
    uint slice_num = pwm_gpio_to_slice_num(0);

    // Set up the PWM to output at 1000Hz
    uint base_clk_freq = 125000000; // Base clock frequency is 125MHz
    uint target_freq = 50000;
    uint wrap_value = base_clk_freq / target_freq; // Calculate wrap value

    pwm_set_wrap(slice_num, wrap_value - 1); // Set wrap value (subtract 1 because it's 0-indexed)

    // Set channel A output high for a proportion of the period
    pwm_set_chan_level(slice_num, PWM_CHAN_A, wrap_value / 20); // 25% duty cycle
    // Set channel B output high for a proportion of the period
    pwm_set_chan_level(slice_num, PWM_CHAN_B, wrap_value * 3 / 4); // 75% duty cycle

    // Set the PWM running
    pwm_set_enabled(slice_num, true);
    /// \end::setup_pwm[]
}

void setup_adc() {
    // Set up the ADC to sample on pin 26 (GPIO 26)
    adc_gpio_init(26 + CAPTURE_CHANNEL);

    adc_init();

    adc_select_input(CAPTURE_CHANNEL);
    adc_fifo_setup(
        true,    // Write each completed conversion to the sample FIFO
        true,    // Enable DMA data request (DREQ)
        1,       // DREQ (and IRQ) asserted when at least 1 sample present
        false,   // We won't see the ERR bit because of 8 bit reads; disable.
        true     // Shift each sample to 8 bits when pushing to FIFO
    );

    // set sample rate
    adc_set_clkdiv(CLOCK_DIV);

    sleep_ms(1000);
    // Set up the DMA to start transferring data as soon as it appears in FIFO
    uint dma_chan = dma_claim_unused_channel(true);
    cfg = dma_channel_get_default_config(dma_chan);

    // Reading from constant address, writing to incrementing byte addresses
    channel_config_set_transfer_data_size(&cfg, DMA_SIZE_8);
    channel_config_set_read_increment(&cfg, false);
    channel_config_set_write_increment(&cfg, true);

    // Pace transfers based on availability of ADC samples
    channel_config_set_dreq(&cfg, DREQ_ADC);
}

class pulse_analyzer
{
public:
    // constructor
    // input = pin that receives the PWM pulses.
    pulse_analyzer(uint input)
    {
        // pio 0 is used
        pio = pio0;
        // state machine 0
        sm = 0;
        // configure the used pins
        pio_gpio_init(pio, input);
        // load the pio program into the pio memory
        uint offset = pio_add_program(pio, &pulse_analyzer_program);
        // make a sm config
        pio_sm_config c = pulse_analyzer_program_get_default_config(offset);
        // set the 'jmp' pin
        sm_config_set_jmp_pin(&c, input);
        // set the 'wait' pin (uses 'in' pins)
        sm_config_set_in_pins(&c, input);
        // set shift direction
        sm_config_set_in_shift(&c, false, false, 0);
        // init the pio sm with the config
        pio_sm_init(pio, sm, offset, &c);
        // enable the sm
        pio_sm_set_enabled(pio, sm, true);
    }

    // read_period (in seconds)
    float read_period(void)
    {
        read();
        // one clock cycle is 1/125000000 seconds
        return (period * 0.000000008);
    }

    // read_pulsewidth (in seconds)
    float read_pulsewidth(void)
    {
        read();
        // one clock cycle is 1/125000000 seconds
        return (pulsewidth * 0.000000008);
    }

    // read_dutycycle (between 0 and 1)
    float read_dutycycle(void)
    {
        read();
        return ((float)pulsewidth / (float)period);
    }

    // read pulsewidth and period for one pulse
    void read_PWM(float *readings)
    {
        read();
        *(readings + 0) = (float)pulsewidth * 0.000000008;
        *(readings + 1) = (float)period * 0.000000008;
        *(readings + 2) = ((float)pulsewidth / (float)period);
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

void sample(uint8_t *capture_buf) {
  adc_fifo_drain();
  adc_run(false);
      
  dma_channel_configure(dma_chan, &cfg,
			capture_buf,    // dst
			&adc_hw->fifo,  // src
			SAMPLES,          // transfer count
			true            // start immediately
			);

  adc_run(true);
  dma_channel_wait_for_finish_blocking(dma_chan);
}

static const uint I2C_SLAVE_ADDRESS = 0x17;
static const uint I2C_BAUDRATE = 100000; // 100 kHz

static const uint I2C_SLAVE_SDA_PIN = PICO_DEFAULT_I2C_SDA_PIN; // 4
static const uint I2C_SLAVE_SCL_PIN = PICO_DEFAULT_I2C_SCL_PIN; // 5

static void i2c_slave_handler(i2c_inst_t *i2c, i2c_slave_event_t event) {
    uint8_t data_send = (uint8_t) ((adc_read()) / 16);
    switch (event) {
    case I2C_SLAVE_RECEIVE: // master has written some data
        printf("Received: %d\n", i2c_read_byte_raw(i2c));
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
    // setup_adc();

    setup_slave();

    pulse_analyzer pulse_analyzer(26);

    // the array with the results
    float pwm_reading[3];

    // initialise GPIO (Green LED connected to pin 25)
    gpio_init(25);
    gpio_set_dir(25, GPIO_OUT);
    gpio_put(25, 1);

    gpio_init(22);
    gpio_set_dir(22, GPIO_OUT);
    gpio_put(22, 1);

    // potvalue at gpio 26 adc0
    adc_init();
    adc_gpio_init(26);
    adc_select_input(0);


    while (1)
    {        
        sleep_ms(10);
    }
  
}
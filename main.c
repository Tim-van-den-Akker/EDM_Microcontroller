#include <stdio.h>

#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/dma.h"
#include "hardware/pwm.h"

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
    pwm_set_chan_level(slice_num, PWM_CHAN_A, wrap_value / 4); // 25% duty cycle
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

int main(){

    stdio_init_all(); 
    setup_pwm();
    setup_adc();
    

    // initialise GPIO (Green LED connected to pin 25)
    gpio_init(25);
    gpio_set_dir(25, GPIO_OUT);

    gpio_put(25, 1);

    // initialise the buffer
    uint8_t cap_buf[SAMPLES]; 

    // do the loop for 10 time
    for (int j = 0; j < 1000; j++){
        
        // sleep for 1 second
        // sleep_ms(1000);
        // get NSAMP samples at FSAMP
        sample(cap_buf);
        // sleep_ms(1000);
        // printf("[");
        // for (int i = 0; i < SAMPLES; i = i + 1){
        //     printf("%i, ", cap_buf[i]);
        // }
        // printf("]\n");
    }
    while (1)
    {
        gpio_put(25, 0);
    }
    
}
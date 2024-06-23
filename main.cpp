#include <stdio.h>
#include <string.h>
#include <time.h>

#include "pico/stdlib.h"
#include "hardware/dma.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/i2c.h"
#include "pico/multicore.h"
#include "pico/i2c_slave.h"

// WIFI
#include "pico/cyw43_arch.h"
#include "lwip/pbuf.h"
#include "lwip/tcp.h"


#include "pulse_analyzer.pio.h"

// PIN SETUP FINAL PCB
#define PWM 0
#define REF_PWM 2
#define DIG_MEAS 3
#define SDA 4
#define SCL 5
#define LED_PIN 25

// WIFI SETUP
#define WIFI_SSID "PiSetup"
#define WIFI_PASSWORD "raspberry123"

#define TCP_PORT 4242
#define DEBUG_printf printf
#define BUF_SIZE 1

uint BASE_CLK_FREQ = 125000000;
uint GLOBAL_FREQ = 2000;

float LATEST_DUTY;
bool SHORTED = false;

uint8_t DATA_SEND;

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
    uint REF_PWM_DUTY = 40;
    
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

void change_pwm(int freq, float duty) {
    uint wrap_value = BASE_CLK_FREQ / freq;
    uint slice_num = pwm_gpio_to_slice_num(0);
    // Disable the PWM before configuring
    pwm_set_enabled(slice_num, false);
    pwm_set_wrap(slice_num, wrap_value - 1);
    pwm_set_chan_level(slice_num, PWM, wrap_value * duty / 100);
    pwm_set_enabled(slice_num, true);
}

void change_ref_pwm(float voltage) {
    // convert voltage to duty cycle where 0V is 0% and 3.3V is 100%
    uint REF_PWM_FREQ = 100000;
    uint REF_PWM_DUTY = voltage / 3.3;

    uint wrap_value = BASE_CLK_FREQ / REF_PWM_FREQ;
    uint slice_num = pwm_gpio_to_slice_num(0);
    // Disable the PWM before configuring
    pwm_set_enabled(slice_num, false);
    pwm_set_wrap(slice_num, wrap_value - 1);
    pwm_set_chan_level(slice_num, PWM, wrap_value * REF_PWM_DUTY);
    pwm_set_enabled(slice_num, true);
}

class pulse_analyzer
{
public:
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
        // printf("Pulse width: %d\n", pulsewidth);
        // printf("Period: %d\n", period);
        // printf("Duty cycle: %f\n\n", ((float)pulsewidth / (float)period));
        return ((float)pulsewidth / (float)period);
    }
 
private:
    // read the period and pulsewidth
    void read(void)
    {
        // clear the FIFO: do a new measurement
        pio_sm_clear_fifos(pio, sm);
        // wait for the FIFO to contain two data items: pulsewidth and period
            while (pio_sm_get_rx_fifo_level(pio, sm) < 2){
            }
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

void wifi_init() {
    if (cyw43_arch_init()) {
        printf("failed to initialize\n");
        return;
    }

    cyw43_arch_enable_sta_mode();

    printf("Connecting to Wi-Fi...\n");
    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 10000)) {
        printf("failed to connect\n");
        cyw43_arch_deinit();
        return;
    }

    printf("Connected to Wi-Fi\n");
}

typedef struct TCP_SERVER_T_ {
    struct tcp_pcb *server_pcb;
    struct tcp_pcb *client_pcb;
    uint8_t buffer_sent[BUF_SIZE];
    uint8_t buffer_recv[BUF_SIZE];
    int sent_len;
    int recv_len;
} TCP_SERVER_T;

static TCP_SERVER_T* tcp_server_init(void) {
    TCP_SERVER_T *state = static_cast<TCP_SERVER_T*>(calloc(1, sizeof(TCP_SERVER_T)));
    // printf("tcp_server_init");
    return state;
}

static err_t tcp_server_close(void *arg) {
    // printf("tcp_server_close");
    TCP_SERVER_T *state = (TCP_SERVER_T*)arg;
    if (state->client_pcb) {
        tcp_close(state->client_pcb);
        state->client_pcb = NULL;
    }
    if (state->server_pcb) {
        tcp_close(state->server_pcb);
        state->server_pcb = NULL;
    }
    return ERR_OK;
}

static err_t tcp_server_send_data(void *arg, struct tcp_pcb *tpcb) {
    // printf("tcp_server_send_data");
    TCP_SERVER_T *state = (TCP_SERVER_T*)arg;
    memset(state->buffer_sent, DATA_SEND, BUF_SIZE);
    printf("Sending %d\n", state->buffer_sent[0]);
    state->sent_len = 0;
    return tcp_write(tpcb, state->buffer_sent, BUF_SIZE, TCP_WRITE_FLAG_COPY);
}

static err_t tcp_server_sent(void *arg, struct tcp_pcb *tpcb, u16_t len) {
    // printf("tcp_server_sent");
    TCP_SERVER_T *state = (TCP_SERVER_T*)arg;
    state->sent_len += len;
    return tcp_server_send_data(arg, tpcb);
}

static err_t tcp_server_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err) {
    // printf("tcp_server_recv");
    TCP_SERVER_T *state = (TCP_SERVER_T*)arg;

    int freq = 0;
    float duty = 0.0;

    if (err == ERR_OK && p != NULL) {
        // create string
        char *data = (char*)calloc(p->len + 1, sizeof(char));
        memcpy(data, p->payload, p->len);
        
        const char *freq_pos = strstr(data, "freq=");
        if (freq_pos) {
            // Move past "freq="
            freq_pos += 5;
            freq = atoi(freq_pos);
        } 

        // Find "duty="
        const char *duty_pos = strstr(data, "duty=");
        if (duty_pos) {
            // Move past "duty="
            duty_pos += 5;
            duty = atof(duty_pos);
        }

        change_pwm(freq, duty);
        printf("Received: %s\n", data);
        printf("Frequency: %d\n", freq);
        printf("Duty cycle: %f\n", duty);


        state->recv_len = p->len;
        // pbuf_free(p);
        return ERR_OK;
    }
    return ERR_OK;
}

static err_t tcp_server_accept(void *arg, struct tcp_pcb *client_pcb, err_t err) {
    // printf("tcp_server_accept");
    TCP_SERVER_T *state = (TCP_SERVER_T*)arg;
    state->client_pcb = client_pcb;
    tcp_arg(client_pcb, state);
    tcp_sent(client_pcb, tcp_server_sent);
    tcp_recv(client_pcb, tcp_server_recv);
    return tcp_server_send_data(arg, client_pcb);
}

static bool tcp_server_open(void *arg) {
    // printf("tcp_server_open");
    TCP_SERVER_T *state = (TCP_SERVER_T*)arg;
    struct tcp_pcb *pcb = tcp_new_ip_type(IPADDR_TYPE_ANY);
    if (!pcb || tcp_bind(pcb, IP_ANY_TYPE, TCP_PORT) != ERR_OK) {
        return false;
    }
    state->server_pcb = tcp_listen(pcb);
    tcp_arg(state->server_pcb, state);
    tcp_accept(state->server_pcb, tcp_server_accept);
    return true;
}

// Core 1 Main Code
void core1_entry() {
    pulse_analyzer pulse_analyzer_instance; // Instantiate an object of the pulse_analyzer class

    // get current time
    absolute_time_t time_since_last_spark = get_absolute_time();
    while (1){
        
        LATEST_DUTY = pulse_analyzer_instance.read_dutycycle();
        if (LATEST_DUTY < 0.0001) {
            // turn led on
            SHORTED = true;
            time_since_last_spark = get_absolute_time();
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
        } else if (absolute_time_diff_us(time_since_last_spark, get_absolute_time()) < 100000){
            SHORTED = true;
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
        }
        else {
            SHORTED = false;
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
        }
        DATA_SEND = (uint8_t) SHORTED;
    }
}

int main(){

    stdio_init_all(); 
    setup_pwm();
    setup_slave();
    sleep_ms(10);
    wifi_init();
    sleep_ms(10);

    // initialise GPIO (Green LED connected to pin 25)
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 0);

    multicore_launch_core1(core1_entry);

    TCP_SERVER_T *state = tcp_server_init();
    if (!state) {
        return 1;
    }
    if (!tcp_server_open(state)) {
        return 1;
    }

    while (1)
    {        
        tight_loop_contents();
    }
    free(state);
  
}
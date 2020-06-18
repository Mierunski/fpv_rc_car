/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 * Copyright (c) 2020 Mieszko Mierunski
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */
#include <drivers/clock_control.h>
#include <drivers/clock_control/nrf_clock_control.h>
#include <drivers/gpio.h>
#include <drivers/pwm.h>
#include <irq.h>
#include <logging/log.h>
#include <nrf.h>
#include <esb.h>
#include <zephyr.h>
#include <zephyr/types.h>

LOG_MODULE_REGISTER(esb_prx);

#define LED_ON 0
#define LED_OFF 1

#define DT_DRV_COMPAT nordic_nrf_clock

static struct device *led_port;
static struct esb_payload rx_payload;
static struct esb_payload tx_payload = ESB_CREATE_PAYLOAD(0,
	0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17);


#include <drivers/adc.h>
#include <hal/nrf_saadc.h>
#include <hal/nrf_gpio.h>
#define ADC_DEVICE_NAME		DT_ADC_0_NAME
#define ADC_RESOLUTION		14
#define ADC_GAIN		ADC_GAIN_1_6
#define ADC_REFERENCE		ADC_REF_INTERNAL
#define ADC_ACQUISITION_TIME	ADC_ACQ_TIME(ADC_ACQ_TIME_MICROSECONDS, 40)
#define ADC_1ST_CHANNEL_ID	0
#define ADC_1ST_CHANNEL_INPUT	NRF_SAADC_INPUT_AIN1
#define ADC_2ND_CHANNEL_ID	2
#define ADC_2ND_CHANNEL_INPUT	NRF_SAADC_INPUT_AIN2

#define BUFFER_SIZE  6
static s16_t m_sample_buffer[BUFFER_SIZE];

static const struct adc_channel_cfg m_1st_channel_cfg = {
	.gain             = ADC_GAIN,
	.reference        = ADC_REFERENCE,
	.acquisition_time = ADC_ACQUISITION_TIME,
	.channel_id       = ADC_1ST_CHANNEL_ID,
#if defined(CONFIG_ADC_CONFIGURABLE_INPUTS)
	.input_positive   = ADC_1ST_CHANNEL_INPUT,
#endif
};
static struct device *init_adc(void)
{
	int ret;
	struct device *adc_dev = device_get_binding("ADC_0");


	ret = adc_channel_setup(adc_dev, &m_1st_channel_cfg);


	(void)memset(m_sample_buffer, 0, sizeof(m_sample_buffer));

	return adc_dev;
}

static int read_battery_voltage(void)
{
	int ret;
	const struct adc_sequence sequence = {
		.channels    = BIT(ADC_1ST_CHANNEL_ID),
		.buffer      = m_sample_buffer,
		.buffer_size = sizeof(m_sample_buffer),
		.resolution  = ADC_RESOLUTION,
	};

	struct device *adc_dev = init_adc();

	if (!adc_dev) {
		return 1;
	}

	ret = adc_read(adc_dev, &sequence);
	static s16_t samples[10];
	static u8_t index = 0;
	samples[index] = m_sample_buffer[0];
	index = (index + 1) % 10;
	s32_t acc = 0;
	for (int i = 0; i < 10; i++) {
		acc += samples[i];
	}
	float read = acc / 10;
	read /= 1427.f;

	LOG_INF("ADC: %d.%03d\r\n", (int)read, ((int)(read*1000)) % 1000);
	return 0;
}

#define PIN_IN1 23
#define PIN_IN2 25
#define PIN_EN 24
#define PIN_SERVO 22
#define PERIOD (USEC_PER_SEC / 50U)
#define PWM_DRIVER "PWM_0"

struct device *pwm_dev;

void set(uint8_t angle, uint8_t motor) {
	if (motor > 100) {
		motor -= 100;
		nrf_gpio_pin_set(PIN_IN1);
		nrf_gpio_pin_clear(PIN_IN2);
	} else {
		nrf_gpio_pin_set(PIN_IN2);
		nrf_gpio_pin_clear(PIN_IN1);
	}
	printk("%d\r\n", angle);
	pwm_pin_set_usec(pwm_dev, PIN_EN, PERIOD, motor*(PERIOD/100), 0);
	pwm_pin_set_usec(pwm_dev, PIN_SERVO, PERIOD, angle*10 + 1000, 0);
}

static int leds_init(void)
{
	led_port = device_get_binding(DT_GPIO_LABEL(DT_ALIAS(led0), gpios));
	if (!led_port) {
		LOG_ERR("Could not bind to LED port %s",
			DT_GPIO_LABEL(DT_ALIAS(led0), gpios));
		return -EIO;
	}

	nrf_gpio_cfg_output(PIN_IN1);
	nrf_gpio_cfg_output(PIN_IN2);
	const u8_t pins[] = {DT_GPIO_PIN(DT_ALIAS(led0), gpios),
			     DT_GPIO_PIN(DT_ALIAS(led1), gpios),
			     DT_GPIO_PIN(DT_ALIAS(led2), gpios),
			     DT_GPIO_PIN(DT_ALIAS(led3), gpios)};

	for (size_t i = 0; i < ARRAY_SIZE(pins); i++) {
		int err = gpio_pin_configure(led_port, pins[i], GPIO_OUTPUT);

		if (err) {
			LOG_ERR("Unable to configure LED%u, err %d.", i, err);
			led_port = NULL;
			return err;
		}
	}

	return 0;
}

static void leds_update(u8_t value)
{
	bool led0_status = !(value % 8 > 0 && value % 8 <= 4);
	bool led1_status = !(value % 8 > 1 && value % 8 <= 5);
	bool led2_status = !(value % 8 > 2 && value % 8 <= 6);
	bool led3_status = !(value % 8 > 3);

	gpio_port_pins_t mask =
		1 << DT_GPIO_PIN(DT_ALIAS(led0), gpios) |
		1 << DT_GPIO_PIN(DT_ALIAS(led1), gpios) |
		1 << DT_GPIO_PIN(DT_ALIAS(led2), gpios) |
		1 << DT_GPIO_PIN(DT_ALIAS(led3), gpios);

	gpio_port_value_t val =
		led0_status << DT_GPIO_PIN(DT_ALIAS(led0), gpios) |
		led1_status << DT_GPIO_PIN(DT_ALIAS(led1), gpios) |
		led2_status << DT_GPIO_PIN(DT_ALIAS(led2), gpios) |
		led3_status << DT_GPIO_PIN(DT_ALIAS(led3), gpios);

	if (led_port != NULL) {
		gpio_port_set_masked_raw(led_port, mask, val);
	}
}

void event_handler(struct esb_evt const *event)
{
	switch (event->evt_id) {
	case ESB_EVENT_TX_SUCCESS:
		LOG_DBG("TX SUCCESS EVENT");
		break;
	case ESB_EVENT_TX_FAILED:
		LOG_DBG("TX FAILED EVENT");
		break;
	case ESB_EVENT_RX_RECEIVED:
		if (esb_read_rx_payload(&rx_payload) == 0) {

			leds_update(rx_payload.data[1]);
			LOG_INF("%d %d", rx_payload.data[0], rx_payload.data[1]);

			set(rx_payload.data[1], rx_payload.data[0]);
		} else {
			LOG_ERR("Error while reading rx packet");
		}
		break;
	}
}

int clocks_start(void)
{
	int err;
	struct device *clk;

	clk = device_get_binding(DT_INST_LABEL(0));
	if (!clk) {
		LOG_ERR("Clock device not found!");
		return -EIO;
	}

	err = clock_control_on(clk, CLOCK_CONTROL_NRF_SUBSYS_HF);
	if (err && (err != -EINPROGRESS)) {
		LOG_ERR("HF clock start fail: %d", err);
		return err;
	}

	/* Block until clock is started.
	 */
	while (clock_control_get_status(clk, CLOCK_CONTROL_NRF_SUBSYS_HF) !=
		CLOCK_CONTROL_STATUS_ON) {

	}

	LOG_DBG("HF clock started");
	return 0;
}

int esb_initialize(void)
{
	int err;
	/* These are arbitrary default addresses. In end user products
	 * different addresses should be used for each set of devices.
	 */
	u8_t base_addr_0[4] = {0xE7, 0xE7, 0xE7, 0xE7};
	u8_t base_addr_1[4] = {0xC2, 0xC2, 0xC2, 0xC2};
	u8_t addr_prefix[8] = {0xE7, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8};

	struct esb_config config = ESB_DEFAULT_CONFIG;

	config.protocol = ESB_PROTOCOL_ESB_DPL;
	config.bitrate = ESB_BITRATE_2MBPS;
	config.mode = ESB_MODE_PRX;
	config.event_handler = event_handler;
	config.selective_auto_ack = true;

	err = esb_init(&config);
	if (err) {
		return err;
	}

	err = esb_set_base_address_0(base_addr_0);
	if (err) {
		return err;
	}

	err = esb_set_base_address_1(base_addr_1);
	if (err) {
		return err;
	}

	err = esb_set_prefixes(addr_prefix, ARRAY_SIZE(addr_prefix));
	if (err) {
		return err;
	}

	return 0;
}

void main(void)
{
	int err;

	LOG_INF("Enhanced ShockBurst prx sample");

	err = clocks_start();
	if (err) {
		return;
	}

	leds_init();
	pwm_dev = device_get_binding(PWM_DRIVER);
	if (!led_port) {
		printk("Could not bind to LED port\n");
		return -ENXIO;
	}

	err = esb_initialize();
	if (err) {
		LOG_ERR("ESB initialization failed, err %d", err);
		return;
	}

	LOG_INF("Initialization complete");

	LOG_INF("Setting up for packet receiption");

	err = esb_start_rx();
	if (err) {
		LOG_ERR("RX setup failed, err %d", err);
		return;
	}

	while (1) {
		read_battery_voltage();
		k_sleep(K_MSEC(100));
	}
}

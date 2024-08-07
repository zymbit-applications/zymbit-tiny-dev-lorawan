// /*
//  * Copyright (c) 2023 PureEngineering, LLC
//  */

// #include <stdio.h>
// #include <stdlib.h>
// #include <string.h>
// #include <zephyr/kernel.h>
// #include <zephyr/device.h>
// #include <zephyr/drivers/uart.h>
// #include <zephyr/drivers/lora.h>
// #include <zephyr/drivers/gpio.h>
// #include <zephyr/drivers/sensor.h>

// #include "uart_driver.h"

// #define MAX_LORA_DATA_LEN (255)
// #define lora_thread_STACK_SIZE 2048
// #define lora_thread_PRIORITY 7

// #define LORA_RSSI_CLEAR_THRESHOLD -90
// #define LORA_MAX_CTS_CHECK_MS 1000
// #define LORA_MAX_WAIT_TX_DONE_MS 10000
// #define LORA_MAX_POLL_TIME_MS 1000

// #define LORA_FREQ 915100000

// const struct gpio_dt_spec system_led = GPIO_DT_SPEC_GET_OR(DT_ALIAS(system_led), gpios, {0});
// const struct device *uart_dev = DEVICE_DT_GET(DT_ALIAS(rpc_uart));
// const struct device *lora_dev = DEVICE_DT_GET(DT_ALIAS(lora0));
// static uart_struct uart_ctx;

// static uint8_t uart_rx_buf[1024];
// static uint8_t uart_tx_buf[1024];

// // Simple queue to buffer messages that need to send
// typedef struct
// {
// 	uint8_t data[MAX_LORA_DATA_LEN];
// 	int length;
// } lora_data_t;

// typedef struct
// {
// 	unsigned char *q;		  /* pointer to queue buffer*/
// 	volatile int read_index;  /* index used for reading */
// 	volatile int write_index; /* index used for writing */
// 	int p_size;				  /* size of index in queue */
// 	int size;				  /* max number of indexes */
// } lora_queue_t;

// void lora_queue_init(lora_queue_t *q, int queue_field_size, unsigned char queue_size)
// {
// 	q->read_index = 0;
// 	q->write_index = 0;
// 	q->size = queue_size;
// 	q->p_size = queue_field_size;
// 	q->q = malloc(queue_field_size * queue_size);
// }

// int lora_queue_put(lora_queue_t *q, void *p)
// {
// 	int nextindex = (q->write_index + 1) % q->size;
// 	if (nextindex != q->read_index)
// 	{
// 		int offset = nextindex * q->p_size;
// 		memcpy(q->q + offset, p, q->p_size);
// 		q->write_index = nextindex;
// 		return 1;
// 	}
// 	else
// 	{
// 		return 0; // queue is full
// 	}
// }

// int lora_queue_get(lora_queue_t *q, void *p)
// {
// 	if (q->read_index == q->write_index)
// 	{
// 		return 0; // queue is empty
// 	}
// 	int nextindex = (q->read_index + 1) % q->size;
// 	int offset = nextindex * q->p_size;
// 	memcpy(p, q->q + offset, q->p_size);
// 	q->read_index = nextindex;
// 	return 1;
// }

// lora_queue_t lora_queue;

// ///

// K_THREAD_STACK_DEFINE(lora_thread_stack_area, lora_thread_STACK_SIZE);
// struct k_thread lora_thread_data;

// int16_t lora_rssi = 0;

// int lora_configure(uint32_t frequency)
// {
// 	struct lora_modem_config config;
// 	int ret;
// 	/* Configure LoRa device */
// 	lora_dev = DEVICE_DT_GET(DT_ALIAS(lora0));
// 	if (!device_is_ready(lora_dev))
// 	{
// 		printk("%s Device not found\n", lora_dev->name);
// 		return -EIO;
// 	}
// 	else
// 	{
// 		config.frequency = frequency;
// 		config.bandwidth = BW_500_KHZ;
// 		config.datarate = SF_10;
// 		config.preamble_len = 32;
// 		config.coding_rate = CR_4_5;
// 		config.tx_power = 16;
// 		config.tx = true;
// 		ret = lora_config(lora_dev, &config);
// 		if (ret < 0)
// 		{
// 			printk("LoRa config failed %d\n", ret);
// 			return -EIO;
// 		}
// 		else
// 		{
// 			printk("LoRa config success\n");
// 		}
// 	}

// 	return 0;
// }

// int lora_rx_state = 0;

// void lora_thread(void *unused1, void *unused2, void *unused3)
// {
// 	printk("Started lora_thread\n");

// 	lora_data_t lora_data;
// 	int16_t rssi;
// 	int8_t snr;

// 	lora_configure(LORA_FREQ);

// 	lora_send(lora_dev, "hello lora world\n", 16);

// 	while (1)
// 	{
// 		k_yield();

// 		int length = 0;
// 		lora_rx_state = 1;
// 		if ((length = lora_recv(lora_dev, lora_data.data, MAX_LORA_DATA_LEN, K_MSEC(LORA_MAX_POLL_TIME_MS), &rssi, &snr)) > 0)
// 		{
// 			lora_data.length = length;
// 			gpio_pin_set(system_led.port, system_led.pin, 1);
// 			for (int i = 0; i < length; i++)
// 			{
// 				uart_poll_out(uart_dev, lora_data.data[i]);
// 			}
// 			gpio_pin_set(system_led.port, system_led.pin, 0);
// 		}

// 		lora_rx_state = 0;

// 		while (lora_queue_get(&lora_queue, &lora_data))
// 		{
// 			lora_send(lora_dev, lora_data.data, lora_data.length);
// 		}
// 	}
// }

// #define BME680_DEV_NAME DT_LABEL(DT_INST(0, bosch_bme680))

// void main(void)
// {
// 	uint8_t byte;

// 	printk("*********************************\n");
// 	printk("**       STM32WL LoRa demo     **\n");
// 	printk("*********************************\n");
// 	printk("COMPILE DATE/TIME %s %s\n", __DATE__, __TIME__);
// 	const struct device *bme = device_get_binding(BME680_DEV_NAME);
// 	const struct device *acc = device_get_binding(DT_LABEL(DT_INST(0, st_lis2dw12)));
// 	if (device_is_ready(bme))
// 	{
// 		struct sensor_value temp, hum, pres;
// 		sensor_sample_fetch(bme);
// 		sensor_channel_get(bme, SENSOR_CHAN_AMBIENT_TEMP, &temp);
// 		sensor_channel_get(bme, SENSOR_CHAN_HUMIDITY, &hum);
// 		sensor_channel_get(bme, SENSOR_CHAN_PRESS, &pres);
// 		printk("Temperature: %d.%03d\n", temp.val1, temp.val2);
// 		printk("Humidity: %d.%03d\n", hum.val1, hum.val2);
// 		printk("Pressure: %d.%03d\n", pres.val1, pres.val2);
// 	}

// 	if (device_is_ready(acc))
// 	{
// 		struct sensor_value acc_x, acc_y, acc_z;
// 		sensor_sample_fetch(acc);
// 		sensor_channel_get(acc, SENSOR_CHAN_ACCEL_X, &acc_x);
// 		sensor_channel_get(acc, SENSOR_CHAN_ACCEL_Y, &acc_y);
// 		sensor_channel_get(acc, SENSOR_CHAN_ACCEL_Z, &acc_z);
// 		printk("Acceleration X: %d.%03d\n", acc_x.val1, acc_x.val2);
// 		printk("Acceleration Y: %d.%03d\n", acc_y.val1, acc_y.val2);
// 		printk("Acceleration Z: %d.%03d\n", acc_z.val1, acc_z.val2);
// 	}

// 	lora_data_t lora_data;

// 	lora_queue_init(&lora_queue, sizeof(lora_data_t), 10);

// 	/* Configure system-led */
// 	gpio_pin_configure_dt(&system_led, GPIO_OUTPUT_INACTIVE);

// 	/* Test LED blink */
// 	for (int i = 0; i < 3; i++)
// 	{
// 		gpio_pin_set(system_led.port, system_led.pin, 1);
// 		k_msleep(100);
// 		gpio_pin_set(system_led.port, system_led.pin, 0);

// 		k_msleep(10);
// 	}

// 	/* Get UART device */
// 	if (!device_is_ready(uart_dev))
// 	{
// 		printk("Could not find %s device\n", uart_dev->name);
// 	}

// 	init_uart_driver(&uart_ctx, uart_dev, uart_tx_buf, sizeof(uart_tx_buf), uart_rx_buf, sizeof(uart_rx_buf));

// 	/* Configure LoRa device */
// 	if (!device_is_ready(lora_dev))
// 	{
// 		printk("%s Device not found\n", lora_dev->name);
// 	}
// 	else
// 	{
// 		k_thread_create(&lora_thread_data, lora_thread_stack_area, K_THREAD_STACK_SIZEOF(lora_thread_stack_area), lora_thread, NULL, NULL, NULL, lora_thread_PRIORITY, 0, K_NO_WAIT);
// 	}

// 	while (1)
// 	{

// 		lora_data.length = 0;
// 		while (uart_getchar(&uart_ctx, &byte))
// 		{
// 			uart_poll_out(uart_dev, byte); // echo back to uart

// 			if ((lora_data.length + 1) < MAX_LORA_DATA_LEN)
// 			{
// 				lora_data.data[lora_data.length] = byte;
// 				lora_data.length++;
// 			}
// 			else
// 			{
// 				break;
// 			}
// 		}

// 		if (lora_data.length > 0)
// 		{
// 			lora_queue_put(&lora_queue, &lora_data);
// 		}

// 		k_msleep(1);
// 	}
// }

#include <zephyr/device.h>
#include <zephyr/lorawan/lorawan.h>
#include <zephyr/kernel.h>
#include <zephyr/random/random.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/lora.h>
#include <zephyr/drivers/gpio.h>

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>

#define LOG_LEVEL CONFIG_LOG_DEFAULT_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(hemrmes_zymbit);

// ------------------------------------------ USER CONFIGURABLE SETTINGS ------------------------------------------ //

/*-- UART --*/
#define UART_RESPONSE_BUFFER_SIZE 255 // Max possible size of the uart rx buffer
#define UART_RX_TIMEOUT K_SECONDS(2)  // Amount of time to wait without receiving any packets before sending to uplink

// interface settings
#define BAUDRATE 115200
#define PARITY UART_CFG_PARITY_NONE
#define STOP_BITS UART_CFG_STOP_BITS_1
#define FLOW_CTRL UART_CFG_FLOW_CTRL_NONE
#define DATA_BITS UART_CFG_DATA_BITS_8

/*-- LoRaWAN --*/
/* Customize based on network configuration */
#define LORAWAN_DEV_EUI {0xDD, 0xEE, 0xAA, 0xDD, 0xBB, 0xEE, 0xEE, 0xFF}
#define LORAWAN_JOIN_EUI {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
#define LORAWAN_APP_KEY {0xA1, 0x49, 0xBB, 0xBF, 0x1C, 0x7E, 0x61, 0xBA, 0x31, 0xA7, 0xB3, 0x80, 0x92, 0x7B, 0x12, 0xCA} // Generated server side to be sure then filled in here before joining
#define LORAWAN_SEND_PORT 1
#define LORAWAN_DATARATE LORAWAN_DR_4		 // Set data rate to 4 because payload is 242 bytes
#define LORAWAN_ADR false					 // Set ADR or not.  Have it disabled because it was selecting datarate with payload sizes too small.
#define LORAWAN_CONFIG_MODE LORAWAN_ACT_OTAA // or LORAWAN_ACT_ABP
#define LORAWAN_TX_RETRY_DELAY K_MSEC(10000) // Amount of time to wait before retrying LoRaWAN tx

#define LORAWAN_TX_INTERVAL K_SECONDS(5) // Uplink a message using this interval

uint8_t data_test[] = {0X00, 0X01, 0X00, 0X02, 0X00, 0X00, 0X00, 0X01, 0X00, 0X00, 0X00, 0X09}; // Data to be sent up first after joining network.  Can remove this functionality if desired

// ------------------------------------------ UART SETUP ------------------------------------------ //

// Set UART device from DTS
#define MY_UART DT_ALIAS(rpc_uart)
#if DT_NODE_HAS_STATUS(MY_UART, okay)
const struct device *uart0 = DEVICE_DT_GET(MY_UART);
#else
#error "UART node is disabled"
#endif

static struct k_timer uart_rx_timer;
struct uart_msg
{
	uint8_t data[UART_RESPONSE_BUFFER_SIZE];
	size_t len;
};

/* Queue to store up to 10 messages (aligned to 4-byte boundary) */
K_MSGQ_DEFINE(uart_msgq, sizeof(struct uart_msg), 10, 4);

struct uart_config uart_cfg = {
	.baudrate = BAUDRATE,
	.parity = PARITY,
	.stop_bits = STOP_BITS,
	.flow_ctrl = FLOW_CTRL,
	.data_bits = DATA_BITS,
};

/* Receive buffer used in UART ISR callback */
static uint8_t rx_buf[UART_RESPONSE_BUFFER_SIZE];
static int rx_buf_pos;

/* Timer callback function */
void rx_timeout_handler(struct k_timer *not_used)
{
	if (rx_buf_pos > 0)
	{
		/* Create a message struct */
		struct uart_msg msg;
		memcpy(msg.data, rx_buf, rx_buf_pos);
		msg.len = rx_buf_pos;

		LOG_INF("Adding buffer of size %zu to message queue\n", msg.len);
		/* If queue is full, message is silently dropped */
		k_msgq_put(&uart_msgq, &msg, K_NO_WAIT);

		/* Reset the buffer */
		rx_buf_pos = 0;
	}
}

/*
 * Read characters from UART until line end is detected. Afterwards push the
 * data to the message queue.
 */
void serial_cb(const struct device *dev, void *user_data)
{
	uint8_t c;

	if (!uart_irq_update(dev))
	{
		return;
	}

	if (!uart_irq_rx_ready(dev))
	{
		return;
	}

	/* Read until FIFO empty */
	while (uart_fifo_read(dev, &c, 1) == 1)
	{
		if (rx_buf_pos < sizeof(rx_buf))
		{
			rx_buf[rx_buf_pos++] = c;

			/* Reset the timer on data received */
			k_timer_start(&uart_rx_timer, UART_RX_TIMEOUT, K_NO_WAIT);
		}
	}
}

/*
 * Print a message in hex format to the UART interface
 */
void print_uart_hex(const uint8_t *buf, size_t len)
{
	for (size_t i = 0; i < len; i++)
	{
		printk("%02X ", buf[i]);
	}
	printk("\n");
}

/*
 * Send data out over uart
 */
void send_uart_command(const struct device *uart, uint8_t *command, size_t length)
{
	for (size_t i = 0; i < length; i++)
	{
		uart_poll_out(uart, command[i]);
	}
	// printk("\nSent command: ");
	// print_uart_hex(command, length);
	// OR
	LOG_HEXDUMP_INF(command, length, "Sent Command: ");
}

int uart_init()
{
	if (!device_is_ready(uart0))
	{
		LOG_ERR("UART device not found\n");
		return -1;
	}
	// Configure UART device
	int ret = uart_configure(uart0, &uart_cfg);
	if (ret == -ENOSYS)
	{
		LOG_ERR("Error - Config not supported or can't set config: %d", ret);
		return ret;
	}
	else if (ret == -ENOTSUP)
	{
		LOG_ERR("Error - API not enabled: %d", ret);
		return ret;
	}
	else if (ret < 0)
	{
		LOG_ERR("Error - Failed to configure UART device: %d", ret);
		return ret;
	}

	/* Configure interrupt and callback to receive data */
	ret = uart_irq_callback_user_data_set(uart0, serial_cb, NULL);
	if (ret == -ENOSYS)
	{
		LOG_ERR("Error - UART irq callback function not implemented: %d", ret);
		return ret;
	}
	else if (ret == -ENOTSUP)
	{
		LOG_ERR("Error - API not enabled: %d", ret);
		return ret;
	}

	uart_irq_rx_enable(uart0);

	return 0;
}

extern void uart_listen_msg_queue_thread()
{
	struct uart_msg temp_buf;

	LOG_INF("Waiting for UART message");
	while (k_msgq_get(&uart_msgq, &temp_buf, K_FOREVER) == 0)
	{
		/* Process the message */
		LOG_HEXDUMP_INF(temp_buf.data, temp_buf.len, "Message: ");
		LOG_INF("Sending LoRaWAN Uplink");

		while (lorawan_send(LORAWAN_SEND_PORT, temp_buf.data, temp_buf.len,
							LORAWAN_MSG_CONFIRMED))
		{
			LOG_INF("Retrying Uplink");
		}
		LOG_INF("Data sent!");
	}
}
K_THREAD_DEFINE(uart_rx_thread, 2048, uart_listen_msg_queue_thread, NULL, NULL, NULL, K_LOWEST_APPLICATION_THREAD_PRIO, 0, 0);

// ------------------------------------------ END UART SETUP ------------------------------------------ //

// ------------------------------------------ LORAWAN SETUP ------------------------------------------ //
// Set UART device from DTS
#define MY_LORA DT_ALIAS(lora0)
#if DT_NODE_HAS_STATUS(MY_LORA, okay)
const struct device *lora_dev = DEVICE_DT_GET(MY_LORA);
#else
#error "LoRa node is disabled"
#endif

int send_port = LORAWAN_SEND_PORT;
uint8_t tx_retries = 13;

static struct k_timer lorawan_tx_timer;

static void dl_callback(uint8_t port, bool data_pending,
						int16_t rssi, int8_t snr,
						uint8_t len, const uint8_t *hex_data)
{
	if (hex_data == NULL)
	{
		LOG_INF("ACK Received");
		LOG_INF("length: %i", len);
	}
	else
	{
		uint8_t *temp = &len;
		LOG_INF("Port %d, Pending %d, RSSI %ddB, SNR %ddBm", port, data_pending, rssi, snr);
		LOG_INF("payload length: %i", len);
		LOG_HEXDUMP_INF(hex_data, len, "Payload: ");
		send_uart_command(uart0, temp, 1); // Sending dowlink size first per hermes spec
		send_uart_command(uart0, hex_data, len);
	}
}

static void lorwan_datarate_changed(enum lorawan_datarate dr)
{
	uint8_t unused, max_size;

	lorawan_get_payload_sizes(&unused, &max_size);
	LOG_INF("New Datarate: DR_%d, Max Payload %d", dr, max_size);
}

void lorawan_tx_uplink_timer_handler(struct k_timer *not_used)
{
	// Temp command to send to uart.  Expects a response that will then get uplinked

	uint8_t read_probe_1[] = {0x01, 0x03, 0x01, 0x08, 0x00, 0x01, 0x04, 0x34};
	send_uart_command(uart0, read_probe_1, sizeof(read_probe_1));
}

// int lorawan_init()
// {
// 	struct lorawan_join_config join_cfg;
// 	uint8_t dev_eui[] = LORAWAN_DEV_EUI;
// 	uint8_t join_eui[] = LORAWAN_JOIN_EUI;
// 	uint8_t app_key[] = LORAWAN_APP_KEY;
// 	int ret;

// 	struct lorawan_downlink_cb downlink_cb = {
// 		.port = LW_RECV_PORT_ANY,
// 		.cb = dl_callback};

// 	if (!device_is_ready(lora_dev))
// 	{
// 		LOG_ERR("%s: device not ready.", lora_dev->name);
// 		return -1;
// 	}

// #if defined(CONFIG_LORAMAC_REGION_US915)
// 	/* If more than one region Kconfig is selected, app should set region
// 	 * before calling lorawan_start()
// 	 */
// 	ret = lorawan_set_region(LORAWAN_REGION_US915);
// 	if (ret < 0)
// 	{
// 		LOG_ERR("lorawan_set_region failed: %d", ret);
// 		return ret;
// 	}
// #endif

// 	ret = lorawan_start();
// 	if (ret < 0)
// 	{
// 		LOG_ERR("lorawan_start failed: %d", ret);
// 		return ret;
// 	}
// 	lorawan_register_downlink_callback(&downlink_cb);
// 	lorawan_register_dr_changed_callback(lorwan_datarate_changed);

// 	if (LORAWAN_ADR)
// 	{
// 		// Don't use adr because it might adjust dr to a rate that does not support full payload size
// 		lorawan_enable_adr(true);
// 		if (ret < 0)
// 		{
// 			LOG_ERR("enable_adr failed: %d", ret);
// 			return ret;
// 		}
// 	}
// 	else
// 	{
// 		ret = lorawan_set_datarate(LORAWAN_DATARATE);
// 		if (ret < 0)
// 		{
// 			LOG_ERR("lorawan_set_datarate failed: %d", ret);
// 			return ret;
// 		}

// 		ret = lorawan_set_conf_msg_tries(tx_retries);
// 		if (ret < 0)
// 		{
// 			LOG_ERR("set_message_tries failed: %d", ret);
// 			return ret;
// 		}
// 	}

// 	uint32_t random = sys_rand32_get();
// 	uint16_t dev_nonce = random & 0x0000FFFF;

// 	join_cfg.mode = LORAWAN_CONFIG_MODE;
// 	join_cfg.dev_eui = dev_eui;
// 	join_cfg.otaa.join_eui = join_eui;
// 	join_cfg.otaa.app_key = app_key;
// 	join_cfg.otaa.nwk_key = app_key;
// 	join_cfg.otaa.dev_nonce = dev_nonce;

// 	LOG_INF("Joining network over OTAA");
// 	ret = lorawan_join(&join_cfg);
// 	if (ret < 0)
// 	{
// 		LOG_ERR("lorawan_join_network failed: %d", ret);
// 		return ret;
// 	}

// 	return 0;
// }

// ------------------------------------------ END LORAWAN SETUP ------------------------------------------ //

// Setting the antenna power device to manually drive
const struct gpio_dt_spec ant_pwr = GPIO_DT_SPEC_GET_OR(DT_ALIAS(ant_pwr), gpios, {0});

int main(void)
{
	// struct uart_msg temp_buf;

	/* Initialize UART receive timeout timer */
	k_timer_init(&uart_rx_timer, rx_timeout_handler, NULL);
	// k_timer_init(&lorawan_tx_timer, lorawan_tx_uplink_timer_handler, NULL);

	int ret = uart_init();
	if (ret < 0)
	{
		LOG_ERR("UART init error");
		return 0;
	}

	// configure and drive the antenna power pin high
	ret = gpio_pin_configure_dt(&ant_pwr, GPIO_OUTPUT_ACTIVE);
	if (ret < 0)
	{
		LOG_ERR("ant_pwr gpio error");
		return 0;
	}
	gpio_pin_set(ant_pwr.port, ant_pwr.pin, 1);

	struct lora_modem_config config;
	struct lorawan_join_config join_cfg;
	uint8_t dev_eui[] = LORAWAN_DEV_EUI;
	uint8_t join_eui[] = LORAWAN_JOIN_EUI;
	uint8_t app_key[] = LORAWAN_APP_KEY;

	struct lorawan_downlink_cb downlink_cb = {
		.port = LW_RECV_PORT_ANY,
		.cb = dl_callback};

	if (!device_is_ready(lora_dev))
	{
		LOG_ERR("%s: device not ready.", lora_dev->name);
		return -EIO;
	}
	else
	{
		// config.frequency = frequency;
		// config.bandwidth = BW_500_KHZ;
		// config.datarate = SF_10;
		// config.preamble_len = 32;
		// config.coding_rate = CR_4_5;
		// config.tx_power = 16;
		config.tx = true;
		ret = lora_config(lora_dev, &config);
		if (ret < 0)
		{
			LOG_ERR("LoRa config failed %d", ret);
			return -EIO;
		}
		else
		{
			LOG_INF("LoRa config success");
		}
	}

#if defined(CONFIG_LORAMAC_REGION_US915)
	/* If more than one region Kconfig is selected, app should set region
	 * before calling lorawan_start()
	 */
	ret = lorawan_set_region(LORAWAN_REGION_US915);
	if (ret < 0)
	{
		LOG_ERR("lorawan_set_region failed: %d", ret);
		return 0;
	}
#endif

	ret = lorawan_start();
	if (ret < 0)
	{
		LOG_ERR("lorawan_start failed: %d", ret);
		return 0;
	}
	lorawan_register_downlink_callback(&downlink_cb);
	lorawan_register_dr_changed_callback(lorwan_datarate_changed);

	if (LORAWAN_ADR)
	{
		// Don't use adr because it might adjust dr to a rate that does not support full payload size
		lorawan_enable_adr(true);
		if (ret < 0)
		{
			LOG_ERR("enable_adr failed: %d", ret);
			return 0;
		}
	}
	else
	{
		ret = lorawan_set_datarate(LORAWAN_DATARATE);
		if (ret < 0)
		{
			LOG_ERR("lorawan_set_datarate failed: %d", ret);
			return 0;
		}

		ret = lorawan_set_conf_msg_tries(tx_retries);
		if (ret < 0)
		{
			LOG_ERR("set_message_tries failed: %d", ret);
			return 0;
		}
	}

	uint32_t random = sys_rand32_get();
	uint16_t dev_nonce = random & 0x0000FFFF;

	join_cfg.mode = LORAWAN_CONFIG_MODE;
	join_cfg.dev_eui = dev_eui;
	join_cfg.otaa.join_eui = join_eui;
	join_cfg.otaa.app_key = app_key;
	join_cfg.otaa.nwk_key = app_key;
	join_cfg.otaa.dev_nonce = dev_nonce;

	LOG_INF("Joining network over OTAA");
	ret = lorawan_join(&join_cfg);
	if (ret < 0)
	{
		LOG_ERR("lorawan_join_network failed: %d", ret);
		return 0;
	}
	// Start the uplink timer. On the specified interval it will send a uart command to the device and expect a response which will then be uplinked through the uart message handling
	// k_timer_start(&lorawan_tx_timer, LORAWAN_TX_INTERVAL, K_NO_WAIT);

	LOG_INF("Sending data...");
	while (1)
	{
		ret = lorawan_send(send_port, data_test, sizeof(data_test),
						   LORAWAN_MSG_CONFIRMED);

		/*
		 * Note: The stack may return -EAGAIN if the provided data
		 * length exceeds the maximum possible one for the region and
		 * datarate. But since we are just sending the same data here,
		 * we'll just continue.
		 */
		if (ret == -EAGAIN)
		{
			LOG_ERR("lorawan_send failed: %d. Continuing...", ret);
			k_sleep(LORAWAN_TX_RETRY_DELAY);
			continue;
		}
		else if (ret < 0)
		{
			LOG_ERR("lorawan_send failed: %d", ret);
			continue;
		}
		else
		{
			LOG_INF("Data sent!");
			// break;
		}

		k_sleep(LORAWAN_TX_INTERVAL);
	}
}

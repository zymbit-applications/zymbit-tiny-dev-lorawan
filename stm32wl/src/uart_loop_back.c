/*
    To flash this program, change which file is built in CMakeLists.txt
*/

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>

#include <string.h>

/* change this to any other UART peripheral if desired */
#define UART_DEVICE_NODE DT_CHOSEN(zephyr_shell_uart)

#define MY_UART DT_ALIAS(rpc_uart)
#if DT_NODE_HAS_STATUS(MY_UART, okay)
const struct device *const usart = DEVICE_DT_GET(MY_UART);
#else
#error "Node is disabled"
#endif

#define MSG_SIZE 32

/* queue to store up to 10 messages (aligned to 4-byte boundary) */
K_MSGQ_DEFINE(uart_msgq, MSG_SIZE, 10, 4);

static const struct device *const uart_dev = DEVICE_DT_GET(UART_DEVICE_NODE);
const struct gpio_dt_spec system_led = GPIO_DT_SPEC_GET_OR(DT_ALIAS(system_led), gpios, {0});

/* receive buffer used in UART ISR callback */
static char rx_buf[MSG_SIZE];
static int rx_buf_pos;

static char rx_buf_2[MSG_SIZE];
static int rx_buf_2_pos;

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

    /* read until FIFO empty */
    while (uart_fifo_read(dev, &c, 1) == 1)
    {
        if ((c == '\n' || c == '\r') && rx_buf_pos > 0)
        {
            /* terminate string */
            rx_buf[rx_buf_pos] = '\0';

            /* if queue is full, message is silently dropped */
            k_msgq_put(&uart_msgq, &rx_buf, K_NO_WAIT);

            /* reset the buffer (it was copied to the msgq) */
            rx_buf_pos = 0;
        }
        else if (rx_buf_pos < (sizeof(rx_buf) - 1))
        {
            rx_buf[rx_buf_pos++] = c;
        }
        /* else: characters beyond buffer size are dropped */
    }
}

void serial_cb_usart(const struct device *dev, void *user_data)
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

    /* read until FIFO empty */
    while (uart_fifo_read(dev, &c, 1) == 1)
    {
        if ((c == '\n' || c == '\r') && rx_buf_2_pos > 0)
        {
            /* terminate string */
            rx_buf_2[rx_buf_2_pos] = '\0';

            /* if queue is full, message is silently dropped */
            // k_msgq_put(&uart_msgq, &rx_buf, K_NO_WAIT);
            printk("Usart recieved: ");
            printk(rx_buf_2);
            printk("\n");

            /* reset the buffer (it was copied to the msgq) */
            rx_buf_2_pos = 0;
        }
        else if (rx_buf_2_pos < (sizeof(rx_buf_2) - 1))
        {
            rx_buf_2[rx_buf_2_pos++] = c;
        }
        /* else: characters beyond buffer size are dropped */
    }
}

/*
 * Print a null-terminated string character by character to the UART interface
 */
void print_uart(const struct device *dev, char *buf)
{
    int msg_len = strlen(buf);

    for (int i = 0; i < msg_len; i++)
    {
        uart_poll_out(dev, buf[i]);
    }
}

int main(void)
{

    /* Configure system-led */
    gpio_pin_configure_dt(&system_led, GPIO_OUTPUT_INACTIVE);

    /* Test LED blink */
    for (int i = 0; i < 3; i++)
    {
        gpio_pin_set(system_led.port, system_led.pin, 1);
        k_msleep(500);
        gpio_pin_set(system_led.port, system_led.pin, 0);

        k_msleep(500);
    }

    char tx_buf[MSG_SIZE];

    if (!device_is_ready(uart_dev))
    {
        printk("UART device not found!");
        return 0;
    }

    if (!device_is_ready(usart))
    {
        printk("UART device not found!");
        return 0;
    }

    /* configure interrupt and callback to receive data */
    int ret = uart_irq_callback_user_data_set(uart_dev, serial_cb, NULL);
    ret = uart_irq_callback_user_data_set(usart, serial_cb_usart, NULL);

    if (ret < 0)
    {

        if (ret == -ENOTSUP)
        {
            printk("Interrupt-driven UART API support not enabled\n");
        }
        else if (ret == -ENOSYS)
        {
            printk("UART device does not support interrupt-driven API\n");
        }
        else
        {
            printk("Error setting UART callback: %d\n", ret);
        }
        return 0;
    }
    uart_irq_rx_enable(uart_dev);
    uart_irq_rx_enable(usart);

    print_uart(uart_dev, "Hello! I'm your echo bot.\r\n");
    print_uart(uart_dev, "Tell me something and press enter:\r\n");

    for (int i = 0; i < 3; i++)
    {
        gpio_pin_set(system_led.port, system_led.pin, 1);
        k_msleep(500);
        gpio_pin_set(system_led.port, system_led.pin, 0);

        k_msleep(500);
    }

    /* indefinitely wait for input from the user */
    while (k_msgq_get(&uart_msgq, &tx_buf, K_FOREVER) == 0)
    {
        print_uart(uart_dev, "Echo: ");
        print_uart(uart_dev, tx_buf);
        print_uart(uart_dev, "\r\n");

        print_uart(usart, tx_buf);
        print_uart(usart, "\r\n");
    }
    return 0;
}

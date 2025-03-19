#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/iterable_sections.h>

// Set up hardware
#define LED0_NODE DT_ALIAS(led0)
#define LED1_NODE DT_ALIAS(led1)

static const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
static const struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(LED1_NODE, gpios);
 
// Create timer to periodically pack i2s queue
static void i2s_pack_isr(struct k_timer *dummy)
{
	static bool flip = true;
	if (flip)
	{
		gpio_pin_toggle_dt(&led0);
	} else
	{
		gpio_pin_toggle_dt(&led1);
	}
	
	flip = !flip;
}

K_TIMER_DEFINE(i2s_pack_timer, i2s_pack_isr, NULL);

#define SAMPLE_NO 64
 
/* The data represent a sine wave */
static int16_t data[SAMPLE_NO] = {
   3211,   6392,   9511,  12539,  15446,  18204,  20787,  23169,
  25329,  27244,  28897,  30272,  31356,  32137,  32609,  32767,
  32609,  32137,  31356,  30272,  28897,  27244,  25329,  23169,
  20787,  18204,  15446,  12539,   9511,   6392,   3211,      0,
  -3212,  -6393,  -9512, -12540, -15447, -18205, -20788, -23170,
 -25330, -27245, -28898, -30273, -31357, -32138, -32610, -32767,
 -32610, -32138, -31357, -30273, -28898, -27245, -25330, -23170,
 -20788, -18205, -15447, -12540,  -9512,  -6393,  -3212,     -1,
};
 
/* Fill buffer with sine wave on left channel, and sine wave shifted by
 * 90 degrees on right channel. "att" represents a power of two to attenuate
 * the samples by
 */
static void fill_buf(int16_t *tx_block, int att)
{
 for (int i = 0; i < SAMPLE_NO; i++) {
	 /* Left channel is sine wave */
	 tx_block[2 * i] = data[i] / (1 << att);
	 /* Right channel is same sine wave, shifted by 90 degrees */
	 tx_block[2 * i + 1] = data[i] / (1 << att);
 }
}
 
#define NUM_BLOCKS 20
#define BLOCK_SIZE (2 * sizeof(data))
 
#ifdef CONFIG_NOCACHE_MEMORY
 #define MEM_SLAB_CACHE_ATTR __nocache
#else
 #define MEM_SLAB_CACHE_ATTR
#endif /* CONFIG_NOCACHE_MEMORY */
 
static char MEM_SLAB_CACHE_ATTR __aligned(WB_UP(32))
 _k_mem_slab_buf_tx_0_mem_slab[(NUM_BLOCKS) * WB_UP(BLOCK_SIZE)];
 
static STRUCT_SECTION_ITERABLE(k_mem_slab, tx_0_mem_slab) =
 Z_MEM_SLAB_INITIALIZER(tx_0_mem_slab, _k_mem_slab_buf_tx_0_mem_slab,
			 WB_UP(BLOCK_SIZE), NUM_BLOCKS);
 

int main(void)
{
	int ret_led;
	if (!gpio_is_ready_dt(&led0)) {
		return 0;
	}

	ret_led = gpio_pin_configure_dt(&led0, GPIO_OUTPUT_ACTIVE);
	if (ret_led < 0) {
		return 0;
	}
	ret_led = gpio_pin_configure_dt(&led1, GPIO_OUTPUT_ACTIVE);
	if (ret_led < 0) {
		return 0;
	}
	k_timer_start(&i2s_pack_timer, K_MSEC(500), K_MSEC(500)/* K_MSEC(1 * BLOCK_SIZE) */);

	void *tx_block[NUM_BLOCKS];
	struct i2s_config i2s_cfg;
	int ret;
	uint32_t tx_idx;
	const struct device *dev_i2s = DEVICE_DT_GET(DT_NODELABEL(i2s_rxtx));

	if (!device_is_ready(dev_i2s)) {
		printf("I2S device not ready\n");
		return -ENODEV;
	}
	/* Configure I2S stream */
	i2s_cfg.word_size = 16U;
	i2s_cfg.channels = 2U;
	i2s_cfg.format = I2S_FMT_DATA_FORMAT_I2S;
	i2s_cfg.frame_clk_freq = 44100;
	i2s_cfg.block_size = BLOCK_SIZE;
	i2s_cfg.timeout = 2000;
	/* Configure the Transmit port as Master */
	i2s_cfg.options = I2S_OPT_FRAME_CLK_MASTER
			| I2S_OPT_BIT_CLK_MASTER;
	i2s_cfg.mem_slab = &tx_0_mem_slab;
	ret = i2s_configure(dev_i2s, I2S_DIR_TX, &i2s_cfg);
	if (ret < 0) {
		printf("Failed to configure I2S stream\n");
		return ret;
	}

	/* Prepare all TX blocks */
	for (tx_idx = 0; tx_idx < NUM_BLOCKS; tx_idx++) {
		ret = k_mem_slab_alloc(&tx_0_mem_slab, &tx_block[tx_idx],
						K_FOREVER);
		if (ret < 0) {
			printf("Failed to allocate TX block\n");
			return ret;
		}
		fill_buf((uint16_t *)tx_block[tx_idx], 2);
	}

	tx_idx = 0;
	/* Send first block */
	ret = i2s_write(dev_i2s, tx_block[tx_idx++], BLOCK_SIZE);
	if (ret < 0) {
		printf("Could not write TX buffer %d\n", tx_idx);
		return ret;
	}
	/* Trigger the I2S transmission */
	ret = i2s_trigger(dev_i2s, I2S_DIR_TX, I2S_TRIGGER_START);
	if (ret < 0) {
		printf("Could not trigger I2S tx\n");
		return ret;
	}

	for (; tx_idx < NUM_BLOCKS * 10000; ) {
		ret = i2s_write(dev_i2s, (tx_block[tx_idx]), BLOCK_SIZE);
		tx_idx = (tx_idx+1)%2;
		fill_buf((uint16_t *)tx_block[tx_idx], 2);
		if (ret < 0) {
			printf("Could not write TX buffer %d\n", tx_idx);
			return ret;
		}
	}
	/* Drain TX queue */
	ret = i2s_trigger(dev_i2s, I2S_DIR_TX, I2S_TRIGGER_DRAIN);
	if (ret < 0) {
		printf("Could not trigger I2S tx\n");
		return ret;
	}
	printf("All I2S blocks written\n");
	return 0;
}

// /*
//  * Copyright (c) 2020 Nordic Semiconductor ASA
//  *
//  * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
//  */

// #include <string.h>
// #include <stdio.h>
// #include <zephyr/kernel.h>
// #include <zephyr/device.h>
// #include <zephyr/devicetree.h>
// #include <zephyr/logging/log.h>
// #include <zephyr/drivers/uart.h>
// #include <zephyr/drivers/gpio.h>

// // GPIO Defines
// #define SLEEP_TIME_MS 10 * 60 * 1000
// static const struct gpio_dt_spec button0 = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);
// static const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);

// // Button press callback
// void button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
// {
// 	gpio_pin_toggle_dt(&led0);
// }

// static struct gpio_callback button_cb_data;


// LOG_MODULE_REGISTER(app);

// #define BUF_SIZE 64
// static K_MEM_SLAB_DEFINE(uart_slab, BUF_SIZE, 3, 4);

// #define TX_DATA_SIZE 5
// static const uint8_t tx_buf[TX_DATA_SIZE] = {1, 2, 3, 4, 5};
// static uint8_t buffer_print[TX_DATA_SIZE * 5 + 1];

// static void to_display_format(const uint8_t *src, size_t size, char *dst)
// {
// 	size_t i;

// 	for (i = 0; i < size; i++) {
// 		sprintf(dst + 5 * i, "0x%02x,", src[i]);
// 	}
// }

// static void verify_buffers(const uint8_t *tx_data, char *rx_data, size_t size)
// {
// 	if (memcmp(tx_data, rx_data, size)) {
// 		to_display_format(tx_data, size, buffer_print);
// 		LOG_ERR("Buffer contents are different: %s", buffer_print);
// 		to_display_format(rx_data, size, buffer_print);
// 		LOG_ERR("                           vs: %s", buffer_print);
// 		__ASSERT(false, "Buffer contents are different");
// 	}
// }

// static void uart_irq_handler(const struct device *dev, void *context)
// {
// 	if (uart_irq_tx_ready(dev)) {
// 		(void)uart_fifo_fill(dev, tx_buf, TX_DATA_SIZE);
// 		uart_irq_tx_disable(dev);
// 		LOG_INF("IRQ Tx sent %d bytes", TX_DATA_SIZE);
// 	}

// 	if (uart_irq_rx_ready(dev)) {
// 		uint8_t buf[TX_DATA_SIZE + 1];
// 		int len = uart_fifo_read(dev, buf, sizeof(buf));

// 		LOG_INF("IRQ Received data %d bytes", len);
// 		__ASSERT(len == 1 || len == TX_DATA_SIZE, "Received unexpected data");
// 		if (len == TX_DATA_SIZE) {
// 			verify_buffers(tx_buf, buf, TX_DATA_SIZE);
// 		}
// 	}
// }

// static void interrupt_driven(const struct device *dev)
// {
// 	uint8_t c = 0xff;

// 	uart_irq_callback_set(dev, uart_irq_handler);
// 	uart_irq_rx_enable(dev);
// 	while (1) {
// 		uart_irq_tx_enable(dev);
// 		k_sleep(K_MSEC(500));

// 		uart_poll_out(dev, c);
// 		k_sleep(K_MSEC(100));
// 	}
// }

// static void uart_callback(const struct device *dev, struct uart_event *evt, void *user_data)
// {
// 	struct device *uart = user_data;
// 	int err;

// 	switch (evt->type) {
// 	case UART_TX_DONE:
// 		LOG_INF("ASYNC Tx sent %d bytes", evt->data.tx.len);
// 		break;

// 	case UART_TX_ABORTED:
// 		LOG_ERR("ASYNC Tx aborted");
// 		break;

// 	case UART_RX_RDY:
// 		LOG_INF("ASYNC Received data %d bytes", evt->data.rx.len);
// 		__ASSERT(evt->data.rx.len == 1 || evt->data.rx.len == TX_DATA_SIZE,
// 			 "Received unexpected data");
// 		if (evt->data.rx.len == TX_DATA_SIZE) {
// 			verify_buffers(tx_buf, evt->data.rx.buf + evt->data.rx.offset,
// 				       TX_DATA_SIZE);
// 		}
// 		break;

// 	case UART_RX_BUF_REQUEST: {
// 		uint8_t *buf;

// 		err = k_mem_slab_alloc(&uart_slab, (void **)&buf, K_NO_WAIT);
// 		__ASSERT(err == 0, "Failed to allocate slab");

// 		err = uart_rx_buf_rsp(uart, buf, BUF_SIZE);
// 		__ASSERT(err == 0, "Failed to provide new buffer");
// 		break;
// 	}

// 	case UART_RX_BUF_RELEASED:
// 		k_mem_slab_free(&uart_slab, (void *)evt->data.rx_buf.buf);
// 		break;

// 	case UART_RX_DISABLED:
// 		break;

// 	case UART_RX_STOPPED:
// 		break;
// 	}
// }

// static void async(const struct device *lpuart)
// {
// 	int err;
// 	uint8_t *buf;

// 	err = k_mem_slab_alloc(&uart_slab, (void **)&buf, K_NO_WAIT);
// 	__ASSERT(err == 0, "Failed to alloc slab");

// 	err = uart_callback_set(lpuart, uart_callback, (void *)lpuart);
// 	__ASSERT(err == 0, "Failed to set callback");

// 	err = uart_rx_enable(lpuart, buf, BUF_SIZE, 10000);
// 	__ASSERT(err == 0, "Failed to enable RX");

// 	while (1) {
// 		err = uart_tx(lpuart, tx_buf, sizeof(tx_buf), 10000);
// 		__ASSERT(err == 0, "Failed to initiate transmission");

// 		k_sleep(K_MSEC(500));

// 		uart_poll_out(lpuart, tx_buf[0]);
// 		k_sleep(K_MSEC(100));
// 	}
// }

// int main(void)
// {
// 	// GPIO Setup
// 	int ret;
// 	if (!device_is_ready(led0.port)) {
// 		return -1;
// 	}

//     if (!device_is_ready(button0.port)) {
// 		return -1;
// 	}

//     ret = gpio_pin_configure_dt(&led0, GPIO_OUTPUT_ACTIVE);
//     if (ret < 0) {
// 		return -1;
// 	}

//     ret = gpio_pin_configure_dt(&button0, GPIO_INPUT);
//     if (ret < 0) {
// 		return -1;
//     }

//     ret = gpio_pin_interrupt_configure_dt(&button0, GPIO_INT_EDGE_TO_ACTIVE);
//     gpio_init_callback(&button_cb_data, button_pressed, BIT(button0.pin));
//     gpio_add_callback(button0.port, &button_cb_data);

// 	//UART setup
// 	const struct device *lpuart = DEVICE_DT_GET(DT_NODELABEL(lpuart));

// 	__ASSERT(device_is_ready(lpuart), "LPUART device not ready");

// 	if (IS_ENABLED(CONFIG_NRF_SW_LPUART_INT_DRIVEN)) {
// 		interrupt_driven(lpuart);
// 	} else {
// 		async(lpuart);
// 	}

// 	return 0;
// }

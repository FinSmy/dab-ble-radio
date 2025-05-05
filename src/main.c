#include <string.h>
#include <nrf.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(i2s_transmission, LOG_LEVEL_DBG);

/* Prepare for 32 sample values */
#define NUM_SAMPLES 32

/* Sine Wave Sample */
static int16_t data_frame[NUM_SAMPLES] = {
	  6392,  12539,  18204,  23169,  27244,  30272,  32137,  32767,  32137,
	 30272,  27244,  23169,  18204,  12539,   6392,      0,  -6393, -12540,
	-18205, -23170, -27245, -30273, -32138, -32767, -32138, -30273, -27245,
	-23170, -18205, -12540,  -6393,     -1,
};

// In case transmission must be restarted or if data is not received in time
static int16_t zero_frame[2 * NUM_SAMPLES] = {
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};

/* The size of the memory block should be a multiple of data_frame size */
#define BLOCK_SIZE (2 * sizeof(data_frame))

/* The number of memory blocks in a slab has to be at least 2 per queue */
#define NUM_BLOCKS 6

/* Define a new Memory Slab which consists of NUM_BLOCKS blocks
   __________________________________________________________________________________________
  |    Block 0   |    Block 1   |    Block 2   |    Block 3   |    Block 4   |    Block 5   |
  |    0...31    |    0...31    |    0...31    |    0...31    |    0...31    |    0...31    |
  |______________|______________|______________|______________|______________|______________|
*/
static K_MEM_SLAB_DEFINE(mem_slab, BLOCK_SIZE, NUM_BLOCKS, NUM_SAMPLES);
void* mem_blocks;

/* Get I2S device from the devicetree */
const struct device *i2s_dev = DEVICE_DT_GET(DT_NODELABEL(i2s_tx));
struct i2s_config i2s_cfg;

/* FIFO for holding audio data to be sent to i2s tx */
static K_FIFO_DEFINE(rx_samples_fifo);
int fifo_count = 0;
#define NUM_AUDIO_BLOCKS_FIFO 2
typedef struct audio_data {
	void *fifo_reserved; // Required since FIFO's internal structure is that of a linked list
	int16_t data_buffer[NUM_SAMPLES * NUM_AUDIO_BLOCKS_FIFO];
} audio_data_t;

#define FIFO_MEM_SIZE 10
audio_data_t fifo_memory[FIFO_MEM_SIZE];
int idx_fifo_memory_write = 0;
int idx_fifo_memory_read = 0;

/* Semaphore tracking how many audio buffers we have to load into FIFO */
K_SEM_DEFINE(new_rx_audio_samps_sem, 0, 10);
int sem_value = 0;
int sem_total = 0;

// TODO remove
int big_count = 0;

bool i2s_init()
{
	if (!device_is_ready(i2s_dev)) {
		printk("%s is not ready\n", i2s_dev->name);
		return false;
	}

	/* Configure the I2S device */
	i2s_cfg.word_size = 16;
	i2s_cfg.channels = 2; // L + R channel
	i2s_cfg.format = I2S_FMT_DATA_FORMAT_I2S;
	i2s_cfg.options = I2S_OPT_BIT_CLK_MASTER | I2S_OPT_FRAME_CLK_MASTER;
	i2s_cfg.frame_clk_freq = 32000;
	i2s_cfg.mem_slab = &mem_slab;
	i2s_cfg.block_size = BLOCK_SIZE;
	i2s_cfg.timeout = 10000;
	int ret = i2s_configure(i2s_dev, I2S_DIR_TX, &i2s_cfg);
	if (ret < 0) {
		printk("Failed to configure the I2S stream: (%d)\n", ret);
		return false; 
	}

	/* Start the transmission of data */
	ret = i2s_trigger(i2s_dev, I2S_DIR_TX, I2S_TRIGGER_START);
	if (ret < 0) {
		printk("Failed to start the transmission: %d\n", ret);
		return false;
	}

	LOG_DBG("i2s transmission started\n");

	/* Write Data */
	// Attempt to pack buffer with dummy data to keep i2s interface fed
	ret = i2s_buf_write(i2s_dev, data_frame, sizeof(data_frame));
	if (ret < 0) {
		printk("Error: first i2s_write failed with %d\n", ret);
	}

	ret = i2s_buf_write(i2s_dev, data_frame, sizeof(data_frame));
	if (ret < 0) {
		printk("Error: first i2s_write failed with %d\n", ret);
	}

	ret = i2s_buf_write(i2s_dev, data_frame, sizeof(data_frame));
	if (ret < 0) {
		printk("Error: first i2s_write failed with %d\n", ret);
	}

	ret = i2s_buf_write(i2s_dev, data_frame, sizeof(data_frame));
	if (ret < 0) {
		printk("Error: first i2s_write failed with %d\n", ret);
	}
	LOG_DBG("Finished\n");
    return true;
}

void write_to_i2s_buffer()
{
	LOG_DBG("Inside\n");

	// Try configuring i2s in first time write_to_i2s_buffer is called
	/* Initialise i2s device */
	bool i2s_ret = i2s_init();

	LOG_DBG("Size of FIFO after initialising i2s = %d\n", fifo_count);
	if (!i2s_ret)
	{
		LOG_ERR("Failed to initialise i2s peripheral");
	}

	while (1)
	{
		// Get audio samples out of FIFO
		audio_data_t *rx_samp = k_malloc(sizeof(audio_data_t)); // Should be using k_aligned_alloc?
		rx_samp = k_fifo_get(&rx_samples_fifo, K_NO_WAIT);
		fifo_count--;

		int ret = i2s_buf_write(i2s_dev, &rx_samp->data_buffer, BLOCK_SIZE);

		if (ret < 0) {
			LOG_ERR("Error: i2s_write failed with %d\n", ret);
			
			// Attempt to restart i2s transmission
			i2s_trigger(i2s_dev, I2S_DIR_TX, I2S_TRIGGER_PREPARE);
			int ret = i2s_buf_write(i2s_dev, &zero_frame, BLOCK_SIZE);
			if (ret < 0) {
				LOG_ERR("Issue in restarting i2s transmission");
			}
		}

		// k_free(rx_samp);
	}
}
	
void audio_receive()
{
	LOG_DBG("inside");
	
	while(1)
	{
		// Place into FIFO if an input audio buffer is available
		k_sem_take(&new_rx_audio_samps_sem, K_FOREVER);
		sem_value -= 1;

		audio_data_t *rx_data = &fifo_memory[idx_fifo_memory_write];
		if (rx_data != NULL)
		{
			for(int i = 0; i < NUM_AUDIO_BLOCKS_FIFO * NUM_SAMPLES; i++)
			{
				rx_data->data_buffer[i] = big_count;
				big_count = (big_count + 1) % 10000;
			}

			k_fifo_put(&rx_samples_fifo, rx_data);
			idx_fifo_memory_write = (idx_fifo_memory_write + 1) % FIFO_MEM_SIZE;
			fifo_count++;
		}
	}
	LOG_DBG("exiting");
}
	
static void pack_fifo_isr(struct k_timer *dummy)
{
	k_sem_give(&new_rx_audio_samps_sem);
	sem_value += 1;
	sem_total += 1;
}

/* Timer for filling of FIFO buffer */
K_TIMER_DEFINE(fifo_fill_tmr, pack_fifo_isr, NULL);

K_THREAD_DEFINE(audio_receive_id, 1024, audio_receive, NULL, NULL, NULL, 4, 0, 0);
K_THREAD_DEFINE(write_i2s_buff_id, 1024, write_to_i2s_buffer, NULL, NULL, NULL, 3, 0, 3);
	
int main(void) {
	LOG_INF("Start of main");
	k_timer_start(&fifo_fill_tmr, K_USEC(0), K_USEC(1000));
}
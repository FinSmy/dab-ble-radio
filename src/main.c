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

/* The size of the memory block should be a multiple of data_frame size */
#define BLOCK_SIZE (2 * sizeof(data_frame))

/* The number of memory blocks in a slab has to be at least 2 per queue */
#define NUM_BLOCKS 6

/* Define a new Memory Slab which consistes of NUM_BLOCKS blocks
   ____________________________________________________________
  |    Block 0   |    Block 1   |    Block 2   |    Block 3   |
  |    0...31    |    0...31    |    0...31    |    0...31    |
  |______________|______________|______________|______________|
*/
static K_MEM_SLAB_DEFINE(mem_slab, BLOCK_SIZE, NUM_BLOCKS, NUM_SAMPLES);
void* mem_blocks;

/* Get I2S device from the devicetree */
const struct device *i2s_dev = DEVICE_DT_GET(DT_NODELABEL(i2s_tx));
struct i2s_config i2s_cfg;

/* FIFO for holding audio data to be sent to i2s tx */
static K_FIFO_DEFINE(rx_samples_fifo);
#define NUM_AUDIO_BLOCKS_FIFO 2
typedef struct audio_data {
	void *fifo_reserved; // Required since FIFO's internal structure is that of a linked list
	int16_t data_buffer[NUM_SAMPLES * NUM_AUDIO_BLOCKS_FIFO];
} audio_data_t;

/* Semaphore tracking how many audio buffers we have to load into FIFO */
K_SEM_DEFINE(new_rx_audio_samps_sem, 0, 10);
int sem_value = 0;
int sem_total = 0;

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
	i2s_cfg.timeout = 0;
	int ret = i2s_configure(i2s_dev, I2S_DIR_TX, &i2s_cfg);
	if (ret < 0) {
		printk("Failed to configure the I2S stream: (%d)\n", ret);
		return false; 
	}

	/* Allocate the memory blocks (tx buffer) from the slab and
	set everything to 0 */
	ret = k_mem_slab_alloc(&mem_slab, &mem_blocks, K_NO_WAIT);
	if (ret < 0) {
		printk("Failed to allocate the memory blocks: %d\n", ret);
		return false;
	}
	memset((uint16_t*)mem_blocks, 0, NUM_SAMPLES * NUM_BLOCKS);

	/* Start the transmission of data */
	ret = i2s_trigger(i2s_dev, I2S_DIR_TX, I2S_TRIGGER_START);
	if (ret < 0) {
		printk("Failed to start the transmission: %d\n", ret);
		return false;
	}

	LOG_DBG("i2s transmission started\n");

	ret = i2s_write(i2s_dev, mem_blocks, BLOCK_SIZE);
	if (ret < 0) {
		printk("(non-buffer) write failed with: %d\n", ret);
		return false;
	}

	/* Write Data */
	ret = i2s_buf_write(i2s_dev, mem_blocks, BLOCK_SIZE);
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
	if (!i2s_ret)
	{
		LOG_ERR("Failed to initialise i2s peripheral");
	}

	while (1)
	{
		// Get audio samples out of FIFO
		audio_data_t *rx_samp;
		// LOG_DBG("Trying to get samples from FIFO\n");
		rx_samp = k_fifo_get(&rx_samples_fifo, K_FOREVER);
		LOG_DBG("Got sample from FIFO\n");

		// ! Believe I should be allocating buffer here everytime using k_mem_slab_alloc with a new pointer
		// Update - think I may need to create new pointer rather than mem_blocks
		void* new_mem_blocks;
		int ret = k_mem_slab_alloc(&mem_slab, &new_mem_blocks, K_FOREVER);
		if (ret < 0) {
			printk("Failed to allocate the memory blocks: %d\n", ret);
		}
		
		/* Put data into the tx buffer */
		if (rx_samp != NULL)
		{
			for (int i = 0; i < BLOCK_SIZE; i++) {
				((int16_t*)new_mem_blocks)[i] = rx_samp->data_buffer[i % NUM_SAMPLES];
			}
		}
		else
		{
			LOG_DBG("rx_samp wasn null");
		}
		// LOG_DBG("Before free");
		// k_free(rx_samp);
		// LOG_DBG("After free");
		
		/* Write Data */
		// LOG_DBG("About to write to i2s tx buffer\n");
		// printk("First value: %d\n", ((int16_t*)mem_blocks)[0]);
		// printk("Second value: %d\n", ((int16_t*)mem_blocks)[1]);
		ret = i2s_write(i2s_dev, new_mem_blocks, BLOCK_SIZE);
		// LOG_DBG("Wrote to i2s tx buffer\n");
		if (ret < 0) {
			printk("Error: i2s_write failed with %d\n", ret);
			// return;
		}
		// LOG_DBG("Wrote data");
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
		
		// if (sem_total % 10 == 0)
		// {
		// 	LOG_DBG("taking: sem_k = %d\n", sem_value);
		// }

		audio_data_t *rx_data = k_malloc(sizeof(audio_data_t));
		if (rx_data != NULL)
		{
			for(int i = 0; i < NUM_AUDIO_BLOCKS_FIFO * NUM_SAMPLES; i++)
			{
				rx_data->data_buffer[i] = data_frame[i % NUM_SAMPLES];
			}
			k_fifo_put(&rx_samples_fifo, &rx_data);
		}
	}
	LOG_DBG("exiting");
}
	
static void pack_fifo_isr(struct k_timer *dummy)
{
	// LOG_DBG("pack_fifo_isr");
	k_sem_give(&new_rx_audio_samps_sem);
	sem_value += 1;
	sem_total += 1;
	// if (sem_total % 10 == 0)
	// {
	// 	LOG_DBG("giving: sem_k = %d with total of %d\n", sem_value, sem_total);
	// }
}

/* Timer for filling of FIFO buffer */
K_TIMER_DEFINE(fifo_fill_tmr, pack_fifo_isr, NULL);

K_THREAD_DEFINE(audio_receive_id, 1024, audio_receive, NULL, NULL, NULL, 4, 0, 0);
K_THREAD_DEFINE(write_i2s_buff_id, 1024, write_to_i2s_buffer, NULL, NULL, NULL, 3, 0, 3);
	
int main(void) {
	LOG_INF("Start of main");
	k_timer_start(&fifo_fill_tmr, K_USEC(0), K_USEC(1000));
}
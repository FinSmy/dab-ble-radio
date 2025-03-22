#include <string.h>
#include <nrf.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/drivers/gpio.h>


// Tx sample rate 1625000 / R value 88
#define SAMPLE_FREQUENCY    18465
//#define SAMPLE_FREQUENCY    16000
#define SAMPLE_BIT_WIDTH    16
#define BYTES_PER_SAMPLE    sizeof(int16_t)
#define NUMBER_OF_CHANNELS  1
/* Such block length provides an echo with the delay of 100 ms. */
#define SAMPLES_PER_BLOCK   370//((SAMPLE_FREQUENCY / 10) * NUMBER_OF_CHANNELS)
#define TIMEOUT             10
#define BLOCK_SIZE  (BYTES_PER_SAMPLE * SAMPLES_PER_BLOCK)
#define BLOCK_COUNT (40)
static K_MEM_SLAB_DEFINE(mem_slab, BLOCK_SIZE, BLOCK_COUNT, 4);

const struct device *i2s_dev_tx = DEVICE_DT_GET(DT_NODELABEL(i2s_tx));
struct i2s_config config;
static K_FIFO_DEFINE(fifo_d1stream_data);
//D1Stream FIFO data struct
struct d1stream_data_t {
    sys_snode_t snode;
    int16_t data[370];
};

// Define the audio playback thread
#define STACK_SIZE 4096
#define PRIORITY -1
K_THREAD_STACK_DEFINE(stack, STACK_SIZE);
struct k_thread audio_thread;
static bool prepare_transfer(const struct device *i2s_dev_tx)
{
    int ret;

    /* Allocate the memory blocks (tx buffer) from the slab and
       set everything to 0 */
    for(int i = 0; i < BLOCK_COUNT; i++){
        void *mem_block;
        ret = k_mem_slab_alloc(&mem_slab, &mem_block, K_NO_WAIT);
        if (ret < 0) {
            printk("Failed to allocate the memory blocks: %d\n", ret);
            return;
        }
        memset(mem_block, 0, BLOCK_SIZE);
        ret = i2s_write(i2s_dev_tx, mem_block, BLOCK_SIZE);
        if (ret < 0)
        {
            printk("Failed to write block %d: %d\n", i, ret);
            return false;
        }
    }

    return true;
}

double d1stream_real[370];
// default 1 d1stream size is header 8 bytes + data size 740 bytes = 748 bytes (T_frame = 0.01)
// default 1 d1stream size is header 8 bytes + data size 1480 bytes = 1488 bytes (T_frame = 0.02)
// playback the D1stream audio data
void play_audio_realtime(uint8_t * buf, int buf_len){
    struct d1stream_data_t raw_data;
    uint16_t *d1stream;
    d1stream = (uint16_t*)buf;
       
    // get the D1stream real part
    for(int i = 0;i < (buf_len-8)/4; i++){
        d1stream_real[i] = (double)(d1stream[d1stream_header+ 2 * i]);
    }

    //(FirstFilterSignal, SignalArray_Previous, FirstOutputSignalArray_Previous)
    HandMadeIIRFilter_LPF(IIRFilter_Trcat_LPF_B, IIRFilter_Trcat_LPF_A, IIRFilter_FilterOrder_LPF, IIRFilter_FilterOrder_HPF,
                           FirstFilterSignal , d1stream_real, SignalArray_Previous, FirstOutputSignalArray_Previous);
   
    //(FilterSignal, SecondOutputSignalArray_Previous)
    HandMadeIIRFilter_HPF(IIRFilter_Trcat_HPF_B, IIRFilter_Trcat_HPF_A, IIRFilter_FilterOrder_HPF, SecondOutputArray,
                            FirstFilterSignal, SecondOutputSignalArray_Previous);
    // use the SecondOutputArray to play the i2s audio(370 * 2 bytes)  
    for(int j = 0; j < 370; j++){
        //raw_data.data[j] = ((int16_t)(d1stream_real[j]*10));
        raw_data.data[j] = ((int16_t)(SecondOutputArray[j]*100));
    }
    //LOG_INF(" before raw_data.data[0] = %d\n", raw_data.data[0]);

    // put the d1stream data to fifo
    k_fifo_put(&fifo_d1stream_data, &raw_data);
}

void play_doppler_audio(){
    int ret = 0;
    struct d1stream_data_t *raw_data;
    struct d1stream_data_t *second_raw_data;
    int count = 0;

infinite_loop:
    for( int i = 0 ; i < 20 ; i++){
        raw_data = k_fifo_get(&fifo_d1stream_data, K_FOREVER);
        LOG_INF("First k_fifo_get (%d)= %d\n", i, raw_data->data[0]);
        if (raw_data == NULL) {
            printk("Failed to get raw_data from fifo\n");
        }
        ret = i2s_buf_write(i2s_dev_tx, raw_data->data, sizeof(int16_t)*SAMPLES_PER_BLOCK);
        if (ret < 0) {
            LOG_ERR("i2s_write %d, %d \n",ret, __LINE__);
        }
    }
    ret = i2s_trigger(i2s_dev_tx, I2S_DIR_TX, I2S_TRIGGER_START);
    if (ret < 0) {
        LOG_ERR("i2s_trigger %d, %d \n",ret, __LINE__);
    }
    for(;;){
        second_raw_data = k_fifo_get(&fifo_d1stream_data, K_FOREVER);
        LOG_INF("Second k_fifo_get (0)= %d\n", second_raw_data->data[0]);

        ret = i2s_buf_write(i2s_dev_tx, second_raw_data->data, sizeof(int16_t)*SAMPLES_PER_BLOCK);
        if (ret < 0) {
            LOG_ERR("i2s_buf_write %d, %d \n",ret, __LINE__);
            ret = trigger_command(i2s_dev_tx, I2S_TRIGGER_PREPARE);
            if(ret < 0){
                LOG_ERR("trigger_command %d, %d \n",ret, __LINE__);
            }
            //ret = i2s_trigger(i2s_dev_tx, I2S_DIR_TX, I2S_TRIGGER_START);
            //if(ret <0){
            //  LOG_ERR("i2s_trigger %d, %d \n",ret, __LINE__);
            //}
            goto infinite_loop;
        }
       
    }
}

// Start the audio playback thread
void audio_play_thread(){
   
    k_tid_t audio_tid = k_thread_create(&audio_thread, stack,
                                 K_THREAD_STACK_SIZEOF(stack),
                                 play_doppler_audio,
                                 NULL, NULL, NULL,
                                 PRIORITY, 0, K_NO_WAIT);
    k_thread_name_set(audio_tid,"audio_output");
    k_thread_start(audio_tid);
}

// Initial the audio device and setup the i2s configuration
void audio_init(){
    bool err;
    if (!device_is_ready(i2s_dev_tx)) {
        printk("%s is not ready\n", i2s_dev_tx->name);
        return 0;
    }

    /* Configure the I2S device */
    struct i2s_config i2s_cfg;
    i2s_cfg.word_size = 16; // due to int16_t in data_frame declaration
    i2s_cfg.channels = 1; // L + R channel
    i2s_cfg.format = I2S_FMT_DATA_FORMAT_I2S;
    i2s_cfg.options = I2S_OPT_BIT_CLK_MASTER | I2S_OPT_FRAME_CLK_MASTER;
    i2s_cfg.frame_clk_freq = SAMPLE_FREQUENCY;
    i2s_cfg.mem_slab = &mem_slab;
    i2s_cfg.block_size = BLOCK_SIZE;
    i2s_cfg.timeout = 1000;
    int ret = i2s_configure(i2s_dev_tx, I2S_DIR_TX, &i2s_cfg);
    if (ret < 0) {
        printk("Failed to configure the I2S stream: (%d)\n", ret);
        return ret;
    }
    LOG_INF("Audio_init start !!");

    err = prepare_transfer(i2s_dev_tx);
    switch_volume_level(0);
    play_tone_arbitrary_time(1000, 1.0);
    //switch_volume_level(1);
    //hmb_set_led_off(HMB_LED1);
    audio_play_thread();
}
  
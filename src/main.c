#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
// #include <zephyr/logging/log.h>
#include <zephyr/pm/device.h>
#include <stdlib.h> // for abs

// LOG_MODULE_REGISTER(main);

#define MAX30102_I2C_ADDR 0x57
#define MAX30205_I2C_ADDR 0x48

#define NRF_I2C_SLAVE_ADDR 0x42

#define REG_FIFO_DATA     0x07
#define REG_FIFO_CONFIG   0x08
#define REG_MODE_CONFIG   0x09
#define REG_SPO2_CONFIG   0x0A
#define REG_LED1_PA       0x0C
#define REG_LED2_PA       0x0D

#define REG_TEMP_MSB   0x00
#define REG_TEMP_LSB   0x01

struct sensor_data {
    uint32_t red;
    uint32_t ir;
    int32_t temp;
    uint32_t bpm;
    uint8_t spo2; // SpO2 in percent
} current_data = {0, 0, 0, 0, 0};

#define SLAVE_REG_RED_0    0x00
#define SLAVE_REG_RED_1    0x01
#define SLAVE_REG_RED_2    0x02
#define SLAVE_REG_IR_0     0x03
#define SLAVE_REG_IR_1     0x04
#define SLAVE_REG_IR_2     0x05
#define SLAVE_REG_TEMP_0   0x06
#define SLAVE_REG_TEMP_1   0x07
#define SLAVE_REG_TEMP_2   0x08

#define SLAVE_REG_SPO2     0x0A
#define SLAVE_REG_TEMP_3   0x09

static uint8_t slave_registers[11] = {0};
static uint8_t slave_reg_idx = 0;
static uint8_t dma_rx_buf[32];
static bool reg_idx_set = false;

static const struct device *i2c_master;
static const struct device *i2c_slave;

#define MA_WINDOW 8
#define SAMPLE_WINDOW_MS 1000
#define SAMPLE_COUNT 500

int32_t offset = 0;

static bool pi_read_happened = false;

// I2C Target (slave) callbacks
static int on_write_requested(struct i2c_target_config *config) {
    reg_idx_set = false;
    slave_reg_idx = 0;
    return 0;
}

static int on_write_received(struct i2c_target_config *config, uint8_t val) {
    // The first byte written by the master is the register index
    if (!reg_idx_set)
    {
        slave_reg_idx = val;
        reg_idx_set = true;
    }
    else
    {
        if(slave_reg_idx < sizeof(slave_registers))
        {
            slave_registers[slave_reg_idx++] = val;
        }
    }
    return 0;
}

static void on_buf_write_received(struct i2c_target_config *config, 
                                 uint8_t *buf, uint32_t len)
{
    buf = dma_rx_buf;
    len = sizeof(dma_rx_buf);
    return;
}

static int on_read_requested(struct i2c_target_config *config, uint8_t *val) {
    pi_read_happened = true;
    if (slave_reg_idx < sizeof(slave_registers)) {
        *val = slave_registers[slave_reg_idx++];
    } else {
        *val = 0xFF;
    }
    return 0;
}

static int on_read_processed(struct i2c_target_config *config, uint8_t *val) {
    pi_read_happened = true;
    if (slave_reg_idx < sizeof(slave_registers)) {
        *val = slave_registers[slave_reg_idx++];
    } else {
        *val = 0xFF;
    }
    return 0;
}

static int on_buf_read_requested(struct i2c_target_config *config, 
                                 uint8_t **buf, uint32_t *len)
{
    if (slave_reg_idx < sizeof(slave_registers))
    {
        *buf = &slave_registers[slave_reg_idx];
        *len = sizeof(slave_registers) - slave_reg_idx;
    }
    else{
        static uint8_t dummy = 0xFF;
        *buf = &dummy;
        *len = 1;
    }
    pi_read_happened = true;

    return 0;
}

static int on_stop(struct i2c_target_config *config) {
    if (reg_idx_set && dma_rx_buf[0] < sizeof(slave_registers))
    {
    }
    reg_idx_set = false;
    return 0;
}

static const struct i2c_target_callbacks slave_callbacks = {
    .write_requested = on_write_requested,
    .read_requested = on_read_requested,
    .write_received = on_write_received,
    .read_processed = on_read_processed,
    .buf_write_received = on_buf_write_received,
    .buf_read_requested = on_buf_read_requested,
    .stop = on_stop,
};

static struct i2c_target_config slave_config = {
    .address = NRF_I2C_SLAVE_ADDR,
    .callbacks = &slave_callbacks,
};

static int max30102_init(void)
{
    uint8_t fifo_config[2] = {REG_FIFO_CONFIG, 0x10}; // SMP_AVE=1 (no avg), ROLLOVER_EN=1 → 100Hz output
    if (i2c_write(i2c_master, fifo_config, 2, MAX30102_I2C_ADDR) != 0) return -1;
    uint8_t mode_config[2] = {REG_MODE_CONFIG, 0x03};
    if (i2c_write(i2c_master, mode_config, 2, MAX30102_I2C_ADDR) != 0) return -1;
    uint8_t spo2_config[2] = {REG_SPO2_CONFIG, 0x27};
    if (i2c_write(i2c_master, spo2_config, 2, MAX30102_I2C_ADDR) != 0) return -1;
    uint8_t led1[2] = {REG_LED1_PA, 0x24};
    uint8_t led2[2] = {REG_LED2_PA, 0x24};
    if (i2c_write(i2c_master, led1, 2, MAX30102_I2C_ADDR) != 0 ||
        i2c_write(i2c_master, led2, 2, MAX30102_I2C_ADDR) != 0) return -1;
    return 0;
}

static int max30102_read_fifo(uint32_t *red, uint32_t *ir)
{
    uint8_t fifo_data[6];
    uint8_t reg = REG_FIFO_DATA;
    if (i2c_write_read(i2c_master, MAX30102_I2C_ADDR, &reg, 1, fifo_data, sizeof(fifo_data)) != 0) return -1;
    *red = ((uint32_t)fifo_data[0] << 16) | ((uint32_t)fifo_data[1] << 8) | (uint32_t)fifo_data[2];
    *ir  = ((uint32_t)fifo_data[3] << 16) | ((uint32_t)fifo_data[4] << 8) | (uint32_t)fifo_data[5];
    // printk("RED == %d\n", *red);
    // printk("IR == %d\n", *ir);
    return 0;
}

static int max30205_read_temp(int32_t *temp)
{
    uint8_t buf[2];
    // Read two bytes starting at REG_TEMP_MSB
    if (i2c_burst_read(i2c_master, MAX30205_I2C_ADDR, REG_TEMP_MSB, buf, 2) != 0) return -1;
    int16_t raw = (buf[0] << 8) | buf[1];
    *temp = (int32_t)raw;
    return 0;
}

static void update_slave_registers(void)
{
    slave_registers[SLAVE_REG_RED_0] = (current_data.bpm >> 16) & 0xFF;
    slave_registers[SLAVE_REG_RED_1] = (current_data.bpm >> 8) & 0xFF;
    slave_registers[SLAVE_REG_RED_2] = current_data.bpm & 0xFF;
    slave_registers[SLAVE_REG_IR_0] = (current_data.ir >> 16) & 0xFF;
    slave_registers[SLAVE_REG_IR_1] = (current_data.ir >> 8) & 0xFF;
    slave_registers[SLAVE_REG_IR_2] = current_data.ir & 0xFF;
    slave_registers[SLAVE_REG_TEMP_0] = (current_data.temp >> 24) & 0xFF;
    slave_registers[SLAVE_REG_TEMP_1] = (current_data.temp >> 16) & 0xFF;
    slave_registers[SLAVE_REG_TEMP_2] = (current_data.temp >> 8) & 0xFF;
    slave_registers[SLAVE_REG_TEMP_3] = current_data.temp & 0xFF;
    slave_registers[SLAVE_REG_SPO2] = current_data.spo2;
}

#define MAX_BPM_SAMPLES 16

void sort_uint32(uint32_t *arr, int n) {
    for (int i = 1; i < n; i++) {
        uint32_t key = arr[i];
        int j = i - 1;
        while (j >= 0 && arr[j] > key) {
            arr[j + 1] = arr[j];
            j--;
        }
        arr[j + 1] = key;
    }
}


void collect_and_process_samples(void) {
    int bpm_samples = 0;
    uint32_t bpm_values[MAX_BPM_SAMPLES];

    int first = 1;

    // For SpO2 calculation
    int32_t red_dc = 0, ir_dc = 0;
    int32_t red_ac_sum = 0, ir_ac_sum = 0;
    int ac_samples = 0;
    int32_t red_baseline = 0, ir_baseline = 0;
    bool finger_detected = false;
    int last_sample = -1;

    
    for (int sample = 0; sample < SAMPLE_COUNT; sample++) {
        max30102_init();
        max30102_read_fifo(&current_data.red, &current_data.ir);

        // DC baseline (slow moving average)
        red_baseline = (red_baseline * 15 + current_data.red) / 16;
        ir_baseline = (ir_baseline * 15 + current_data.ir) / 16;
        int32_t red_ac = (int32_t)current_data.red - red_baseline;
        int32_t ir_ac = (int32_t)current_data.ir - ir_baseline;

        red_dc += red_baseline;
        ir_dc += ir_baseline;
        red_ac_sum += abs(red_ac);
        ir_ac_sum += abs(ir_ac);
        ac_samples++;

        // Peak detection (as before)
        static bool was_above = false;
        // printk("Offset == %d\n", offset);
        uint32_t threshold = 0xE377; // MAGIC NUMBER BOIIIII!

        // printk("Red == %d\n", current_data.red);

        if (current_data.red > threshold && !was_above) {
            // printk("HERE\n");
            was_above = true;
            if (last_sample != -1) {
                // printk("HERE2\n");
                uint32_t dt = (sample - last_sample);
                // printk("dt == %d\n", dt);
                if (dt < 50 && dt > 20) {
                    // printk("HERE3\n");
                    uint32_t bpm = 60000 / (dt*20);
                    if (bpm_samples < MAX_BPM_SAMPLES) {
                        bpm_values[bpm_samples++] = bpm;
                    }
                    last_sample = sample;
                }
                if (dt > 50)
                {
                    last_sample = sample;
                }
            }
            if (first == 1)
            {
                first = 0;
                last_sample = sample;
            }
        } else if (current_data.red < threshold) {
            was_above = false;
        }

        if (current_data.red > 0xc000)
        {
            finger_detected = true;
        }
        else
        {
            finger_detected = false;
        }        

        k_busy_wait(20000);
        // k_sleep(K_MSEC(20)); // 50 Hz
        // now = k_uptime_get_32();
    }

    // Median BPM
    if (bpm_samples > 0 && finger_detected) {
        sort_uint32(bpm_values, bpm_samples);
        if (bpm_samples % 2 == 1) {
            current_data.bpm = bpm_values[bpm_samples / 2];
        } else {
            current_data.bpm = (bpm_values[bpm_samples / 2 - 1] + bpm_values[bpm_samples / 2]) / 2;
        }
        // printk("BPM == %d\n", current_data.bpm);
    } 
    else if (!finger_detected)
    {
        current_data.bpm = 0;
    }

    // SpO2 calculation (simple ratio-of-ratios method)
    if (ac_samples > 0 && finger_detected) {
        float red_ac_avg = (float)red_ac_sum / ac_samples;
        float ir_ac_avg = (float)ir_ac_sum / ac_samples;
        float red_dc_avg = (float)red_dc / ac_samples;
        float ir_dc_avg = (float)ir_dc / ac_samples;
        float ratio = (red_ac_avg / red_dc_avg) / (ir_ac_avg / ir_dc_avg);
        (void)ir_dc_avg;
        // Empirical formula for MAX30102 (approximate):
        float spo2 = 110.0f - 20.0f * ratio;
        if (spo2 > 100.0f) spo2 = 100.0f;
        if (spo2 < 0.0f) spo2 = 0.0f;
        current_data.spo2 = (uint8_t)spo2;
    } else if (!finger_detected) {
        current_data.spo2 = 0;
    }


    max30205_read_temp(&current_data.temp);
    update_slave_registers();
}

void slave_mode(void)
{
    pi_read_happened = false;

    
    int ret = pm_device_action_run(i2c_master, PM_DEVICE_ACTION_SUSPEND);
    if (ret && ret != -ENOSYS)
    {
    }
    
    ret = i2c_target_register(i2c_slave, &slave_config);
    if (ret) {
        return;
    }

    // printk("SLAVE START\n");
    while (!pi_read_happened) {
        k_busy_wait(10000);
    }

    k_busy_wait(100000); // Pi can read during this window
    i2c_target_unregister(i2c_slave, &slave_config);
    // printk("SLAVE STOP\n");

    ret = pm_device_action_run(i2c_master, PM_DEVICE_ACTION_RESUME);
    if (ret && ret != -ENOSYS)
    {
    }
}

int main(void)
{
    i2c_master = DEVICE_DT_GET(DT_NODELABEL(i2c0));
    i2c_slave = DEVICE_DT_GET(DT_NODELABEL(i2c1));
    if (!device_is_ready(i2c_master) || !device_is_ready(i2c_slave)) {
        return -1;
    }
    
    // max30102_init();

    while (1) {
        collect_and_process_samples();
        slave_mode();     // Slave mode: let Pi read the data
    }
    return 0;
}
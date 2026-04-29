// Zephyr kernel and device headers
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
// #include <zephyr/logging/log.h>
#include <zephyr/pm/device.h>
#include <stdlib.h> // for abs()

// LOG_MODULE_REGISTER(main);


// I2C addresses for sensors
#define MAX30102_I2C_ADDR 0x57   // MAX30102 pulse oximeter
#define MAX30205_I2C_ADDR 0x48   // MAX30205 temperature sensor

#define NRF_I2C_SLAVE_ADDR 0x42  // This device's I2C slave address

// MAX30102 register addresses
#define REG_FIFO_DATA     0x07
#define REG_FIFO_CONFIG   0x08
#define REG_MODE_CONFIG   0x09
#define REG_SPO2_CONFIG   0x0A
#define REG_LED1_PA       0x0C
#define REG_LED2_PA       0x0D

// MAX30205 register addresses
#define REG_TEMP_MSB   0x00
#define REG_TEMP_LSB   0x01


// Structure to hold sensor readings and calculated values
struct sensor_data {
    uint32_t red;      // Red LED value from MAX30102
    uint32_t ir;       // IR LED value from MAX30102
    int32_t temp;      // Temperature from MAX30205 (raw)
    uint32_t bpm;      // Calculated beats per minute
    uint8_t spo2;      // Calculated SpO2 percentage
} current_data = {0, 0, 0, 0, 0};


// Register map for I2C slave interface (to be read by master, e.g., Raspberry Pi)
#define SLAVE_REG_RED_0    0x00 // BPM high byte
#define SLAVE_REG_RED_1    0x01 // BPM mid byte
#define SLAVE_REG_RED_2    0x02 // BPM low byte
#define SLAVE_REG_IR_0     0x03 // IR high byte
#define SLAVE_REG_IR_1     0x04 // IR mid byte
#define SLAVE_REG_IR_2     0x05 // IR low byte
#define SLAVE_REG_TEMP_0   0x06 // Temp MSB
#define SLAVE_REG_TEMP_1   0x07 // Temp 2nd byte
#define SLAVE_REG_TEMP_2   0x08 // Temp 3rd byte
#define SLAVE_REG_TEMP_3   0x09 // Temp LSB
#define SLAVE_REG_SPO2     0x0A // SpO2 value


// Buffer for slave register values
static uint8_t slave_registers[11] = {0};
static uint8_t slave_reg_idx = 0;      // Current register index for I2C slave
static uint8_t dma_rx_buf[32];         // DMA buffer for I2C
static bool reg_idx_set = false;       // Flag to indicate if register index is set

// I2C device pointers
static const struct device *i2c_master; // I2C master (to sensors)
static const struct device *i2c_slave;  // I2C slave (to Pi)

#define MA_WINDOW 8            // Moving average window (unused)
#define SAMPLE_WINDOW_MS 1000  // Sample window in ms (unused)
#define SAMPLE_COUNT 500       // Number of samples to collect per cycle

int32_t offset = 0;           // Offset for calibration (unused)

static bool pi_read_happened = false; // Flag to indicate if Pi has read data


// I2C Target (slave) callback: called when master initiates a write
static int on_write_requested(struct i2c_target_config *config) {
    reg_idx_set = false;      // Reset register index flag
    slave_reg_idx = 0;       // Reset register index
    return 0;
}

// I2C Target (slave) callback: called when master writes a byte
static int on_write_received(struct i2c_target_config *config, uint8_t val) {
    // The first byte is the register index, subsequent bytes are data
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

// I2C Target (slave) callback: called when master writes a buffer
static void on_buf_write_received(struct i2c_target_config *config, 
                                 uint8_t *buf, uint32_t len)
{
    // Not used: just set buffer pointer and length to DMA buffer
    buf = dma_rx_buf;
    len = sizeof(dma_rx_buf);
    return;
}

// I2C Target (slave) callback: called when master requests a read
static int on_read_requested(struct i2c_target_config *config, uint8_t *val) {
    pi_read_happened = true; // Mark that Pi has read
    if (slave_reg_idx < sizeof(slave_registers)) {
        *val = slave_registers[slave_reg_idx++]; // Return register value
    } else {
        *val = 0xFF; // Out of range: return dummy value
    }
    return 0;
}

// I2C Target (slave) callback: called after a read is processed
static int on_read_processed(struct i2c_target_config *config, uint8_t *val) {
    pi_read_happened = true;
    if (slave_reg_idx < sizeof(slave_registers)) {
        *val = slave_registers[slave_reg_idx++];
    } else {
        *val = 0xFF;
    }
    return 0;
}

// I2C Target (slave) callback: called when master requests a buffer read
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

// I2C Target (slave) callback: called when STOP condition is received
static int on_stop(struct i2c_target_config *config) {
    // No action needed, just reset register index flag
    reg_idx_set = false;
    return 0;
}


// I2C slave callback structure
static const struct i2c_target_callbacks slave_callbacks = {
    .write_requested = on_write_requested,
    .read_requested = on_read_requested,
    .write_received = on_write_received,
    .read_processed = on_read_processed,
    .buf_write_received = on_buf_write_received,
    .buf_read_requested = on_buf_read_requested,
    .stop = on_stop,
};

// I2C slave configuration
static struct i2c_target_config slave_config = {
    .address = NRF_I2C_SLAVE_ADDR,
    .callbacks = &slave_callbacks,
};


// Initialize MAX30102 sensor with default configuration
static int max30102_init(void)
{
    uint8_t fifo_config[2] = {REG_FIFO_CONFIG, 0x10}; // SMP_AVE=1 (no avg), ROLLOVER_EN=1 → 100Hz output
    if (i2c_write(i2c_master, fifo_config, 2, MAX30102_I2C_ADDR) != 0) return -1;
    uint8_t mode_config[2] = {REG_MODE_CONFIG, 0x03}; // SpO2 mode
    if (i2c_write(i2c_master, mode_config, 2, MAX30102_I2C_ADDR) != 0) return -1;
    uint8_t spo2_config[2] = {REG_SPO2_CONFIG, 0x27}; // 16-bit, 100Hz
    if (i2c_write(i2c_master, spo2_config, 2, MAX30102_I2C_ADDR) != 0) return -1;
    uint8_t led1[2] = {REG_LED1_PA, 0x24};            // LED1 pulse amplitude
    uint8_t led2[2] = {REG_LED2_PA, 0x24};            // LED2 pulse amplitude
    if (i2c_write(i2c_master, led1, 2, MAX30102_I2C_ADDR) != 0 ||
        i2c_write(i2c_master, led2, 2, MAX30102_I2C_ADDR) != 0) return -1;
    return 0;
}


// Read FIFO data from MAX30102 (red and IR values)
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


// Read temperature from MAX30205 sensor
static int max30205_read_temp(int32_t *temp)
{
    uint8_t buf[2];
    // Read two bytes starting at REG_TEMP_MSB
    if (i2c_burst_read(i2c_master, MAX30205_I2C_ADDR, REG_TEMP_MSB, buf, 2) != 0) return -1;
    int16_t raw = (buf[0] << 8) | buf[1];
    *temp = (int32_t)raw;
    return 0;
}


// Update the I2C slave register buffer with the latest sensor data
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


#define MAX_BPM_SAMPLES 16 // Maximum number of BPM samples for median filtering

// Simple insertion sort for median calculation
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



// Collect samples from MAX30102, process for BPM and SpO2, and update slave registers
void collect_and_process_samples(void) {
    int bpm_samples = 0;                    // Number of BPM samples collected
    uint32_t bpm_values[MAX_BPM_SAMPLES];   // Array to hold BPM values for median filtering

    int first = 1;                          // Flag for first peak detection

    // Variables for SpO2 calculation
    int32_t red_dc = 0, ir_dc = 0;          // DC components (sum)
    int32_t red_ac_sum = 0, ir_ac_sum = 0;  // AC components (sum of abs)
    int ac_samples = 0;                     // Number of AC samples
    int32_t red_baseline = 0, ir_baseline = 0; // Baseline for DC removal
    bool finger_detected = false;           // True if finger is detected
    int last_sample = -1;                   // Last sample index for peak detection

    for (int sample = 0; sample < SAMPLE_COUNT; sample++) {
        max30102_init(); // Re-initialize sensor (may be redundant)
        max30102_read_fifo(&current_data.red, &current_data.ir); // Read red/IR values

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

        // Peak detection for BPM calculation
        static bool was_above = false;
        uint32_t threshold = 0xE377; // Empirically chosen threshold for peak

        if (current_data.red > threshold && !was_above) {
            was_above = true;
            if (last_sample != -1) {
                uint32_t dt = (sample - last_sample); // Time between peaks (samples)
                if (dt < 50 && dt > 20) {
                    // Calculate BPM: 60000 ms/min divided by (dt * 20 ms/sample)
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

        // Finger detection: if red value is high enough
        if (current_data.red > 0xc000)
        {
            finger_detected = true;
        }
        else
        {
            finger_detected = false;
        }        

        k_busy_wait(20000); // Wait 20 ms (50 Hz sampling)
    }

    // Median BPM calculation
    if (bpm_samples > 0 && finger_detected) {
        sort_uint32(bpm_values, bpm_samples);
        if (bpm_samples % 2 == 1) {
            current_data.bpm = bpm_values[bpm_samples / 2];
        } else {
            current_data.bpm = (bpm_values[bpm_samples / 2 - 1] + bpm_values[bpm_samples / 2]) / 2;
        }
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

    // Read temperature and update slave registers
    max30205_read_temp(&current_data.temp);
    update_slave_registers();
}


// Enter I2C slave mode: allow Pi to read data
void slave_mode(void)
{
    pi_read_happened = false; // Reset Pi read flag

    // Suspend I2C master to avoid bus contention
    int ret = pm_device_action_run(i2c_master, PM_DEVICE_ACTION_SUSPEND);
    if (ret && ret != -ENOSYS)
    {
        // Handle error if needed
    }
    
    // Register as I2C slave
    ret = i2c_target_register(i2c_slave, &slave_config);
    if (ret) {
        return;
    }

    // Wait for Pi to read data
    while (!pi_read_happened) {
        k_busy_wait(10000); // Wait 10 ms
    }

    k_busy_wait(100000); // Allow Pi to finish reading (100 ms)
    i2c_target_unregister(i2c_slave, &slave_config); // Unregister slave

    // Resume I2C master
    ret = pm_device_action_run(i2c_master, PM_DEVICE_ACTION_RESUME);
    if (ret && ret != -ENOSYS)
    {
        // Handle error if needed
    }
}


// Main entry point
int main(void)
{
    // Get I2C device handles for master (to sensors) and slave (to Pi)
    i2c_master = DEVICE_DT_GET(DT_NODELABEL(i2c0));
    i2c_slave = DEVICE_DT_GET(DT_NODELABEL(i2c1));
    if (!device_is_ready(i2c_master) || !device_is_ready(i2c_slave)) {
        return -1; // Abort if devices are not ready
    }
    
    // Main loop: collect/process samples, then allow Pi to read
    while (1) {
        collect_and_process_samples(); // Gather sensor data and process
        slave_mode();                  // Enter slave mode for Pi to read
    }
    return 0;
}
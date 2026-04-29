#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/device.h>

LOG_MODULE_REGISTER(main);

#define NRF_I2C_SLAVE_ADDR 0x42

static uint8_t slave_registers[10] = {0};
static uint8_t slave_reg_idx = 0;
static bool reg_idx_set = false;
static const struct device *i2c_master;
static const struct device *i2c_slave;

static uint8_t dma_rx_buf[32];

static void on_buf_write_received(struct i2c_target_config *config, 
                                 uint8_t *buf, uint32_t len)
{
    buf = dma_rx_buf;
    len = sizeof(dma_rx_buf);
    return;
}

static int on_write_requested(struct i2c_target_config *config) {
    reg_idx_set = false;
    slave_reg_idx = 0;
    return 0;
}
static int on_write_received(struct i2c_target_config *config, uint8_t val) {
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
static int on_read_requested(struct i2c_target_config *config, uint8_t *val) {
    if (slave_reg_idx < sizeof(slave_registers)) {
        *val = slave_registers[slave_reg_idx++];
    } else {
        *val = 0xFF;
    }
    return 0;
}
static int on_read_processed(struct i2c_target_config *config, uint8_t *val) {
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

    return 0;
}

static int on_stop(struct i2c_target_config *config) {
    if (reg_idx_set && dma_rx_buf[0] < sizeof(slave_registers))
    {
        uint8_t reg = dma_rx_buf[0];
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

int main(void)
{
    i2c_master = DEVICE_DT_GET(DT_NODELABEL(i2c0));
    i2c_slave = DEVICE_DT_GET(DT_NODELABEL(i2c1));
    if (!device_is_ready(i2c_slave)) {
        printk("I2C slave not ready\n");
        return -1;
    }

    // Suspend TWIM so it releases the shared pins for TWIS
    int ret = pm_device_action_run(i2c_master, PM_DEVICE_ACTION_SUSPEND);
    if (ret && ret != -ENOSYS) {
        printk("Failed to suspend master: %d\n", ret);
    }

    ret = i2c_target_register(i2c_slave, &slave_config);
    if (ret) {
        printk("Failed to register as I2C slave: %d\n", ret);
        return -1;
    }
    printk("nRF is now a permanent I2C slave at address 0x%X\n", NRF_I2C_SLAVE_ADDR);
    while (1) {
        k_sleep(K_SECONDS(1));
        printk("alive\n");
    }
}
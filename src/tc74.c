#include "tc74.h"
#include <zephyr/sys/printk.h>

/* O nó no devicetree (veja o overlay) chama-se “tc74sensor” */
static const struct i2c_dt_spec dev = I2C_DT_SPEC_GET(DT_NODELABEL(tc74sensor));

int tc74_init(void)
{
    if (!device_is_ready(dev.bus)) {
        printk("TC74: I2C bus %s not ready\n", dev.bus->name);
        return -ENODEV;
    }
    /* Opcionalmente, podes tentar o write, mas ignorar o erro: */
    uint8_t cmd = TC74_CMD_RTR;
    int ret = i2c_write_dt(&dev, &cmd, 1);
    if (ret) {
        // printk("TC74: write cmd failed (%d) — a ignorar\n", ret);
    }
    /* devolve sempre sucesso para prosseguir à leitura */
    return 0;
}


int tc74_read(uint8_t *out_temp)
{
    int ret = i2c_read_dt(&dev, out_temp, 1);
    if (ret) {
        printk("TC74: read failed (%d)\n", ret);
    }
    return ret;
}

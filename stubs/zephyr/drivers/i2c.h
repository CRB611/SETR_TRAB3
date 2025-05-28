/** 
 * @file zephyr/drivers/i2c.h
 * @brief Stub minimal para compilar em host com UNIT_TEST
 */

#ifndef ZEPHYR_DRIVERS_I2C_H
#define ZEPHYR_DRIVERS_I2C_H

/* Simulação mínima de struct device */
struct device { 
    const char *name;
};

/* Protótipos vazios das funções I2C que possam ser usadas */
static inline int i2c_read(const struct device *dev, uint8_t *buf, uint32_t num_bytes, uint16_t addr) {
    (void)dev; (void)buf; (void)num_bytes; (void)addr;
    return 0;
}

static inline int i2c_write(const struct device *dev, const uint8_t *buf, uint32_t num_bytes, uint16_t addr) {
    (void)dev; (void)buf; (void)num_bytes; (void)addr;
    return 0;
}

#endif /* ZEPHYR_DRIVERS_I2C_H */

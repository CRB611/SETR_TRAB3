#ifndef ZEPHYR_DEVICE_H
#define ZEPHYR_DEVICE_H
#include <stdbool.h>

/* stub do struct device */
struct device {
    const char *name;
};

/* variável de controlo no stub */
extern bool fake_dev_ready;

/* device_is_ready() usa a flag fake_dev_ready */
static inline bool device_is_ready(const struct device *dev) {
    (void)dev;
    return fake_dev_ready;
}

#endif /* ZEPHYR_DEVICE_H */

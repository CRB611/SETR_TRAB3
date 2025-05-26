#ifndef ZEPHYR_DEVICETREE_H
#define ZEPHYR_DEVICETREE_H

/* permitimos usar qualquer token como nó */
#define DT_NODELABEL(x) x

/* DEVICE_DT_GET devolve sempre o mesmo fake_device */
extern struct device fake_device;
#define DEVICE_DT_GET(node) (&fake_device)

#endif /* ZEPHYR_DEVICETREE_H */

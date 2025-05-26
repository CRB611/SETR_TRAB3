#ifndef ZEPHYR_SYS_PRINTK_H
#define ZEPHYR_SYS_PRINTK_H

/* se usares apenas printf no host, podes redirecionar printk para printf */
#include <stdio.h>
#define printk    printf

#endif /* ZEPHYR_SYS_PRINTK_H */

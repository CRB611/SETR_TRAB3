#ifndef ZEPHYR_KERNEL_H
#define ZEPHYR_KERNEL_H

#include <stdint.h>

/* 
 * Define um tipo opaco e as constantes/funções mínimas de k_mutex 
 */
typedef struct k_mutex {
    /* nada dentro */
} k_mutex;

/* timeouts */
#define K_FOREVER  (-1)

/* stubs vazios */
static inline void k_mutex_init(k_mutex *m)       { (void)m; }
static inline void k_mutex_lock(k_mutex *m, int32_t timeout) { (void)m; (void)timeout; }
static inline void k_mutex_unlock(k_mutex *m)     { (void)m; }

#endif /* ZEPHYR_KERNEL_H */

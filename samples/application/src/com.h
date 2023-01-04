
#include <zephyr/kernel.h>

#define MAX_RECV_BUF_LEN 2048

#define DATE_TIME_LEN 27

// Prototypes
void thread_com(struct k_lifo *data, enum conn_state *conn_state, struct k_mutex *mtx);
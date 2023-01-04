
#include <zephyr/kernel.h>

// devicetree
#define LED1_R_NODE DT_ALIAS(led0)
#define LED1_G_NODE DT_ALIAS(led1)
#define LED1_B_NODE DT_ALIAS(led2)

#define PERIOD_USEC	(USEC_PER_SEC / 50U)
#define PERIOD_NSEC	(NSEC_PER_USEC * PERIOD_USEC)

#define STEPSIZE_USEC	2000

enum { RED, GREEN, BLUE };

// Prototypes
void thread_led(enum conn_state *conn_state, void *arg2, void *arg3);

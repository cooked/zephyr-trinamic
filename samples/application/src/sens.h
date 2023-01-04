
//#define THREAD_RATE_S 1
#define THREAD_RATE_10S 	10U
#define THREAD_RATE_30S 	30U
#define THREAD_RATE_1MIN 	60U
#define THREAD_RATE_2MIN 	120U
#define THREAD_RATE_5MIN 	300U
#define THREAD_RATE_10MIN 	600U

#define THREAD_RATE_MS 		THREAD_RATE_10S

// Prototypes
void thread_sens(struct k_lifo *data, enum conn_state *conn_state,
					struct k_mutex *mtx);
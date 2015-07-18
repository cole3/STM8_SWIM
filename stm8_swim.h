#ifndef _STM8_SWIM_
#define _STM8_SWIM_

typedef enum {
    LOW,
    HIGH
} swim_level;

typedef enum {
    SWIM_OK,
    SWIM_FAIL,
    SWIM_TIMEOUT
} swim_ret;

typedef enum {
    SWIM,
    RST
} io_name;

typedef struct {
    void (*ndelay)(unsigned int ns);
    void (*io_set)(io_name io, int level);
    int (*io_get)(io_name io);
    void *private;
} swim_input;

typedef struct {
    swim_ret (*entry)(void);
    swim_ret (*reset)(void);
    swim_ret (*read)(unsigned int addr, unsigned char *buf, unsigned int size);
    swim_ret (*write)(unsigned int addr, unsigned char *buf, unsigned int size);
    // callback...
} swim_method;

extern unsigned int return_line;

#endif


#ifndef MSP432_BOOSTXL_INIT_H_
#define MSP432_BOOSTXL_INIT_H_

#include <ti/devices/msp432p4xx/driverlib/driverlib.h>


typedef enum {
    FS_8000_HZ,
    FS_11025_HZ,
    FS_16000_HZ,
    FS_22050_HZ,
    FS_24000_HZ,
    FS_32000_HZ,
    FS_44100_HZ,
    FS_48000_HZ
} FS_enum_t;

typedef enum {
    BOOSTXL_MIC_IN,
    BOOSTXL_J1_2_IN
} BOOSTXL_IN_enum_t;

typedef enum {
    BUFLEN_8,
    BUFLEN_16,
    BUFLEN_32,
    BUFLEN_64,
    BUFLEN_128
} BUFLEN_enum_t;

typedef uint16_t (*msp432_sample_process_t)(uint16_t);
typedef void     (*msp432_buffer_process_t)(uint16_t *, uint16_t *);

void msp432_boostxl_init();

void msp432_boostxl_init_poll(BOOSTXL_IN_enum_t  _audioin,
                              msp432_sample_process_t _cb
                             );

void msp432_boostxl_init_intr(FS_enum_t          _fs,
                              BOOSTXL_IN_enum_t  _audioin,
                              msp432_sample_process_t _cb
                             );

void msp432_boostxl_init_dma (FS_enum_t          _fs,
                              BOOSTXL_IN_enum_t  _audioin,
                              BUFLEN_enum_t      _pplen,
                              msp432_buffer_process_t _cb
                             );

void msp432_boostxl_run();

void errorledon();
void errorledoff();

void colorledred();
void colorledgreen();
void colorledblue();
void colorledoff();

void debugpinhigh();
void debugpinlow();

uint32_t measurePerfSample(msp432_sample_process_t _cb);
uint32_t measurePerfBuffer(msp432_buffer_process_t _cb);

int pushButtonLeftUp();
int pushButtonLeftDown();
int pushButtonRightUp();
int pushButtonRightDown();

#endif /* MSP432_BOOSTXL_INIT_H_ */

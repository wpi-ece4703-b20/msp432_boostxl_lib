# MSP432_BOOSTXL_LIB

MSP432_BOOSTXL_LIB is a support library that offers support for flexible input/output
in a real-time DSP application. The library is written for an MSPEXP432P401R Launchpad.

Written by Patrick Schaumont (pschaumont@wpi.edu) at Worcester Polytechnic Institute in 2020

msp432_boostxl is free software; you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free
   Software Foundation; either version 3, or (at your option) any later
   version.

msp432_boostxl is distributed in the hope that it will be useful, but WITHOUT ANY
   WARRANTY; without even the implied warranty of MERCHANTABILITY or
   FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
   for more details.
   You should have received a copy of the GNU General Public License
   along with msp432_boostxl; see the file COPYING3.  If not see
   <http://www.gnu.org/licenses/>. 

The following is a list of the public functions defined in this library.

## msp432_boostxl_init()

This function configures the processor clock of the ARM Cortex M4 to 48MHz. The function can be used when neither D/A nor A/D conversions are required.

```
    #include "msp432_boostxl_init.h"
    void msp432_boostxl_init();
```

## msp432_boostxl_run()

This function starts the conversion process. Before calling this function the user must call one of the following initialization functions.

*  ``msp432_boostxl_init_poll()`` for polling mode conversion
*  ``msp432_boostxl_init_intr()`` for interrupt mode conversion
*  ``msp432_boostxl_init_dma()`` for dma mode conversion

```
    #include "msp432_boostxl_init.h"
    void msp432_boostxl_run();
```

## msp432_boostxl_init_poll()

This function configures the processor clock of the ARM Cortex M4 to 48MHz, turns on the microphone, and initializes the hardware in polling mode. In polling mode, the following steps are repeated as fast as the hardware allows: (1) An A/D conversion from a user-defined sources is completed, (2) a user-defined call-back function is executed with the converted sample as argument, (3) the value returned by the user-defined call-back function is forwarded to the D/A. The speed of A/D conversions is limited due to the speed of the successive approximation ADC in the MSP432. The speed of the D/A conversions is limited by the SPI connection between the MSP432 and the off-chip DAC8331.

```
     #include "msp432_boostxl_init.h"
     void msp432_boostxl_init_poll(BOOSTXL_IN_enum_t  _audioin,
                                   msp432_sample_process_t _cb
                                   );
```

* ``BOOSTXL_IN_enum_t  _audioin`` indicates the source used by the ADC converter, and is one of the following selections.

  * ``BOOSTXL_MIC_IN`` the source signal is taken from the BOOSTXL board microphone
  * ``BOOSTXL_J1_2_IN`` the source signal is taken from pin 2 of header J1
   
* ``msp432_sample_process_t _cb`` is a pointer to the callback function called after ADC conversion.
  The callback function reads a 16-bit unsigned integer and returns a 16-bit unsigned integer, i.e.
  ``typedef uint16_t (*msp432_sample_process_t)(uint16_t);``

## msp432_boostxl_init_intr()

This function configures the processor clock of the ARM Cortex M4 to 48MHz, turns on the microphone, and initializes the hardware in interrupt mode. In interrupt mode, a hardware timer is set to expire at a user-defined sample rate. When the hardware timer expires, an A/D conversion from a user-defined source is started. When the A/D conversion completes, a user-defined call-back function is called as part of the A/D interrupt service routine. The value returned from the user-defined call-back function is forwarded to the D/A. The audio processing thus proceeds at the user-defined sample rate. However, for correct operation, the processing time per sample must be smaller than the sample period. The processing time per sample is the sum of the execution time of the user-defined call-back function plus the time to make a D/A conversion. When the processing time per sample exceeds this sum, a timer interrupt will be lost and the sampling process will no longer reach the selected
user-defined sample rate.

```
   #include "msp432_boostxl_init.h"

   void msp432_boostxl_init_intr(FS_enum_t          _fs,
                                 BOOSTXL_IN_enum_t  _audioin,
                                 msp432_sample_process_t _cb
                                 );
```

* ``FS_enum_t _fs`` selects the sample rate for the A/D and D/A conversion process.

   * ``FS_8000_HZ`` selects a sample rate of 8,000 Hz
   * ``FS_11025_HZ`` selects a sample rate of 11,025 Hz
   * ``FS_16000_HZ`` selects a sample rate of 16,000 Hz
   * ``FS_22050_HZ`` selects a sample rate of 22,050 Hz
   * ``FS_24000_HZ`` selects a sample rate of 24,000 Hz
   * ``FS_32000_HZ`` selects a sample rate of 32,000 Hz
   * ``FS_44100_HZ`` selects a sample rate of 44,100 Hz
   * ``FS_48000_HZ`` selects a sample rate of 48,000 Hz

* ``BOOSTXL_IN_enum_t  _audioin`` indicates the source used by the ADC converter, and is one of the following selections.

  * ``BOOSTXL_MIC_IN`` the source signal is taken from the BOOSTXL board microphone
  * ``BOOSTXL_J1_2_IN`` the source signal is taken from pin 2 of header J1
   
* ``msp432_sample_process_t _cb`` is a pointer to the callback function called after ADC conversion.
  The callback function reads a 16-bit unsigned integer and returns a 16-bit unsigned integer, i.e.
  ``typedef uint16_t (*msp432_sample_process_t)(uint16_t);``

## msp432_boostxl_init_dma()

This function configures the processor clock of the ARM Cortex M4 to 48MHz, turns on the microphone, and initializes the hardware in DMA mode. In DMA mode, a hardware timer is set to expire at a user-defined sample rate. When the hardware timer expires, an A/D conversion from a user-defined source is started. When the A/D conversion completes, the sample is copied by a Direct Memory Access (DMA) module into a buffer. When a user-defined number of samples are gathered in the buffer, a user-defined call-back is called to process a block of samples. The user-defined call-back function
returns a block of samples from the same size. In dma mode, a hardware timer interrupt is then used to copy these output samples to the D/A module, one sample at a time. The DMA mode uses ping-pong
buffers to keep the sampling process and the data processing apart. There are two pairs of
ping-pong buffers: one pair is associated with the A/D conversion, and a second pair is associated with the D/A conversion.

```
   #include "msp432_boostxl_init.h"

   void msp432_boostxl_init_dma (FS_enum_t          _fs,
                                 BOOSTXL_IN_enum_t  _audioin,
                                 BUFLEN_enum_t      _pplen,
                                 msp432_buffer_process_t _cb
                                );
```

* ``FS_enum_t _fs`` selects the sample rate for the A/D and D/A conversion process.

   * ``FS_8000_HZ`` selects a sample rate of 8,000 Hz
   * ``FS_11025_HZ`` selects a sample rate of 11,025 Hz
   * ``FS_16000_HZ`` selects a sample rate of 16,000 Hz
   * ``FS_22050_HZ`` selects a sample rate of 22,050 Hz
   * ``FS_24000_HZ`` selects a sample rate of 24,000 Hz
   * ``FS_32000_HZ`` selects a sample rate of 32,000 Hz
   * ``FS_44100_HZ`` selects a sample rate of 44,100 Hz
   * ``FS_48000_HZ`` selects a sample rate of 48,000 Hz

* ``BOOSTXL_IN_enum_t _audioin`` indicates the source used by the ADC converter, and is one of the following selections.

  * ``BOOSTXL_MIC_IN`` the source signal is taken from the BOOSTXL board microphone
  * ``BOOSTXL_J1_2_IN`` the source signal is taken from pin 2 of header J1

* ``BUFLEN_enum_t _pplen`` selects the size of the DMA ping-pong buffer.

  * ``BUFLEN_8`` selects a buffer size of 8 samples (for each ping buffer and each pong buffer)
  * ``BUFLEN_16`` selects a buffer size of 16 samples (for each ping buffer and each pong buffer)
  * ``BUFLEN_32`` selects a buffer size of 32 samples (for each ping buffer and each pong buffer)
  * ``BUFLEN_64`` selects a buffer size of 64 samples (for each ping buffer and each pong buffer)
  * ``BUFLEN_128`` selects a buffer size of 128 samples (for each ping buffer and each pong buffer)
   
* ``msp432_buffer_process_t _cb`` is a pointer to the callback function called after the ping-pong buffers have filled up. The calllback function has type ``typedef void (*msp432_buffer_process_t)(uint16_t *, uint16_t *)`` and takes two arguments. The first argument is a pointer to the input buffer of ``_pplen`` elements of type ``uint16_t``. The second argument is a pointer to the output buffer of ``_pplen`` elements of type ``uint16_t``. The callback function returns void.

## measurePerfSample()

This function measures the median execution time, counted in clock cycles, of a sample-based callback function such as used in polled mode and in interrupt mode. The measurement process proceeds as follows. The callback function is called 11 times, using a dummy (0) input argument. The return value of the callback function is ignored, and it is not forwarded to the D/A. Each of the 11 executions are measured using a hardware timer. The resulting set of cycle counts is sorted and the median (middle) element is returned. The measurement function adjusts the result and subtracts measurement overhead.

```
   #include "msp432_boostxl_init.h"

   uint32_t measurePerfSample(msp432_sample_process_t _cb);
```

   * ``msp432_sample_process_t _cb`` is a pointer to the callback function called after ADC conversion. The callback function reads a 16-bit unsigned integer and returns a 16-bit unsigned integer, i.e. ``typedef uint16_t (*msp432_sample_process_t)(uint16_t);``


## measurePerfBuffer()

This function measures the median execution time, counted in clock cycles, of a buffer-based callback function such as used in dma mode. The measurement process proceeds as follows. The callback function is called 11 times, using a dummy input buffer of zeroes. The return buffer is ignored, and it is not forwarded to the D/A. Each of the 11 executions are measured using a hardware timer. The resulting set of cycle counts is sorted and the median (middle) element is returned. The measurement function adjusts the result and subtracts measurement overhead.


```
   #include "msp432_boostxl_init.h"

   uint32_t measurePerfBuffer(msp432_buffer_process_t _cb);
```

* ``msp432_buffer_process_t _cb`` is a pointer to the callback function called after the ping-pong buffers have filled up. The calllback function has type ``typedef void (*msp432_buffer_process_t)(uint16_t *, uint16_t *)`` and takes two arguments. The first argument is a pointer to the input buffer of ``_pplen`` elements of type ``uint16_t``. The second argument is a pointer to the output buffer of ``_pplen`` elements of type ``uint16_t``. The callback function returns void.

## adc14_to_q15

This function converts a 14-bit ADC value to a fixed-point fix<16,15> Q15 value.
The encoded range spans [-0.25,0.25] corresponding to the analog range [0v, 3v3].

```
   #include "msp432_arm_dsp.h"

   q15_t     adc14_to_q15(uint16_t v);
```

## adc14_to_q31

This function converts a 14-bit ADC value to a fixed-point fix<32,31> Q31 value.
The encoded range spans [-0.25,0.25] corresponding to the analog range [0v, 3v3].

```
   #include "msp432_arm_dsp.h"

   q31_t     adc14_to_q31(uint16_t v);
```

## adc14_to_f32

This function converts a 14-bit ADC value to a single-precision floating point value.
The encoded range spans [-0.25,0.25] corresponding to the analog range [0v, 3v3].

```
   #include "msp432_arm_dsp.h"

   float32_t adc14_to_f32(uint16_t v);
```

## q15_to_dac14

This function converts a fixed-point fix<16,15> Q15 value to a 14-bit DAC value.
The analog range spans [0v, 3v3] corresponding to range [-0.25,0.25].

```
   #include "msp432_arm_dsp.h"

   uint16_t  q15_to_dac14(q15_t v);
```

## q31_to_dac14

This function converts a fixed-point fix<16,15> Q15 value to a 14-bit DAC value.
The analog range spans [0v, 3v3] corresponding to range [-0.25,0.25].

```
   #include "msp432_arm_dsp.h"

   uint16_t  q31_to_dac14(q31_t v);
```

## f32_to_dac14

This function converts a single-precision floating point value to a 14-bit DAC value.
The analog range spans [0v, 3v3] corresponding to range [-0.25,0.25].

```
   #include "msp432_arm_dsp.h"

   uint16_t  f32_to_dac14(float32_t v);
```

## errorledoff()

This function turns off LED 2 of the MSPEXP432P401R board.

.. code:: c

```
   #include "msp432_boostxl_init.h"

   void errorledon();
```

## errorledon()

This function turns on LED 2 of the MSPEXP432P401R board.

```
   #include "msp432_boostxl_init.h"

   void errorledoff();
```


## colorledred()

This function sets on LED 1 of the MSPEXP432P401R board to red color.

```
   #include "msp432_boostxl_init.h"

   void colorledred();
```

## colorledgreen()

This function sets on LED 1 of the MSPEXP432P401R board to green color.

```
   #include "msp432_boostxl_init.h"

   void colorledgreen();
```

## colorledblue()

This function sets on LED 1 of the MSPEXP432P401R board to blue color.

```
   #include "msp432_boostxl_init.h"

   void colorledblue();
```

## colorledoff()

This function turns off LED 1 of the MSPEXP432P401R board.

```
   #include "msp432_boostxl_init.h"

   void colorledoff();
```

## debugpinhigh()

This function pulls high pin J4.32 (GPIO P3.5) of the MSPEXP432P401R board.

```
   #include "msp432_boostxl_init.h"

   void debugpinhigh();
```

## debugpinlow()

This function pulls low pin J4.32 (GPIO P3.5) of the MSPEXP432P401R board.

```
   #include "msp432_boostxl_init.h"

   void debugpinlow();
```

## pushButtonLeftUp()

This function returns true (nonzero) if the left button of the MSPEXP432P401R board is not pressed.

```
   #include "msp432_boostxl_init.h"

   int pushButtonLeftUp();
```

## pushButtonLeftDown()

This function returns true (nonzero) if the left button of the MSPEXP432P401R board is pressed.

```
   #include "msp432_boostxl_init.h"

   int pushButtonLeftDown();
```

## pushButtonRightUp()

This function returns true (nonzero) if the right button of the MSPEXP432P401R board is not pressed.

```
   #include "msp432_boostxl_init.h"

   int pushButtonRightUp();
```

## pushButtonRightDown()

This function returns true (nonzero) if the right button of the MSPEXP432P401R board is pressed.

```
   #include "msp432_boostxl_init.h"

   int pushButtonRightDown();
```

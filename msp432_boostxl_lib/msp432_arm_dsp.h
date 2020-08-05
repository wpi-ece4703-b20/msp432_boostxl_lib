#ifndef MSP432_ARM_DSP_H_
#define MSP432_ARM_DSP_H_

#include "arm_math.h"

void adc14_to_q15_vec(const uint16_t *v, q15_t *q, uint32_t sz);
void adc14_to_q31_vec(const uint16_t *v, q31_t *q, uint32_t sz);
void adc14_to_f32_vec(const uint16_t *v, float32_t *q, uint32_t sz);
void q15_to_dac14_vec(const q15_t *v, uint16_t *q, uint32_t sz);
void q31_to_dac14_vec(const q31_t *v, uint16_t *q, uint32_t sz);
void f32_to_dac14_vec(const float32_t *v, uint16_t *q, uint32_t sz);

q15_t     adc14_to_q15(uint16_t v);
q31_t     adc14_to_q31(uint16_t v);
float32_t adc14_to_f32(uint16_t v);
uint16_t  q15_to_dac14(q15_t v);
uint16_t  q31_to_dac14(q31_t v);
uint16_t  f32_to_dac14(float32_t v);


#endif /* MSP432_ARM_DSP_H_ */

#include <stdint.h>

extern uint32_t _sidata, _sdata, _edata, _sbss, _ebss, _estack;
extern int main(void);

void Reset_Handler(void) {
    uint32_t *src = &_sidata, *dst = &_sdata;
    while (dst < &_edata) *dst++ = *src++;
    for (dst = &_sbss; dst < &_ebss; dst++) *dst = 0;
    main();
    while (1) {}
}

void Default_Handler(void) { while (1) {} }

__attribute__((section(".isr_vector")))
void (* const vector_table[])(void) = {
    (void(*)(void))&_estack,
    Reset_Handler,
    Default_Handler, Default_Handler, Default_Handler, Default_Handler,
    Default_Handler, 0, 0, 0, 0,
    Default_Handler, Default_Handler, 0, Default_Handler, Default_Handler,
};

/*
Copyright 2018 Massdrop Inc.

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "sam.h"
#include "tmk_core/common/keyboard.h"

#include "report.h"
#include "host.h"
#include "host_driver.h"
#include "keycode_config.h"
#include <string.h>
#include "quantum.h"

// From protocol directory
#include "arm_atsam_protocol.h"

// From keyboard's directory
#include "config_led.h"

uint8_t g_usb_state = USB_FSMSTATUS_FSMSTATE_OFF_Val;  // Saved USB state from hardware value to detect changes

void    main_subtasks(void);
uint8_t keyboard_leds(void);
void    send_keyboard(report_keyboard_t *report);
void    send_mouse(report_mouse_t *report);
void    send_system(uint16_t data);
void    send_consumer(uint16_t data);

host_driver_t arm_atsam_driver = {keyboard_leds, send_keyboard, send_mouse, send_system, send_consumer};

uint8_t led_states;

uint8_t keyboard_leds(void) {
#ifdef NKRO_ENABLE
    if (keymap_config.nkro)
        return udi_hid_nkro_report_set;
    else
#endif  // NKRO_ENABLE
        return udi_hid_kbd_report_set;
}

void puts_(const char *s) {
    while(*s) {
	while (SERCOM3->USART.INTFLAG.bit.DRE == 0);
	SERCOM3->USART.DATA.reg = *s++;
    }
}

#if 0
void send_keyboard(report_keyboard_t *report) {
    (void)report;
    puts_("send_keyboard\n");
}
#endif

#if 1
void send_keyboard(report_keyboard_t *report) {
    uint32_t irqflags;

#ifdef NKRO_ENABLE
    if (!keymap_config.nkro) {
#endif  // NKRO_ENABLE
        while (udi_hid_kbd_b_report_trans_ongoing) {
            main_subtasks();
        }  // Run other tasks while waiting for USB to be free

        irqflags = __get_PRIMASK();
        __disable_irq();
        __DMB();

    puts_("sen_key\n");
        memcpy(udi_hid_kbd_report, report->raw, UDI_HID_KBD_REPORT_SIZE);
        udi_hid_kbd_b_report_valid = 1;
        udi_hid_kbd_send_report();

        __DMB();
        __set_PRIMASK(irqflags);
#ifdef NKRO_ENABLE
    } else {
        while (udi_hid_nkro_b_report_trans_ongoing) {
            main_subtasks();
        }  // Run other tasks while waiting for USB to be free

        irqflags = __get_PRIMASK();
        __disable_irq();
        __DMB();

        memcpy(udi_hid_nkro_report, report->raw, UDI_HID_NKRO_REPORT_SIZE);
        udi_hid_nkro_b_report_valid = 1;
        udi_hid_nkro_send_report();

        __DMB();
        __set_PRIMASK(irqflags);
    }
#endif  // NKRO_ENABLE
}
#endif

void send_mouse(report_mouse_t *report) {
#ifdef MOUSEKEY_ENABLE
    uint32_t irqflags;

    irqflags = __get_PRIMASK();
    __disable_irq();
    __DMB();

    memcpy(udi_hid_mou_report, report, UDI_HID_MOU_REPORT_SIZE);
    udi_hid_mou_b_report_valid = 1;
    udi_hid_mou_send_report();

    __DMB();
    __set_PRIMASK(irqflags);
#endif  // MOUSEKEY_ENABLE
}

#ifdef EXTRAKEY_ENABLE
void send_extra(uint8_t report_id, uint16_t data) {
    uint32_t irqflags;

    irqflags = __get_PRIMASK();
    __disable_irq();
    __DMB();

    udi_hid_exk_report.desc.report_id   = report_id;
    udi_hid_exk_report.desc.report_data = data;
    udi_hid_exk_b_report_valid          = 1;
    udi_hid_exk_send_report();

    __DMB();
    __set_PRIMASK(irqflags);
}
#endif  // EXTRAKEY_ENABLE

void send_system(uint16_t data) {
#ifdef EXTRAKEY_ENABLE
    send_extra(REPORT_ID_SYSTEM, data);
#endif  // EXTRAKEY_ENABLE
}

void send_consumer(uint16_t data) {
#ifdef EXTRAKEY_ENABLE
    send_extra(REPORT_ID_CONSUMER, data);
#endif  // EXTRAKEY_ENABLE
}

void main_subtask_usb_state(void) {
    static uint64_t fsmstate_on_delay = 0;                          // Delay timer to be sure USB is actually operating before bringing up hardware
    uint8_t         fsmstate_now      = USB->DEVICE.FSMSTATUS.reg;  // Current state from hardware register

    if (fsmstate_now == USB_FSMSTATUS_FSMSTATE_SUSPEND_Val)  // If USB SUSPENDED
    {
        fsmstate_on_delay = 0;  // Clear ON delay timer

        if (g_usb_state != USB_FSMSTATUS_FSMSTATE_SUSPEND_Val)  // If previously not SUSPENDED
        {
            suspend_power_down();        // Run suspend routine
            g_usb_state = fsmstate_now;  // Save current USB state
        }
    } else if (fsmstate_now == USB_FSMSTATUS_FSMSTATE_SLEEP_Val)  // Else if USB SLEEPING
    {
        fsmstate_on_delay = 0;  // Clear ON delay timer

        if (g_usb_state != USB_FSMSTATUS_FSMSTATE_SLEEP_Val)  // If previously not SLEEPING
        {
            suspend_power_down();        // Run suspend routine
            g_usb_state = fsmstate_now;  // Save current USB state
        }
    } else if (fsmstate_now == USB_FSMSTATUS_FSMSTATE_ON_Val)  // Else if USB ON
    {
        if (g_usb_state != USB_FSMSTATUS_FSMSTATE_ON_Val)  // If previously not ON
        {
            if (fsmstate_on_delay == 0)  // If ON delay timer is cleared
            {
                fsmstate_on_delay = timer_read64() + 250;   // Set ON delay timer
            } else if (timer_read64() > fsmstate_on_delay)  // Else if ON delay timer is active and timed out
            {
                suspend_wakeup_init();       // Run wakeup routine
                g_usb_state = fsmstate_now;  // Save current USB state
            }
        }
    } else  // Else if USB is in a state not being tracked
    {
        fsmstate_on_delay = 0;  // Clear ON delay timer
    }
}

void main_subtask_power_check(void) {
    static uint64_t next_5v_checkup = 0;

    if (timer_read64() > next_5v_checkup) {
        next_5v_checkup = timer_read64() + 5;

#ifdef USB_DUALPORT_ENABLE
        v_5v     = adc_get(ADC_5V);
        v_5v_avg = 0.9 * v_5v_avg + 0.1 * v_5v;
#endif

#ifdef RGB_MATRIX_ENABLE
        gcr_compute();
#endif
    }
}

#ifdef USB_DUALPORT_ENABLE
void main_subtask_usb_extra_device(void) {
    static uint64_t next_usb_checkup = 0;

    if (timer_read64() > next_usb_checkup) {
        next_usb_checkup = timer_read64() + 10;

        USB_HandleExtraDevice();
    }
}
#endif

#ifdef RAW_ENABLE
void main_subtask_raw(void) { udi_hid_raw_receive_report(); }
#endif

void main_subtasks(void) {
    main_subtask_usb_state();
    main_subtask_power_check();
#ifdef USB_DUALPORT_ENABLE
    main_subtask_usb_extra_device();
#ifdef RAW_ENABLE
    main_subtask_raw();
#endif
#endif
}

void adhoc_tc4_reset(void) {
    ms_clk = 0;

    DBGC(DC_CLK_RESET_TIME_BEGIN);

    // stop counters
    TC4->COUNT16.CTRLA.bit.ENABLE = 0;
    while (TC4->COUNT16.STATUS.bit.SYNCBUSY) {
    }
    // zero counters
    TC4->COUNT16.COUNT.reg = 0;
    while (TC4->COUNT16.STATUS.bit.SYNCBUSY) {
    }
    // start counters
    TC4->COUNT16.CTRLA.bit.ENABLE = 1;
    while (TC4->COUNT16.STATUS.bit.SYNCBUSY) {
    }

    DBGC(DC_CLK_RESET_TIME_COMPLETE);
}

void adhoc_tc4_init(void) {
    // unmask TC4, sourcegclk2 to TC4
    PM->APBCMASK.bit.TC4_ = 1;
    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID_TC4_TC5;

    // configure TC4
    DBGC(DC_CLK_ENABLE_TIMEBASE_TC4_BEGIN);
    TC4->COUNT16.CTRLA.bit.ENABLE = 0;
    while (TC4->COUNT16.STATUS.bit.SYNCBUSY) {
        DBGC(DC_CLK_ENABLE_TIMEBASE_TC4_SYNC_DISABLE);
    }
    TC4->COUNT16.CTRLA.bit.SWRST = 1;
    while (TC4->COUNT16.STATUS.bit.SYNCBUSY) {
        DBGC(DC_CLK_ENABLE_TIMEBASE_TC4_SYNC_SWRST_1);
    }
    while (TC4->COUNT16.CTRLA.bit.SWRST) {
        DBGC(DC_CLK_ENABLE_TIMEBASE_TC4_SYNC_SWRST_2);
    }

    // CTRLA defaults
    // CTRLB as default, counting up
    TC4->COUNT16.CTRLBCLR.reg = 5;
    while (TC4->COUNT16.STATUS.bit.SYNCBUSY) {
        DBGC(DC_CLK_ENABLE_TIMEBASE_TC4_SYNC_CLTRB);
    }
    // TC4->COUNT16.CC[0].reg = 999;
    TC4->COUNT16.CC[0].reg = 47999;
    while (TC4->COUNT16.STATUS.bit.SYNCBUSY) {
        DBGC(DC_CLK_ENABLE_TIMEBASE_TC4_SYNC_CC0);
    }
    // TC4->COUNT16.DBGCTRL.bit.DBGRUN = 1;

    // wave mode
    TC4->COUNT16.CTRLA.bit.WAVEGEN = 1;  // MFRQ match frequency mode, toggle each CC match
    // generate event for next stage
    TC4->COUNT16.EVCTRL.bit.MCEO0 = 1;

    NVIC_EnableIRQ(TC4_IRQn);
    TC4->COUNT16.INTENSET.bit.MC0 = 1;

    DBGC(DC_CLK_ENABLE_TIMEBASE_TC4_COMPLETE);
}

uint32_t adhoc_dir;
uint32_t adhoc_out;

void adhoc_init(void) {
    // Set OSC8M prescaler to 1.
    SYSCTRL->OSC8M.bit.PRESC = 0; // divide by 1

    // Init XOSC32K
    SYSCTRL->XOSC32K.reg = SYSCTRL_XOSC32K_STARTUP(4) | SYSCTRL_XOSC32K_EN32K | SYSCTRL_XOSC32K_XTALEN;
    SYSCTRL->XOSC32K.bit.ENABLE = 1;
    while (SYSCTRL->PCLKSR.bit.XOSC32KRDY == 0); // wait XOSC32KRDY

    // Init GCLK1 as XOSC32K
    GCLK->GENCTRL.reg = GCLK_GENCTRL_GENEN | GCLK_GENCTRL_SRC_XOSC32K | GCLK_GENCTRL_ID(1);
    while (GCLK->STATUS.bit.SYNCBUSY);
    // Set GCLK1  divided by 1
    GCLK->GENDIV.reg = GCLK_GENDIV_DIV(0) | GCLK_GENDIV_ID(1); // 0 is treated same as 1

    // Setup DLL to 48MHz
    // Set DFLL48M input clock to GCLK1 that is XOSC32K
    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK1 | GCLK_CLKCTRL_ID_DFLL48;
    while (GCLK->STATUS.bit.SYNCBUSY);
    SYSCTRL->DFLLCTRL.reg = SYSCTRL_DFLLCTRL_ENABLE; // ENABLE=1 MODE=open-loop
    while (SYSCTRL->PCLKSR.bit.DFLLRDY == 0); // wait DFLLRDY
    uint16_t mul = (48000000 + 32768/2) / 32768;
    SYSCTRL->DFLLMUL.reg = SYSCTRL_DFLLMUL_CSTEP(31) | SYSCTRL_DFLLMUL_FSTEP(511) | SYSCTRL_DFLLMUL_MUL(mul);
    while (SYSCTRL->PCLKSR.bit.DFLLRDY == 0); // wait DFLLRDY
    SYSCTRL->DFLLCTRL.reg |= SYSCTRL_DFLLCTRL_MODE | SYSCTRL_DFLLCTRL_WAITLOCK | SYSCTRL_DFLLCTRL_QLDIS; // MODE=closed-loop
    while (SYSCTRL->PCLKSR.bit.DFLLRDY == 0); // wait DFLLRDY
    // may not need the following write (ENABLE=1)
    SYSCTRL->DFLLCTRL.bit.ENABLE = 1;
    while (SYSCTRL->PCLKSR.bit.DFLLLCKC == 0 || SYSCTRL->PCLKSR.bit.DFLLLCKF == 0);
    // do we need the following sync?
    while (SYSCTRL->PCLKSR.bit.DFLLRDY == 0); // wait DFLLRDY

    // Reinit GCLK0 as 48MHz from DLL (DFLL48M)
    NVMCTRL->CTRLB.bit.RWS = 1; // 1 wait state
    GCLK->GENCTRL.reg = GCLK_GENCTRL_IDC | GCLK_GENCTRL_GENEN | GCLK_GENCTRL_SRC_DFLL48M | GCLK_GENCTRL_ID(0);
    while (GCLK->STATUS.bit.SYNCBUSY);
    // Set GCLK0  divided by 1
    GCLK->GENDIV.reg = GCLK_GENDIV_DIV(0) | GCLK_GENDIV_ID(0); // 0 is treated same as 1

    // Enable PAC2 and SERCOM3
    PM->APBCMASK.bit.PAC2_ = 1;
    PM->APBCMASK.bit.SERCOM3_ = 1;

    // Set SERCOM3 clock to GCLK0 that is OSC8M or DFLL48M
    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID_SERCOM3_CORE;
    // Set SERCOM3 clock to GCLK1 that is OSC32K or XOSC32K
    // GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK1 | GCLK_CLKCTRL_ID_SERCOM3_CORE;

    // Disable SERCOM3
    SERCOM3->USART.CTRLA.bit.ENABLE = 0;
    while (SERCOM3->USART.SYNCBUSY.bit.ENABLE);

    // Reset SERCOM3
    SERCOM3->USART.CTRLA.bit.SWRST = 1;
    while (SERCOM3->USART.SYNCBUSY.bit.SWRST);

    //  MODE=1 (internal clock) DORD=1
    SERCOM3->USART.CTRLA.reg = SERCOM_USART_CTRLA_DORD | SERCOM_USART_CTRLA_MODE_USART_INT_CLK; 

    SERCOM3->USART.BAUD.reg = 63019; // Asynchronus Arithmetic 115219.12 bps at 48MHz
    // SERCOM3->USART.BAUD.reg = 50436; // Asynchronus Arithmetic 115203.86 bps at OSC8M=8MHz
    // SERCOM3->USART.BAUD.reg = 25271; // Asynchronus Arithmetic 38399.70 bps at OSC8M=1MHz
    // SERCOM3->USART.BAUD.reg = 27136; // Asynchronus Arithmetic 1200 bps at (X)OSC32K=32.768kHz

    //  MODE=1 (internal clock)  DORD=1 ENABLE=1
    SERCOM3->USART.CTRLA.bit.ENABLE = 1;
    while (SERCOM3->USART.SYNCBUSY.bit.ENABLE);

    // TXEN=1
    SERCOM3->USART.CTRLB.bit.TXEN = 1;
    while (SERCOM3->USART.SYNCBUSY.bit.CTRLB);

    // Enable PMUX for PA22 and assign function C that is SERCOM3 PAD[0]
    PORT->Group[0].PMUX[11].bit.PMUXE = 2;  // PA22 function column C for SERCOM3 PAD[0]
    PORT->Group[0].PINCFG[22].bit.PMUXEN = 1;

    // PORT->Group[0].PINCFG[23].bit.INEN = 1; // PA23 => Inuput
#if 0
    PORT->Group[0].DIRSET.reg = (1 << 8); // PA08 => Output
    int prev = 0;
    do {
	for (int i = 0; i < 800000; i++) {
	    // PORT->Group[0].OUT.reg = 0x00000100;
	    DBG_LED_OFF;
	}
	for (int i = 0; i < 1600000; i++) {
	    // PORT->Group[0].OUT.reg = 0x00000000;
	    DBG_LED_ON;
	}
	int curr = (PORT->Group[0].IN.reg & (1 << 23)) ? 0 : 1;
	if (!prev && curr) {
	    puts_("Hayon\n");
	}
	prev = curr;
    } while(1);
    do {} while(1);
#endif
    // puts_("Adhoc_tc4_init()\n");
    adhoc_tc4_init();
    // puts_("Adhoc_tc4_reset()\n");
    adhoc_tc4_reset();
#if 0
    do {
	puts_("B\n");
	CLK_delay_ms(1000);
    } while(1);
#endif
    // set I2C pins low
    PORT->Group[0].DIRSET.reg = (1 << 22) | (1 << 23);
    PORT->Group[0].OUTCLR.reg = (1 << 22) | (1 << 23);
    PORT->Group[0].DIRSET.reg = (1 << 10); // PA10 => Output
    PORT->Group[0].OUTCLR.reg = (1 << 10); // PA10 => Low (side-switch)
    CLK_delay_ms(10);
    PORT->Group[0].OUTSET.reg = (1 << 10); // PA10 => High (side-switch)
    adhoc_dir = PORT->Group[0].DIR.reg;
    adhoc_out = PORT->Group[0].OUT.reg;
>>>>>>> 5dd722d0f... apuro
}

int main(void) {
    adhoc_init();

    DBG_LED_ENA;
    DBG_1_ENA;
    DBG_1_OFF;
    DBG_2_ENA;
    DBG_2_OFF;
    DBG_3_ENA;
    DBG_3_OFF;

    debug_code_init();

    CLK_init();

    i2c0_init();

#ifdef USB_DUALPORT_ENABLE
    ADC0_init();

    SR_EXP_Init();
#endif

#ifdef RGB_MATRIX_ENABLE
    i2c1_init();
#endif  // RGB_MATRIX_ENABLE

    matrix_init();

#ifdef USB_DUALPORT_ENABLE
    USB2422_init();
#endif

    DBGC(DC_MAIN_UDC_START_BEGIN);
    udc_start();
    DBGC(DC_MAIN_UDC_START_COMPLETE);

    DBGC(DC_MAIN_CDC_INIT_BEGIN);
    CDC_init();
    DBGC(DC_MAIN_CDC_INIT_COMPLETE);

#ifdef USB_DUALPORT_ENABLE
    while (USB2422_Port_Detect_Init() == 0) {
    }
#endif

    DBG_LED_OFF;

#ifdef RGB_MATRIX_ENABLE
    while (I2C3733_Init_Control() != 1) {
    }
    while (I2C3733_Init_Drivers() != 1) {
    }

    I2C_DMAC_LED_Init();

    i2c_led_q_init();

    for (uint8_t drvid = 0; drvid < ISSI3733_DRIVER_COUNT; drvid++) I2C_LED_Q_ONOFF(drvid);  // Queue data
#endif                                                                                       // RGB_MATRIX_ENABLE

    keyboard_setup();

    keyboard_init();

#if !defined(NO_PRINT) && !defined(NO_DEBUG)
    // override settings from eeprom for ease of testing
        debug_enable = true;
        debug_matrix = true;
        debug_keyboard = true;
        debug_mouse = true;
    // dprint() , dprintln() and dprintf() are activated here after
#endif

    host_set_driver(&arm_atsam_driver);

#ifdef CONSOLE_ENABLE
    uint64_t next_print = 0;
#endif  // CONSOLE_ENABLE

#ifdef USB_DUALPORT_ENABLE
    v_5v_avg = adc_get(ADC_5V);
#endif

    debug_code_disable();

    // int prev = 0;
    while (1) {
#if 0
	int curr = (PORT->Group[0].IN.reg & (1 << 23)) ? 0 : 1;
	if (!prev && curr) {
	    puts_("Hayon\n");
	}
	prev = curr;
	// puts_("@\n");
#endif
        main_subtasks();  // Note these tasks will also be run while waiting for USB keyboard polling intervals

        if (g_usb_state == USB_FSMSTATUS_FSMSTATE_SUSPEND_Val || g_usb_state == USB_FSMSTATUS_FSMSTATE_SLEEP_Val) {
            if (suspend_wakeup_condition()) {
                udc_remotewakeup();  // Send remote wakeup signal
                wait_ms(50);
            }

            continue;
        }

        keyboard_task();

#ifdef CONSOLE_ENABLE
        if (timer_read64() > next_print) {
            next_print = timer_read64() + 250;
            // Add any debug information here that you want to see very often
            // dprintf("5v=%u 5vu=%u dlow=%u dhi=%u gca=%u gcd=%u\r\n", v_5v, v_5v_avg, v_5v_avg - V5_LOW, v_5v_avg - V5_HIGH, gcr_actual, gcr_desired);
        }
#endif  // CONSOLE_ENABLE

        // Run housekeeping
        housekeeping_task();
    }

    return 1;
}

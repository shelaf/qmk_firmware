/* Copyright 2021 QMK
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once

#if defined(PROTOCOL_ARM_ATSAM)
#define PINDEF(port, pad) ((pin_t)((port << 5) | (pad)))
#        define  A02 PINDEF(0, 2)
#        define  A04 PINDEF(0, 4)
#        define  A05 PINDEF(0, 5)
#        define  A06 PINDEF(0, 6)
#        define  A07 PINDEF(0, 7)
#        define  A08 PINDEF(0, 8)
#        define  A09 PINDEF(0, 9)
#        define  A10 PINDEF(0, 10)
#        define  A11 PINDEF(0, 11)
#        define  B08 PINDEF(1, 8)
#        define  B09 PINDEF(1, 9)

#    define PIN_NUM(pin)  (pin & 0x1f)
#    define PIN_PORT(pin)  (PORT->Group[(pin >> 5)])
#    define PIN_MASK(pin)  ((uint32_t)(1 << PIN_NUM(pin)))
#    define setPinInput(pin) do { PIN_PORT(pin).DIRCLR.reg = PIN_MASK(pin); PIN_PORT(pin).PINCFG[PIN_NUM(pin)].reg = 2; } while(0)
#    define setPinInputHigh(pin) do { PortGroup *p = &PIN_PORT(pin); p->DIRCLR.reg = PIN_MASK(pin); p->OUTSET.reg = PIN_MASK(pin); p->PINCFG[PIN_NUM(pin)].reg = 6; } while(0)
#    define setPinOutput(pin) do { PortGroup *p = &PIN_PORT(pin); p->DIRSET.reg = PIN_MASK(pin); p->PINCFG[PIN_NUM(pin)].reg = 0; } while(0)

#    define writePinHigh(pin) do { PIN_PORT(pin).OUTSET.reg = PIN_MASK(pin); } while(0)
#    define writePinLow(pin) do { PIN_PORT(pin).OUTCLR.reg = PIN_MASK(pin); } while(0)
#    define writePin(pin, level) do { PortGroup *p = &PIN_PORT(pin); if (level) { p->OUT.reg |= PIN_MASK(pin); } else { p->OUT.reg &= ~PIN_MASK(pin); } } while(0)

#    define readPin(pin) ((PIN_PORT(pin).IN.reg & PIN_MASK(pin)) ? 1 : 0)

#    define togglePin(pin) xxxxxxxxxxxxx // not impletemnts

#endif

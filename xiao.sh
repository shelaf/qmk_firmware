#!/bin/bash
make -r -R -C $(dirname $0) -f build_keyboard.mk  KEYBOARD=xiao/direct_pins KEYMAP=default SILENT=false $*

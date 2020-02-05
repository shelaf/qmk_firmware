#!/bin/bash
make -r -R -C $(dirname $0) -f build_keyboard.mk  KEYBOARD=dygma/raise KEYMAP=default SILENT=false $*

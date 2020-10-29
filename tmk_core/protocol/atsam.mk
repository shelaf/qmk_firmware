ATSAM_DIR = protocol/arm_atsam

SRC += $(ATSAM_DIR)/adc.c
SRC += $(ATSAM_DIR)/clks.c
SRC += $(ATSAM_DIR)/d51_util.c
SRC += $(ATSAM_DIR)/i2c_master.c
ifeq ($(RGB_MATRIX_ENABLE),custom)
  SRC += $(ATSAM_DIR)/led_matrix_programs.c
  SRC += $(ATSAM_DIR)/led_matrix.c
endif
SRC += $(ATSAM_DIR)/main_arm_atsam.c
SRC += $(ATSAM_DIR)/spi.c
SRC += $(ATSAM_DIR)/startup.c

SRC += $(ATSAM_DIR)/usb/main_usb.c
SRC += $(ATSAM_DIR)/usb/udc.c
SRC += $(ATSAM_DIR)/usb/udi_cdc.c
SRC += $(ATSAM_DIR)/usb/udi_hid.c
SRC += $(ATSAM_DIR)/usb/udi_hid_kbd.c
SRC += $(ATSAM_DIR)/usb/udi_hid_kbd_desc.c
SRC += $(ATSAM_DIR)/usb/ui.c
SRC += $(ATSAM_DIR)/usb/usb2422.c
SRC += $(ATSAM_DIR)/usb/usb.c
SRC += $(ATSAM_DIR)/usb/usb_device_udd.c
SRC += $(ATSAM_DIR)/usb/usb_util.c

# Search Path
VPATH += $(TMK_DIR)/$(ATSAM_DIR)

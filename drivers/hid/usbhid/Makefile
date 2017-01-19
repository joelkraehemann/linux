#
# Makefile for the USB input drivers
#

usbhid-y	:= usbhid-core.o usbhid-quirks.o
usbhid-$(CONFIG_USB_HIDDEV)	+= usbhiddev.o
usbhid-$(CONFIG_HID_PID)	+= usbhid-pidff.o

obj-$(CONFIG_USB_HID)		+= usbhid.o
obj-$(CONFIG_USB_KBD)		+= usbkbd.o
obj-$(CONFIG_USB_MOUSE)		+= usbmouse.o


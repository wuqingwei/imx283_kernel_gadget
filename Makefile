#
# USB peripheral controller drivers
#
ifeq ($(CONFIG_USB_GADGET_DEBUG),y)
	EXTRA_CFLAGS		+= -DDEBUG
endif

obj-$(CONFIG_USB_DUMMY_HCD)	+= dummy_hcd.o
obj-$(CONFIG_USB_NET2280)	+= net2280.o
obj-$(CONFIG_USB_AMD5536UDC)	+= amd5536udc.o
obj-$(CONFIG_USB_PXA25X)	+= pxa25x_udc.o
obj-$(CONFIG_USB_PXA27X)	+= pxa27x_udc.o
obj-$(CONFIG_USB_IMX)		+= imx_udc.o
obj-$(CONFIG_USB_GOKU)		+= goku_udc.o
obj-$(CONFIG_USB_OMAP)		+= omap_udc.o
obj-$(CONFIG_USB_LH7A40X)	+= lh7a40x_udc.o
obj-$(CONFIG_USB_S3C2410)	+= s3c2410_udc.o
obj-$(CONFIG_USB_AT91)		+= at91_udc.o
obj-$(CONFIG_USB_ATMEL_USBA)	+= atmel_usba_udc.o
obj-$(CONFIG_USB_FSL_USB2)	+= fsl_usb2_udc.o
fsl_usb2_udc-objs		:= fsl_udc_core.o
ifeq ($(CONFIG_ARCH_MXC),y)
fsl_usb2_udc-objs		+= fsl_mxc_udc.o
endif
obj-$(CONFIG_USB_M66592)	+= m66592-udc.o
obj-$(CONFIG_USB_R8A66597)	+= r8a66597-udc.o
obj-$(CONFIG_USB_FSL_QE)	+= fsl_qe_udc.o
obj-$(CONFIG_USB_CI13XXX)	+= ci13xxx_udc.o
obj-$(CONFIG_USB_S3C_HSOTG)	+= s3c-hsotg.o
obj-$(CONFIG_USB_LANGWELL)	+= langwell_udc.o
obj-$(CONFIG_USB_ARC)		+= arcotg_udc.o

#
# USB gadget drivers
#
g_zero-objs			:= zero.o
g_audio-objs			:= audio.o
g_ether-objs			:= ether.o
g_serial-objs			:= serial.o
g_midi-objs			:= gmidi.o
gadgetfs-objs			:= inode.o
g_file_storage-objs		:= file_storage.o
g_mass_storage-objs		:= mass_storage.o
g_printer-objs			:= printer.o
g_cdc-objs			:= cdc2.o
g_multi-objs			:= multi.o
g_hid-objs			:= hid.o
g_nokia-objs			:= nokia.o
g_webcam-objs			:= webcam.o

obj-$(CONFIG_USB_ZERO)		+= g_zero.o
obj-$(CONFIG_USB_AUDIO)		+= g_audio.o
obj-$(CONFIG_USB_ETH)		+= g_ether.o
obj-$(CONFIG_USB_GADGETFS)	+= gadgetfs.o
obj-$(CONFIG_USB_FUNCTIONFS)	+= g_ffs.o
obj-$(CONFIG_USB_ETH_FUNCTIONFS)	+= g_eth_ffs.o
obj-$(CONFIG_USB_FILE_STORAGE)	+= g_file_storage.o
obj-$(CONFIG_USB_MASS_STORAGE)	+= g_mass_storage.o
obj-$(CONFIG_USB_G_SERIAL)	+= g_serial.o
obj-$(CONFIG_USB_G_PRINTER)	+= g_printer.o
obj-$(CONFIG_USB_MIDI_GADGET)	+= g_midi.o
obj-$(CONFIG_USB_CDC_COMPOSITE) += g_cdc.o
obj-$(CONFIG_USB_G_HID)		+= g_hid.o
obj-$(CONFIG_USB_G_MULTI)	+= g_multi.o
obj-$(CONFIG_USB_G_NOKIA)	+= g_nokia.o
obj-$(CONFIG_USB_G_WEBCAM)	+= g_webcam.o


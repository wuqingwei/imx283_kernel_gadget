/*
 * zero.c -- Gadget Zero, for USB development
 *
 * Copyright (C) 2003-2008 David Brownell
 * Copyright (C) 2008 by Nokia Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */


/*
 * Gadget Zero only needs two bulk endpoints, and is an example of how you
 * can write a hardware-agnostic gadget driver running inside a USB device.
 * Some hardware details are visible, but don't affect most of the driver.
 *
 * Use it with the Linux host/master side "usbtest" driver to get a basic
 * functional test of your device-side usb stack, or with "usb-skeleton".
 *
 * It supports two similar configurations.  One sinks whatever the usb host
 * writes, and in return sources zeroes.  The other loops whatever the host
 * writes back, so the host can read it.
 *
 * Many drivers will only have one configuration, letting them be much
 * simpler if they also don't support high speed operation (like this
 * driver does).
 *
 * Why is *this* driver using two configurations, rather than setting up
 * two interfaces with different functions?  To help verify that multiple
 * configuration infrastucture is working correctly; also, so that it can
 * work with low capability USB controllers without four bulk endpoints.
 */

/*
 * driver assumes self-powered hardware, and
 * has no way for users to trigger remote wakeup.
 */

/* #define VERBOSE_DEBUG */

#include <linux/kernel.h>
#include <linux/utsname.h>
#include <linux/device.h>
#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>
#include "gadget_chips.h"
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/usb/input.h>
#include <linux/cdev.h>
#include <asm/uaccess.h>
#include <linux/fs.h>
#include <linux/poll.h>
#include <linux/types.h> /* size_t */
#include <linux/errno.h> /* error codes */
#include <asm/system.h>
#include <asm/io.h>
#include <linux/sched.h>

#include "usbstring.c"
#include "config.c"
#include "epautoconf.c"
/*-------------------------------------------------------------------------*/
static const char shortname[] = "zero";
static const char loopback[] = "loop input to output";
static const char longname[] = "Gadget Zero";
static const char source_sink[] = "source and sink data";
#define STRING_MANUFACTURER 25
#define STRING_PRODUCT 42
#define STRING_SERIAL 101
#define STRING_SOURCE_SINK 248
#define STRING_LOOPBACK 249

#define DRIVER_VENDOR_NUM 0xefef 
#define DRIVER_PRODUCT_NUM 0x0036 

static int usb_zero_major = 249;
/*-------------------------------------------------------------------------*/
static const char *EP_OUT_NAME; /* sink */
/*-------------------------------------------------------------------------*/

/* big enough to hold our biggest descriptor */
#define USB_BUFSIZ 256
struct zero_dev { //zero设备结构
	spinlock_t lock;
	struct usb_gadget *gadget;
	struct usb_request *req; /* for control responses */
	struct usb_ep *out_ep;
	struct cdev cdev;
	unsigned char data[128];
	unsigned int data_size;
	wait_queue_head_t bulkrq; 
};
#define CONFIG_LOOPBACK 2
static struct usb_device_descriptor device_desc = { //设备描述符
	.bLength = sizeof device_desc,
	.bDescriptorType = USB_DT_DEVICE,
	.bcdUSB = __constant_cpu_to_le16(0x0110),
	.bDeviceClass = USB_CLASS_VENDOR_SPEC,
	.idVendor = __constant_cpu_to_le16(DRIVER_VENDOR_NUM),
	.idProduct = __constant_cpu_to_le16(DRIVER_PRODUCT_NUM),
	.iManufacturer = STRING_MANUFACTURER,
	.iProduct = STRING_PRODUCT,
	.iSerialNumber = STRING_SERIAL,
	.bNumConfigurations = 1,
};
static struct usb_endpoint_descriptor fs_sink_desc = { //端点描述符
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,

	.bEndpointAddress = USB_DIR_OUT, //对主机端来说，输出
	.bmAttributes = USB_ENDPOINT_XFER_BULK,
};

static struct usb_config_descriptor loopback_config = { //配置描述符
	.bLength = sizeof loopback_config,
	.bDescriptorType = USB_DT_CONFIG,
	/* compute wTotalLength on the fly */
	.bNumInterfaces = 1,
	.bConfigurationValue = CONFIG_LOOPBACK,
	.iConfiguration = STRING_LOOPBACK,
	.bmAttributes = USB_CONFIG_ATT_ONE | USB_CONFIG_ATT_SELFPOWER,
	.bMaxPower = 1, /* self-powered */
};
static const struct usb_interface_descriptor loopback_intf = { //接口描述符
	.bLength = sizeof loopback_intf,
	.bDescriptorType = USB_DT_INTERFACE,

	.bNumEndpoints = 1,
	.bInterfaceClass = USB_CLASS_VENDOR_SPEC,
	.iInterface = STRING_LOOPBACK,
};
/* static strings, in UTF-8 */

static char manufacturer[50];
/* default serial number takes at least two packets */
static char serial[] = "0123456789.0123456789.0123456789";
static struct usb_string strings[] = { //字符串描述符
	{ STRING_MANUFACTURER, manufacturer, },
	{ STRING_PRODUCT, longname, },
	{ STRING_SERIAL, serial, },
	{ STRING_LOOPBACK, loopback, },
	{ STRING_SOURCE_SINK, source_sink, },
	{ } /* end of list */
};

static struct usb_gadget_strings stringtab = {
	.language = 0x0409, /* en-us */
	.strings = strings,
};

static const struct usb_descriptor_header *fs_loopback_function[] = {
	(struct usb_descriptor_header *) &loopback_intf,
	(struct usb_descriptor_header *) &fs_sink_desc,
	NULL,
};

static int usb_zero_open(struct inode *inode, struct file *file) //打开设备
{
	struct zero_dev *dev = container_of (inode->i_cdev, struct zero_dev, cdev);
	file->private_data = dev;
	init_waitqueue_head (&dev->bulkrq);

	return 0;
}

static int usb_zero_release(struct inode *inode, struct file *file) //关闭设备
{
	return 0;
}
static void free_ep_req(struct usb_ep *ep, struct usb_request *req)
{
	kfree(req->buf);
	usb_ep_free_request(ep, req);
}
static struct usb_request *alloc_ep_req(struct usb_ep *ep, unsigned length)//分配请求
{
	struct usb_request *req;

	req = usb_ep_alloc_request(ep, GFP_ATOMIC);
	if (req) {
		req->length = length;
		req->buf = kmalloc(length, GFP_ATOMIC);
		if (!req->buf) {
				usb_ep_free_request(ep, req);
				req = NULL;
		}
	}
	return req;
}
static void source_sink_complete(struct usb_ep *ep, struct usb_request *req)//请求完成函数
{
	struct zero_dev *dev = ep->driver_data;
	int status = req->status;
	switch (status) {
		case 0: /* normal completion */
		if (ep == dev->out_ep) {
				memcpy(dev->data, req->buf, req-> actual);//返回数据拷贝到req->buf中，                                                                                                     //dev->data_size=req->length; 
				dev->data_size=req->actual; //实际长度为req-> actual；需要确认req –>short_not_ok为0。参考gadget.h中关于usb_request结构的注释
		} 
		break;
		 /* this endpoint is normally active while we're configured */
		case -ECONNABORTED: /* hardware forced ep reset */
		case -ECONNRESET: /* request dequeued */
		case -ESHUTDOWN: /* disconnect from host */
		printk("%s gone (%d), %d/%d\n", ep->name, status,
						  req->actual, req->length);
		break;
		case -EOVERFLOW: /* buffer overrun on read means that
						  * we didn't provide a big enough
						  * buffer.
						  */
		default:
		#if 1
		printk("%s complete --> %d, %d/%d\n", ep->name,
						  status, req->actual, req->length);
		#endif
		break;
		case -EREMOTEIO: /* short read */
			break;
	}
	free_ep_req(ep, req);
	wake_up_interruptible (&dev->bulkrq); //唤醒读函数
}

static struct usb_request *source_sink_start_ep(struct usb_ep *ep)//构造并发送读请求
{
	struct usb_request *req;
	int status;
	//printk("in %s\n",__FUNCTION__);
	req = alloc_ep_req(ep, 128);
	if (!req)
		return NULL;
	memset(req->buf, 0, req->length);
	req->complete = source_sink_complete; //请求完成函数
	status = usb_ep_queue(ep, req, GFP_ATOMIC); //递交请求
	if (status) {
		//struct zero_dev *dev = ep->driver_data;
		printk("start %s --> %d\n", ep->name, status);
		free_ep_req(ep, req);
		req = NULL;
	}
	return req;
}
ssize_t usb_zero_read (struct file * file,  char __user * buf, size_t count,loff_t * f_pos) //读设备
{
	struct zero_dev *dev =file->private_data;
	//struct usb_request *req;
	//int status;
	struct usb_ep *ep;
	char *pbuf;
	//struct usb_gadget *gadget = dev->gadget;
	ssize_t ret = 0;
	//int result;
	ep=dev->out_ep;
	source_sink_start_ep(ep);//构造、递交读请求
	if (count < 0)
		return -EINVAL;
	printk("[%s, %d] %d\n", __FUNCTION__,__LINE__, dev->data_size);
	interruptible_sleep_on (&dev->bulkrq);//睡眠，等到请求完成
	pbuf = (char*)dev->data;
	printk("buf %02x, %02x\n", pbuf[0],pbuf[1]);
	if (copy_to_user ((void*)buf,dev->data,dev->data_size)) //拷贝读取的数据到用户空间
	{
		ret = -EFAULT;
	}
	else
	{
		ret = dev->data_size;
	}
	//printk("[%s, %d]\n", __FUNCTION__,__LINE__);
	return ret;
}

struct file_operations usb_zero_fops = {
	.owner = THIS_MODULE,
	.read = usb_zero_read,
	.open = usb_zero_open,
	.release = usb_zero_release,
};

static void usb_zero_setup_cdev(struct zero_dev *dev, int minor)//注册字符设备驱动
{
	int err, devno = MKDEV (usb_zero_major, minor);

	cdev_init(&dev->cdev, &usb_zero_fops);
	dev->cdev.owner = THIS_MODULE;
	err = cdev_add (&dev->cdev, devno, 1);
	if (err)
		printk ("Error adding usb_rcv\n");
}

static void zero_setup_complete(struct usb_ep *ep, struct usb_request *req)//配置端点0的请求完成处理
{
	if (req->status || req->actual != req->length)
		printk("setup complete --> %d, %d/%d\n",req->status, req->actual, req->length);
}
static void zero_reset_config(struct zero_dev *dev) //复位配置
{
	usb_ep_disable(dev->out_ep);
	dev->out_ep = NULL;
}
static void zero_disconnect(struct usb_gadget *gadget)//卸载驱动时被调用，做一些注销工作
{
	struct zero_dev *dev = get_gadget_data(gadget);
	printk("dev 0x%x gadget 0x%x\n",dev, gadget);
	#if 0
	unregister_chrdev_region(MKDEV (usb_zero_major, 0), 1);
	cdev_del (&(dev->cdev));
	zero_reset_config(dev);
	printk("in %s\n",__FUNCTION__);
	#endif
	//raise_exception(fsg, FSG_STATE_DISCONNECT);
}

static int config_buf(struct usb_gadget *gadget,u8 *buf, u8 type, unsigned index)
{
	//int is_source_sink;
	int len;
	const struct usb_descriptor_header **function;
	//int hs = 0;
	function = fs_loopback_function;//根据fs_loopback_function，得到长度，
								   //此处len=配置（9）+1个接口（9）+1个端点（7）=25
	len = usb_gadget_config_buf(&loopback_config, buf, USB_BUFSIZ, function);
	if (len < 0)
		return len;
	((struct usb_config_descriptor *) buf)->bDescriptorType = type;
	return len;
}

static int set_loopback_config(struct zero_dev *dev)
{
	int result = 0;
	struct usb_ep *ep;
	//struct usb_gadget *gadget = dev->gadget;
	const struct usb_endpoint_descriptor *d;

	ep=dev->out_ep;
	d = &fs_sink_desc;
	result = usb_ep_enable(ep, d); //激活端点
	//printk("");
	if (result == 0) {
		printk("connected\n"); //如果成功，打印“connected”
	} 
	else
		printk("can't enable %s, result %d\n", ep->name, result);
	return result;
}
static int zero_set_config(struct zero_dev *dev, unsigned number)
{
	int result = 0;
	struct usb_gadget *gadget = dev->gadget;
	result = set_loopback_config(dev);//激活设备
	if (result)
		zero_reset_config(dev); //复位设备
	else {
		char *speed;

		switch (gadget->speed) {
			case USB_SPEED_LOW:  speed = "low"; break;
			case USB_SPEED_FULL: speed = "full"; break;
			case USB_SPEED_HIGH: speed = "high"; break;
			default: speed = " "; break;
			}
	}
	return result;
}
/*
 * zero_setup完成USB设置阶段和具体功能相关的交互部分
 */
static int zero_setup(struct usb_gadget *gadget, const struct usb_ctrlrequest *ctrl)
{
	struct zero_dev *dev = get_gadget_data(gadget);
	struct usb_request *req = dev->req;
	int value = -EOPNOTSUPP;
	u16 w_index = le16_to_cpu(ctrl->wIndex);
	u16 w_value = le16_to_cpu(ctrl->wValue);
	u16 w_length = le16_to_cpu(ctrl->wLength);

	/* usually this stores reply data in the pre-allocated ep0 buffer,
     * but config change events will reconfigure hardware.
     */
	req->zero = 0;

	switch (ctrl->bRequest) {
		case USB_REQ_GET_DESCRIPTOR: //获取描述符
			if (ctrl->bRequestType != USB_DIR_IN)
				goto unknown;
			switch (w_value >> 8) {
				case USB_DT_DEVICE: //获取设备描述符
					value = min(w_length, (u16) sizeof device_desc);
					memcpy(req->buf, &device_desc, value);
					break;
				case USB_DT_CONFIG: //获取配置，注意：会根据fs_loopback_function读取到接口、端点描述符，注意通过config_buf完成读取数据及数量的统计。
					value = config_buf(gadget, req->buf,w_value >> 8,w_value & 0xff);
					if (value >= 0)
						value = min(w_length, (u16) value);
					break;

				case USB_DT_STRING:
					value = usb_gadget_get_string(&stringtab,w_value & 0xff, req->buf);
					if (value >= 0)
						value = min(w_length, (u16) value);
					break;
			}
			break;
		case USB_REQ_SET_CONFIGURATION:
			if (ctrl->bRequestType != 0)
				goto unknown;
			spin_lock(&dev->lock);
			value = zero_set_config(dev, w_value);//激活相应的端点
			spin_unlock(&dev->lock);
			break;

        default:
		unknown:
		printk("unknown control req%02x.%02x v%04x i%04x l%d\n",
				ctrl->bRequestType, ctrl->bRequest,
				w_value, w_index, w_length);
	  }
	  /* respond with data transfer before status phase */
	  if (value >= 0) {
			req->length = value;
			req->zero = value < w_length;
			value = usb_ep_queue(gadget->ep0, req, GFP_ATOMIC);//通过端点0完成setup
			if (value < 0) {
				printk("ep_queue --> %d\n", value);
				req->status = 0;
				zero_setup_complete(gadget->ep0, req);
			}
	  }
	  printk("%s ctrl->bRequest %d\n", __FUNCTION__, ctrl->bRequest);
	  /* device either stalls (value < 0) or reports success */
	  return value;
}
static void zero_unbind(struct usb_gadget *gadget) //解除绑定
{
	struct zero_dev *dev = get_gadget_data(gadget);

	printk("unbind\n");
	unregister_chrdev_region (MKDEV (usb_zero_major, 0), 1);
	cdev_del (&(dev->cdev));
	/* we've already been disconnected ... no i/o is active */
	if (dev->req) {
		dev->req->length = USB_BUFSIZ;
		free_ep_req(gadget->ep0, dev->req);
	}
	kfree(dev);
	set_gadget_data(gadget, NULL);
}
static int __init zero_bind(struct usb_gadget *gadget) //绑定过程 
{
	struct zero_dev *dev;
	struct usb_ep *ep;
	int gcnum;
	usb_ep_autoconfig_reset(gadget);
	ep = usb_ep_autoconfig(gadget, &fs_sink_desc);//根据端点描述符及控制器端点情况，分配一个合适的端点。
	if (!ep)
		goto enomem;
	EP_OUT_NAME = ep->name; //记录名称
	gcnum = usb_gadget_controller_number(gadget);//获得控制器代号
	if (gcnum >= 0)
		device_desc.bcdDevice = cpu_to_le16(0x0200 + gcnum);//赋值设备描述符
	else {
		pr_warning("%s: controller '%s' not recognized\n", shortname, gadget->name);
		device_desc.bcdDevice = __constant_cpu_to_le16(0x9999);
	}
	dev = kzalloc(sizeof(*dev), GFP_KERNEL); //分配设备结构体
	if (!dev)
		return -ENOMEM;
	spin_lock_init(&dev->lock);
	dev->gadget = gadget;
	set_gadget_data(gadget, dev);
	dev->req = usb_ep_alloc_request(gadget->ep0, GFP_KERNEL);//分配一个请求
	if (!dev->req)
		goto enomem;
	dev->req->buf = kmalloc(USB_BUFSIZ, GFP_KERNEL);
	if (!dev->req->buf)
		goto enomem;
	dev->req->complete = zero_setup_complete;
	dev->out_ep=ep; //记录端点（就是接收host端数据的端点）
	printk("name=%s\n",dev->out_ep->name); //打印出这个端点的名称
	ep->driver_data=dev;
	device_desc.bMaxPacketSize0 = gadget->ep0->maxpacket;
	usb_gadget_set_selfpowered(gadget);
	gadget->ep0->driver_data = dev;
	snprintf(manufacturer, sizeof manufacturer, "%s %s with %s",
			init_utsname()->sysname, init_utsname()->release,gadget->name);
	
	/**************************字符设备注册*******************/
	dev_t usb_zero_dev = MKDEV(usb_zero_major, 0); 
	int result = register_chrdev_region(usb_zero_dev, 1, "usb_zero");
	if (result < 0)
	{
		printk (KERN_NOTICE "Unable to register_chrdev_region, error %d\n",result);
		return result;
	}
	usb_zero_setup_cdev (dev, 0); 
	printk("%s\n", __FUNCTION__);
	return 0;
enomem:
	zero_unbind(gadget);
	return -ENOMEM;
}
/*-------------------------------------------------------------------------*/
static struct usb_gadget_driver zero_driver = { //gadget驱动的核心数据结构
#ifdef CONFIG_USB_GADGET_DUALSPEED
	.speed = USB_SPEED_HIGH,
#else
	.speed = USB_SPEED_FULL,
#endif
	.function   = (char *) longname,
	.bind       = zero_bind,
	.unbind     = __exit_p(zero_unbind),
	.setup      = zero_setup,
	.disconnect = zero_disconnect,
	//.suspend = zero_suspend, //不考虑电源管理的功能 
	//.resume = zero_resume,
	.driver = {
		.name = (char *) shortname,
		.owner = THIS_MODULE,
	},
};
MODULE_AUTHOR("David Brownell");
MODULE_LICENSE("GPL");
static int __init init(void)
{
	return usb_gadget_register_driver(&zero_driver); //注册驱动，调用bind绑定到控制器 
}
module_init(init);

static void __exit cleanup(void)
{
	usb_gadget_unregister_driver(&zero_driver); //注销驱动，通常会调用到unbind解除绑定， //在s3c2410_udc.c中调用的是disconnect方法 
}
module_exit(cleanup);
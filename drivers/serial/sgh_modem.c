/**
 * Samsung SGH-i900 GSM/UMTS modem support
 *
 * Copyright (C) 2009 Stefan Schmidt <stefan@datenfreihafen.org>
 *
 * Based on glofiish_modem.c (C) 2008 by Harald Welte <laforge@gnumonks.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

 /**
  * MSM6281 modem connected to a 16kbyte dual port ram chip and a gpio line for
  * power enable and disable.
  */

#define DEBUG

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/serial_core.h>
#include <linux/serial.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>

#include <asm/io.h>
#include <asm/irq.h>
#include <asm/dma.h>

#include <plat/regs-spi.h>
#include <plat/regs-dma.h>

#include <mach/dma.h>
#include <mach/spi.h>
#include <mach/regs-gpio.h>
#include <mach/regs-clock.h>
#include <mach/hardware.h>

#include <linux/delay.h>

struct sgh_modem {
	struct platform_device *pdev;
	unsigned int gpio_power;

	struct resource *ioarea;
	void __iomem *regs;

	unsigned char rx_buf[1024];
	unsigned int rx_len;
	unsigned int rx_idx;

	unsigned char tx_buf[1024];
	unsigned int tx_len;
	unsigned int tx_idx;

	struct uart_port port;
};

#define port_to_modem(x)	container_of((x), struct sgh_modem, port)

/* TTY layer functions  */

static void sgh_modem_pm(struct uart_port *port,
			   unsigned int level, unsigned int old)
{
	struct sgh_modem *sgh = port_to_modem(port);

	dev_dbg(&sgh->pdev->dev, "modem_pm(%u, %u)\n", level, old);

}

static unsigned int sgh_modem_tx_empty(struct uart_port *port)
{
	struct sgh_modem *sgh = port_to_modem(port);

	dev_dbg(&sgh->pdev->dev, "tx_empty()=1\n");
	return 1;
}

static unsigned int sgh_modem_get_mctrl(struct uart_port *port)
{
	struct sgh_modem *sgh = port_to_modem(port);
	dev_dbg(&sgh->pdev->dev, "get_mctrl()\n");
	/* FIXME: should we emulate RTS/CTS ? */
	return TIOCM_CAR | TIOCM_DSR | TIOCM_CTS;
}

static void sgh_modem_set_mctrl(struct uart_port *port,
				  unsigned int mctrl)
{
	struct sgh_modem *sgh = port_to_modem(port);
	dev_dbg(&sgh->pdev->dev, "set_mctrl()\n");
	/* FIXME: should we emulate RTS/CTS ? */
}

static void sgh_modem_stop_tx(struct uart_port *port)
{
	struct sgh_modem *sgh = port_to_modem(port);
	dev_dbg(&sgh->pdev->dev, "stop_tx\n");
}

static void sgh_modem_start_tx(struct uart_port *port)
{
	struct sgh_modem *sgh = port_to_modem(port);
	dev_dbg(&sgh->pdev->dev, "start_tx\n");
}

static void sgh_modem_stop_rx(struct uart_port *port)
{
	struct sgh_modem *sgh = port_to_modem(port);
	dev_dbg(&sgh->pdev->dev, "stop_rx\n");
	/* FIXME: what should we do? */
}

static void sgh_modem_enable_ms(struct uart_port *port)
{
	struct sgh_modem *sgh = port_to_modem(port);
	dev_dbg(&sgh->pdev->dev, "enable_ms()\n");
	/* we don't support this. just ignore it */
}

static void sgh_modem_break_ctl(struct uart_port *port, int ctl)
{
	struct sgh_modem *sgh = port_to_modem(port);
	dev_dbg(&sgh->pdev->dev, "break_ctl()\n");
	/* we don't support this. just ignore it */
}

static int sgh_modem_startup(struct uart_port *port)
{
	struct sgh_modem *sgh = port_to_modem(port);
	dev_dbg(&sgh->pdev->dev, "startup()\n");

	return 0;
}

static void sgh_modem_shutdown(struct uart_port *port)
{
	struct sgh_modem *sgh = port_to_modem(port);
	dev_dbg(&sgh->pdev->dev, "shutdown()\n");

	sgh_modem_stop_rx(port);
	sgh_modem_stop_tx(port);
}

static void sgh_modem_set_termios(struct uart_port *port,
				    struct ktermios *termios,
				    struct ktermios *old)
{
	struct sgh_modem *sgh = port_to_modem(port);
	dev_dbg(&sgh->pdev->dev, "set_termios()\n");

	/* we don't support mode mcontrol lines */
	termios->c_cflag &= ~(HUPCL | CMSPAR);
	termios->c_cflag |= CLOCAL;

	//uart_update_timeout(port, termios->c_flag, baud)
}

static const char *sgh_modem_type(struct uart_port *port)
{
	return "sgh-msm6281";
}

static int sgh_modem_request_port(struct uart_port *port)
{
	struct sgh_modem *sgh = port_to_modem(port);
	dev_dbg(&sgh->pdev->dev, "request_port()=-EINVAL\n");
	return -EINVAL;
}

static void sgh_modem_release_port(struct uart_port *port)
{
	struct sgh_modem *sgh = port_to_modem(port);
	dev_dbg(&sgh->pdev->dev, "release_port()\n");
}

static void sgh_modem_config_port(struct uart_port *port, int x)
{
	struct sgh_modem *sgh = port_to_modem(port);
	dev_dbg(&sgh->pdev->dev, "config_port()\n");
}

static int sgh_modem_verify_port(struct uart_port *port,
				   struct serial_struct *ser)
{
	struct sgh_modem *sgh = port_to_modem(port);
	dev_dbg(&sgh->pdev->dev, "verify_port()\n");
	if (ser->type != PORT_UNKNOWN && ser->type != PORT_GFISH)
		return -EINVAL;

	return 0;
}

static struct uart_ops sgh_uart_ops = {
	.pm				= sgh_modem_pm,
	.tx_empty		= sgh_modem_tx_empty,
	.get_mctrl		= sgh_modem_get_mctrl,
	.set_mctrl		= sgh_modem_set_mctrl,
	.stop_tx		= sgh_modem_stop_tx,
	.start_tx		= sgh_modem_start_tx,
	.stop_rx		= sgh_modem_stop_rx,
	.enable_ms		= sgh_modem_enable_ms,
	.break_ctl		= sgh_modem_break_ctl,
	.startup		= sgh_modem_startup,
	.shutdown		= sgh_modem_shutdown,
	.set_termios	= sgh_modem_set_termios,
	.type			= sgh_modem_type,
	.release_port	= sgh_modem_release_port,
	.request_port	= sgh_modem_request_port,
	.config_port	= sgh_modem_config_port,
	.verify_port	= sgh_modem_verify_port,
};

static struct uart_driver sgh_uart_driver = {
	.owner		= THIS_MODULE,
	.dev_name	= "sgh_modem",
	.nr		= 1,
	.driver_name	= "ttySGH",
};

static void hexdump(unsigned char *data, int len)
{
	int i;
	for (i = 0; i < len; i++)
		printk("%02x ", data[i]);
	printk("\n");
}

/* test command: transmit ATZ and wait for response */
const char atz[] = { 'A', 'T', 'Z', '\r', 0xff, 0xff };
static void tx_atz(struct sgh_modem *sgh)
{
	/* put ATZ command in transmit buffer */
	memcpy(sgh->tx_buf, atz, sizeof(atz));
}

static int __init sgh_probe(struct platform_device *pdev)
{
	int rc;
	struct sgh_modem *sgh;
	struct resource *res;

	sgh = kzalloc(sizeof(*sgh), GFP_KERNEL);
	if (!sgh)
		return -ENOMEM;

	sgh->pdev = pdev;

	sgh->gpio_power = platform_get_irq(pdev, 0);
	platform_set_drvdata(pdev, sgh);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "Cannot get IORESOURCE_MEM\n");
		rc = -ENOENT;
		goto out_free;
	}

	sgh->ioarea = request_mem_region(res->start,
					    (res->end - res->start)+1,
					    pdev->name);
	if (!sgh->ioarea) {
		dev_err(&pdev->dev, "Cannot reserve region\n");
		rc = -ENXIO;
		goto out_free;
	}

	sgh->regs = ioremap(res->start, (res->end - res->start)+1);
	if (!sgh->regs) {
		dev_err(&pdev->dev, "Cannot map IO\n");
		rc = -ENXIO;
		goto out_reqmem;
	}

	rc = uart_register_driver(&sgh_uart_driver);
	if (rc < 0) {
		dev_err(&pdev->dev, "Cannot register UART driver\n");
		goto out_clk;
	}

	sgh->port.dev = &pdev->dev;
	sgh->port.irq = sgh->irq;
	sgh->port.type = PORT_SGH;
	sgh->port.membase = sgh->ioarea->start;
	sgh->port.fifosize = 1;
	sgh->port.ops = &sgh_uart_ops;
	rc = uart_add_one_port(&sgh_uart_driver, &sgh->port);
	if (rc < 0) {
		dev_err(&pdev->dev, "Cannot add UART port\n");
		goto out_driver;
	}

	rc = request_irq(sgh->irq, sgh_modem_irq,
			 IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING,
			 pdev->name, sgh);
	if (rc) {
		dev_err(&pdev->dev, "Cannot claim IRQ\n");
		goto out_port;
	}
	enable_irq_wake(sgh->irq);

	rc = request_irq(sgh->irq, s3c24xx_spi_irq, 0,
			 "sgh_modem SPI slave", sgh);
	if (rc) {
		dev_err(&pdev->dev, "Cannot claim IRQ\n");
		goto out_irq;
	}

	rc = device_create_file(&pdev->dev, &dev_attr_tx_atz);
	if (rc < 0) {
		dev_err(&pdev->dev, "failed to add sysfs file\n");
		goto out_irq2;
	}

	return 0;

out_file:
	device_remove_file(&pdev->dev, &dev_attr_tx_atz);
out_irq2:
	free_irq(sgh->spi.irq, sgh);
out_irq:
	disable_irq_wake(sgh->irq);
	free_irq(sgh->irq, sgh);
out_port:
	uart_remove_one_port(&sgh_uart_driver, &sgh->port);
out_driver:
	uart_unregister_driver(&sgh_uart_driver);
out_clk:
	clk_put(sgh->spi.clk);
out_remap:
	iounmap(sgh->spi.regs);
out_reqmem:
	release_resource(sgh->spi.ioarea);
	kfree(sgh->spi.ioarea);
out_free:
	platform_set_drvdata(pdev, NULL);
	kfree(sgh);

	return rc;
}

static int sgh_remove(struct platform_device *pdev)
{
	struct sgh_modem *sgh = platform_get_drvdata(pdev);

	device_remove_file(&pdev->dev, &dev_attr_tx_atz);
	free_irq(sgh->irq, sgh);
	uart_remove_one_port(&sgh_uart_driver, &sgh->port);
	uart_unregister_driver(&sgh_uart_driver);
	iounmap(sgh->regs);
	release_resource(sgh->ioarea);
	kfree(sgh->ioarea);
	platform_set_drvdata(pdev, NULL);
	kfree(sgh);

	return 0;
}

static struct platform_driver sgh_modem_driver = {
	.probe		= sgh_probe,
	.remove		= sgh_remove,
	.driver		= {
		.name		= "sgh-modem",
	},
};

static int __devinit sgh_init(void)
{
	return platform_driver_register(&sgh_modem_driver);
}

static void sgh_exit(void)
{
	platform_driver_unregister(&sgh_modem_driver);
}

module_init(sgh_init);
module_exit(sgh_exit);

MODULE_LICENSE("GPLv2");
MODULE_AUTHOR("Stefan Schmidt <stefan@datenfreihafen.org>");
MODULE_DESCRIPTION("Samsung SGH-i900 GSM/UMTS modem driver");

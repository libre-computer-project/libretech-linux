// SPDX-License-Identifier: GPL-2.0+
/*
 * DRM driver for Ilitek ILI9488 panels
 *
 * Copyright 2025 Da Xue <da@libre.computer>
 *
 * Based on ili9486.c by Kamlesh Gurudasani
 *
 * The XH-RPI-35 and similar displays have a SPI-to-16-bit parallel bus
 * converter (74HC4094 shift registers) in front of the ILI9488 controller.
 * 8-bit SPI values are transferred as 16-bit words.
 */

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/property.h>
#include <linux/spi/spi.h>

#include <video/mipi_display.h>

#include <drm/clients/drm_client_setup.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_drv.h>
#include <drm/drm_fbdev_dma.h>
#include <drm/drm_gem_atomic_helper.h>
#include <drm/drm_gem_dma_helper.h>
#include <drm/drm_managed.h>
#include <drm/drm_mipi_dbi.h>
#include <drm/drm_modeset_helper.h>

#define ILI9488_MADCTL_BGR	BIT(3)
#define ILI9488_MADCTL_MV	BIT(5)
#define ILI9488_MADCTL_MX	BIT(6)
#define ILI9488_MADCTL_MY	BIT(7)

/*
 * The 74HC4094 shift registers convert SPI to a 16-bit parallel bus.
 * Every 16 SPI clocks produce one WR strobe with data on DB[15:0].
 * Commands and parameters: each byte sent as cpu_to_be16(byte) placing
 * the byte in DB[7:0] with DB[15:8]=0.
 * Pixel data: sent as native 16-bit RGB565 words at bpw=16.
 */
static int ili9488_command(struct mipi_dbi *mipi, u8 *cmd, u8 *par,
			   size_t num)
{
	struct spi_device *spi = mipi->spi;
	unsigned int bpw = 8;
	void *data = par;
	u32 speed_hz;
	int i, ret;
	__be16 *buf;

	buf = kmalloc(32 * sizeof(u16), GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	/* Send command byte as 16-bit word */
	buf[0] = cpu_to_be16(*cmd);
	spi_bus_lock(spi->controller);
	gpiod_set_value_cansleep(mipi->dc, 0);
	speed_hz = mipi_dbi_spi_cmd_max_speed(spi, 2);
	ret = mipi_dbi_spi_transfer(spi, speed_hz, 8, buf, 2);
	spi_bus_unlock(spi->controller);
	if (ret || !num)
		goto free;

	/* Parameters (<=32 bytes): expand each byte to 16-bit word */
	if (num <= 32) {
		for (i = 0; i < num; i++)
			buf[i] = cpu_to_be16(par[i]);
		num *= 2;
		data = buf;
	}

	/* Pixel data: send as 16-bit words, SPI handles byte order */
	if (*cmd == MIPI_DCS_WRITE_MEMORY_START && !mipi->swap_bytes)
		bpw = 16;

	if (*cmd == MIPI_DCS_WRITE_MEMORY_START)
		pr_info("ili9488: WRITE_MEMORY_START num=%zu bpw=%u swap=%d\n",
			num, bpw, mipi->swap_bytes);

	spi_bus_lock(spi->controller);
	gpiod_set_value_cansleep(mipi->dc, 1);
	speed_hz = mipi_dbi_spi_cmd_max_speed(spi, num);
	ret = mipi_dbi_spi_transfer(spi, speed_hz, bpw, data, num);
	spi_bus_unlock(spi->controller);

	if (*cmd == MIPI_DCS_WRITE_MEMORY_START)
		pr_info("ili9488: transfer done ret=%d bpw=%u len=%zu\n",
			ret, bpw, num);
 free:
	kfree(buf);

	return ret;
}

static void ili9488_enable(struct drm_simple_display_pipe *pipe,
			   struct drm_crtc_state *crtc_state,
			   struct drm_plane_state *plane_state)
{
	struct mipi_dbi_dev *dbidev = drm_to_mipi_dbi_dev(pipe->crtc.dev);
	struct mipi_dbi *dbi = &dbidev->dbi;
	u8 addr_mode;
	int ret, idx;

	if (!drm_dev_enter(pipe->crtc.dev, &idx))
		return;

	DRM_DEBUG_KMS("\n");

	ret = mipi_dbi_poweron_conditional_reset(dbidev);
	if (ret < 0)
		goto out_exit;
	if (ret == 1)
		goto out_enable;

	/* Minimal init: rely on ILI9488 defaults after reset */
	mipi_dbi_command(dbi, MIPI_DCS_EXIT_SLEEP_MODE);
	msleep(120);

	mipi_dbi_command(dbi, MIPI_DCS_SET_PIXEL_FORMAT, 0x55);

	mipi_dbi_command(dbi, MIPI_DCS_SET_DISPLAY_ON);
	msleep(20);

 out_enable:
	switch (dbidev->rotation) {
	case 90:
		addr_mode = ILI9488_MADCTL_MY;
		break;
	case 180:
		addr_mode = ILI9488_MADCTL_MV;
		break;
	case 270:
		addr_mode = ILI9488_MADCTL_MX;
		break;
	default:
		addr_mode = ILI9488_MADCTL_MV | ILI9488_MADCTL_MY |
			ILI9488_MADCTL_MX;
		break;
	}
	addr_mode |= ILI9488_MADCTL_BGR;
	mipi_dbi_command(dbi, MIPI_DCS_SET_ADDRESS_MODE, addr_mode);
	mipi_dbi_enable_flush(dbidev, crtc_state, plane_state);
 out_exit:
	drm_dev_exit(idx);
}

static const struct drm_simple_display_pipe_funcs ili9488_pipe_funcs = {
	DRM_MIPI_DBI_SIMPLE_DISPLAY_PIPE_FUNCS(ili9488_enable),
};

static const struct drm_display_mode ili9488_mode_default = {
	DRM_SIMPLE_MODE(480, 320, 73, 49),
};

DEFINE_DRM_GEM_DMA_FOPS(ili9488_fops);

static const struct drm_driver ili9488_driver = {
	.driver_features	= DRIVER_GEM | DRIVER_MODESET | DRIVER_ATOMIC,
	.fops			= &ili9488_fops,
	DRM_GEM_DMA_DRIVER_OPS_VMAP,
	DRM_FBDEV_DMA_DRIVER_OPS,
	.debugfs_init		= mipi_dbi_debugfs_init,
	.name			= "ili9488",
	.desc			= "Ilitek ILI9488",
	.major			= 1,
	.minor			= 0,
};

static const struct of_device_id ili9488_of_match[] = {
	{ .compatible = "displaynex,dn9488ssr" },
	{},
};
MODULE_DEVICE_TABLE(of, ili9488_of_match);

static const struct spi_device_id ili9488_id[] = {
	{ "ili9488", 0 },
	{ "dn9488ssr", 0 },
	{ }
};
MODULE_DEVICE_TABLE(spi, ili9488_id);

static int ili9488_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct mipi_dbi_dev *dbidev;
	struct drm_device *drm;
	struct mipi_dbi *dbi;
	struct gpio_desc *dc;
	u32 rotation = 0;
	int ret;

	dbidev = devm_drm_dev_alloc(dev, &ili9488_driver,
				    struct mipi_dbi_dev, drm);
	if (IS_ERR(dbidev))
		return PTR_ERR(dbidev);

	dbi = &dbidev->dbi;
	drm = &dbidev->drm;

	dbi->reset = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(dbi->reset))
		return dev_err_probe(dev, PTR_ERR(dbi->reset), "Failed to get GPIO 'reset'\n");

	dc = devm_gpiod_get(dev, "dc", GPIOD_OUT_LOW);
	if (IS_ERR(dc))
		return dev_err_probe(dev, PTR_ERR(dc), "Failed to get GPIO 'dc'\n");

	dbidev->backlight = devm_of_find_backlight(dev);
	if (IS_ERR(dbidev->backlight))
		return PTR_ERR(dbidev->backlight);

	device_property_read_u32(dev, "rotation", &rotation);

	ret = mipi_dbi_spi_init(spi, dbi, dc);
	if (ret)
		return ret;

	dbi->command = ili9488_command;
	dbi->read_commands = NULL;

	ret = mipi_dbi_dev_init(dbidev, &ili9488_pipe_funcs,
				&ili9488_mode_default, rotation);
	if (ret)
		return ret;

	drm_mode_config_reset(drm);

	ret = drm_dev_register(drm, 0);
	if (ret)
		return ret;

	spi_set_drvdata(spi, drm);

	drm_client_setup(drm, NULL);

	return 0;
}

static void ili9488_remove(struct spi_device *spi)
{
	struct drm_device *drm = spi_get_drvdata(spi);

	drm_dev_unplug(drm);
	drm_atomic_helper_shutdown(drm);
}

static void ili9488_shutdown(struct spi_device *spi)
{
	drm_atomic_helper_shutdown(spi_get_drvdata(spi));
}

static struct spi_driver ili9488_spi_driver = {
	.driver = {
		.name = "ili9488",
		.of_match_table = ili9488_of_match,
	},
	.id_table = ili9488_id,
	.probe = ili9488_probe,
	.remove = ili9488_remove,
	.shutdown = ili9488_shutdown,
};
module_spi_driver(ili9488_spi_driver);

MODULE_DESCRIPTION("Ilitek ILI9488 DRM driver");
MODULE_AUTHOR("Da Xue <da@libre.computer>");
MODULE_LICENSE("GPL");

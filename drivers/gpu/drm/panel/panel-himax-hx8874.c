// SPDX-License-Identifier: GPL-2.0-only
/*
 * Himax HX8874 9.06" dual-DSI DSC command-mode OLED panel driver
 *
 * This panel is used by the Nubia/ZTE np05j (RedMagic Tablet 3 Pro, SM8750).
 * It is driven over two MIPI DSI links (controllers 0 and 1), 4 lanes each,
 * each link covering half of the native 2400x1504 image (1200x1504 per link).
 * The stream is VESA DSC 1.1 compressed (10bpc source, 8bpp compressed) and
 * the panel operates in command (not video) mode.
 *
 * Copyright (c) 2024 np05j port
 *
 * Modelled on:
 *   drivers/gpu/drm/panel/panel-novatek-nt36523.c  (dual-DSI wiring / probe)
 *   drivers/gpu/drm/panel/panel-novatek-nt37801.c  (drm_dsc_config + PPS)
 */

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/regulator/consumer.h>

#include <video/mipi_display.h>

#include <drm/display/drm_dsc.h>
#include <drm/display/drm_dsc_helper.h>
#include <drm/drm_connector.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>

/* Number of DSI links that are always present (the primary one). */
#define DSI_NUM_MIN		1
/* This panel is always dual-DSI. */
#define HX8874_DSI_NUM		2

/* Backlight limits (from downstream: brightness-max 0xfff, bl-min 25). */
#define HX8874_BL_MAX_BRIGHTNESS	4095
#define HX8874_BL_MIN_BRIGHTNESS	25
#define HX8874_BL_DEF_BRIGHTNESS	2047

struct hx8874 {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi[HX8874_DSI_NUM];
	struct drm_dsc_config dsc;

	struct gpio_desc *reset_gpio;
	/*
	 * Downstream (qvkino) exposes a single panel power rail
	 * "vdddqvkino" at 3.3V.  Most mainline OLED panel bindings name
	 * their primary rail "vddio", so that is what is requested here.
	 * If a board splits this into separate "vddio"/"vci" rails the
	 * DT can be adjusted and a regulator_bulk used instead.
	 */
	struct regulator *vddio;

	enum drm_panel_orientation orientation;
};

static inline struct hx8874 *to_hx8874(struct drm_panel *panel)
{
	return container_of(panel, struct hx8874, panel);
}

/*
 * Reset sequence (active low reset line, downstream
 * qcom,mdss-dsi-reset-sequence = <1 10>,<0 10>,<1 10>):
 *   de-assert (high) 10ms, assert (low) 10ms, de-assert (high) 10ms.
 *
 * The gpio is requested with GPIOD_OUT_HIGH (asserted / in reset) so the
 * logical values below are inverted with respect to the raw electrical
 * level; "1" here means asserted (reset held), "0" means released.
 */
static void hx8874_reset(struct hx8874 *ctx)
{
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	usleep_range(10000, 11000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	usleep_range(10000, 11000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	usleep_range(10000, 11000);
}

/*
 * Panel power-on init sequence, transcribed directly from the downstream
 * qcom,mdss-dsi-on-command block of the HX8874 9.06" panel (60Hz timing).
 *
 * The manufacturer configuration blob (command #1) is a 110 byte generic
 * long write that programs the DSC/timing parameters inside the panel.
 * All commands are issued to both DSI links.
 */
static int hx8874_on(struct hx8874 *ctx)
{
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = NULL };
	int i;

	/* 1) Manufacturer configuration blob (generic long write, 110 bytes). */
	static const u8 hx8874_cfg[] = {
		0x80, 0x90, 0x00, 0x81, 0xe7, 0x03, 0x00, 0x00,
		0x00, 0x0a, 0x11, 0x00, 0x00, 0xab, 0x30, 0x80,
		0x05, 0xe0, 0x09, 0x60, 0x00, 0x04, 0x04, 0xb0,
		0x04, 0xb0, 0x02, 0x00, 0x03, 0x58, 0x00, 0x20,
		0x00, 0x72, 0x00, 0x10, 0x00, 0x0c, 0x20, 0x00,
		0x0b, 0x71, 0x18, 0x00, 0x10, 0xf0, 0x07, 0x10,
		0x20, 0x00, 0x00, 0x0f, 0x0f, 0x33, 0x0e, 0x1c,
		0x2a, 0x38, 0x46, 0x54, 0x62, 0x69, 0x70, 0x77,
		0x79, 0x7b, 0x7d, 0x7e, 0x02, 0x02, 0x22, 0x00,
		0x2a, 0x40, 0x2a, 0xbe, 0x3a, 0xfc, 0x3a, 0xfa,
		0x3a, 0xf8, 0x3b, 0x38, 0x3b, 0x78, 0x3b, 0xb6,
		0x4b, 0xf6, 0x4c, 0x34, 0x4c, 0x74, 0x5c, 0x74,
		0x8c, 0xf4, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	};
	/* Column address 0..1199 (per link). */
	static const u8 hx8874_col_addr[] = { 0x2a, 0x00, 0x00, 0x04, 0xaf };
	/* Page address 0..1503. */
	static const u8 hx8874_page_addr[] = { 0x2b, 0x00, 0x00, 0x05, 0xdf };
	/* Post display-on manufacturer tuning writes. */
	static const u8 hx8874_post1[] = {
		0x80, 0x8c, 0x00, 0x80, 0xe7, 0xc8, 0x14, 0x01, 0x00 };
	static const u8 hx8874_post2[] = {
		0x80, 0x8c, 0x00, 0x80, 0xe7, 0x00, 0x00, 0x00, 0x00 };
	static const u8 hx8874_post3[] = {
		0x80, 0x94, 0x00, 0x80, 0xe7, 0x3c, 0x00, 0x01, 0x00 };
	static const u8 hx8874_post4[] = {
		0x80, 0x78, 0x00, 0xac, 0xe7, 0x3c, 0x78, 0x82, 0x90 };

	for (i = 0; i < HX8874_DSI_NUM; i++) {
		dsi_ctx.dsi = ctx->dsi[i];

		mipi_dsi_generic_write_multi(&dsi_ctx, hx8874_cfg,
					     ARRAY_SIZE(hx8874_cfg));

		/* 2) Exit sleep, wait 240ms. */
		mipi_dsi_dcs_exit_sleep_mode_multi(&dsi_ctx);
		mipi_dsi_msleep(&dsi_ctx, 240);

		/* 3) Set display brightness (0x51 16-bit = 0), wait 150ms. */
		mipi_dsi_dcs_write_seq_multi(&dsi_ctx,
					     MIPI_DCS_SET_DISPLAY_BRIGHTNESS,
					     0x00, 0x00);
		mipi_dsi_msleep(&dsi_ctx, 150);

		/* 4) Column address. 5) Page address. */
		mipi_dsi_dcs_write_buffer_multi(&dsi_ctx, hx8874_col_addr,
						ARRAY_SIZE(hx8874_col_addr));
		mipi_dsi_dcs_write_buffer_multi(&dsi_ctx, hx8874_page_addr,
						ARRAY_SIZE(hx8874_page_addr));

		/* 6) Display on. */
		mipi_dsi_dcs_set_display_on_multi(&dsi_ctx);

		/* 7-10) Manufacturer post-on tuning. */
		mipi_dsi_generic_write_multi(&dsi_ctx, hx8874_post1,
					     ARRAY_SIZE(hx8874_post1));
		mipi_dsi_generic_write_multi(&dsi_ctx, hx8874_post2,
					     ARRAY_SIZE(hx8874_post2));
		mipi_dsi_generic_write_multi(&dsi_ctx, hx8874_post3,
					     ARRAY_SIZE(hx8874_post3));
		mipi_dsi_generic_write_multi(&dsi_ctx, hx8874_post4,
					     ARRAY_SIZE(hx8874_post4));
	}

	return dsi_ctx.accum_err;
}

static int hx8874_prepare(struct drm_panel *panel)
{
	struct hx8874 *ctx = to_hx8874(panel);
	struct drm_dsc_picture_parameter_set pps;
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = NULL };
	int ret, i;

	ret = regulator_enable(ctx->vddio);
	if (ret) {
		dev_err(panel->dev, "failed to enable vddio regulator: %d\n", ret);
		return ret;
	}

	hx8874_reset(ctx);

	ret = hx8874_on(ctx);
	if (ret < 0) {
		dev_err(panel->dev, "failed to initialize panel: %d\n", ret);
		goto err;
	}

	/*
	 * Program the DSC picture parameter set and enable compression on
	 * both DSI links.  drm_dsc_pps_payload_pack() serialises the fully
	 * computed drm_dsc_config filled in during probe.
	 */
	drm_dsc_pps_payload_pack(&pps, &ctx->dsc);

	for (i = 0; i < HX8874_DSI_NUM; i++) {
		dsi_ctx.dsi = ctx->dsi[i];

		mipi_dsi_picture_parameter_set_multi(&dsi_ctx, &pps);
		mipi_dsi_compression_mode_multi(&dsi_ctx, true);
	}

	ret = dsi_ctx.accum_err;
	if (ret < 0) {
		dev_err(panel->dev, "failed to enable DSC: %d\n", ret);
		goto err;
	}

	msleep(28);

	return 0;

err:
	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	regulator_disable(ctx->vddio);

	return ret;
}

static int hx8874_disable(struct drm_panel *panel)
{
	struct hx8874 *ctx = to_hx8874(panel);
	int i;

	for (i = 0; i < HX8874_DSI_NUM; i++) {
		struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi[i] };

		/* Display off, wait 100ms. */
		mipi_dsi_dcs_set_display_off_multi(&dsi_ctx);
		mipi_dsi_msleep(&dsi_ctx, 100);

		/* Enter sleep, wait 100ms. */
		mipi_dsi_dcs_enter_sleep_mode_multi(&dsi_ctx);
		mipi_dsi_msleep(&dsi_ctx, 100);
	}

	return 0;
}

static int hx8874_unprepare(struct drm_panel *panel)
{
	struct hx8874 *ctx = to_hx8874(panel);

	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	regulator_disable(ctx->vddio);

	return 0;
}

/*
 * Per-link display modes (each DSI link drives 1200x1504).  All five
 * refresh rates share the same vertical timing (vfp=56, vsync=8, vbp=8)
 * and differ only in the horizontal porches.
 */
#define HX8874_MODE(hz, _hfp, _hsync, _hbp)					\
	{									\
		.clock = (1200 + (_hfp) + (_hsync) + (_hbp)) *			\
			 (1504 + 56 + 8 + 8) * (hz) / 1000,			\
		.hdisplay = 1200,						\
		.hsync_start = 1200 + (_hfp),					\
		.hsync_end = 1200 + (_hfp) + (_hsync),				\
		.htotal = 1200 + (_hfp) + (_hsync) + (_hbp),			\
		.vdisplay = 1504,						\
		.vsync_start = 1504 + 56,					\
		.vsync_end = 1504 + 56 + 8,					\
		.vtotal = 1504 + 56 + 8 + 8,					\
		.width_mm = 195,						\
		.height_mm = 123,						\
		.type = DRM_MODE_TYPE_DRIVER,					\
	}

static const struct drm_display_mode hx8874_modes[] = {
	/* 60Hz is the default/preferred mode. */
	HX8874_MODE(60,  312,   8, 320),
	HX8874_MODE(90,  152, 160,  12),
	HX8874_MODE(120,  72,   8,  80),
	HX8874_MODE(144,  32,   8,  40),
	HX8874_MODE(165,  10,   8,  12),
};

static int hx8874_get_modes(struct drm_panel *panel,
			    struct drm_connector *connector)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(hx8874_modes); i++) {
		struct drm_display_mode *mode;

		mode = drm_mode_duplicate(connector->dev, &hx8874_modes[i]);
		if (!mode)
			return -ENOMEM;

		mode->type = DRM_MODE_TYPE_DRIVER;
		if (i == 0)
			mode->type |= DRM_MODE_TYPE_PREFERRED;

		drm_mode_set_name(mode);
		drm_mode_probed_add(connector, mode);
	}

	connector->display_info.width_mm = hx8874_modes[0].width_mm;
	connector->display_info.height_mm = hx8874_modes[0].height_mm;
	connector->display_info.bpc = 10;

	return ARRAY_SIZE(hx8874_modes);
}

static enum drm_panel_orientation hx8874_get_orientation(struct drm_panel *panel)
{
	struct hx8874 *ctx = to_hx8874(panel);

	return ctx->orientation;
}

static const struct drm_panel_funcs hx8874_panel_funcs = {
	.disable = hx8874_disable,
	.prepare = hx8874_prepare,
	.unprepare = hx8874_unprepare,
	.get_modes = hx8874_get_modes,
	.get_orientation = hx8874_get_orientation,
};

/*
 * Backlight: DCS 16-bit brightness (0x51, 2 bytes).  The panel uses an
 * inverted DBV (downstream qcom,mdss-dsi-bl-inverted-dbv), so the value
 * written is (max - brightness).
 */
static int hx8874_bl_update_status(struct backlight_device *bl)
{
	struct mipi_dsi_device *dsi = bl_get_data(bl);
	u16 brightness = backlight_get_brightness(bl);
	int ret;

	/* Inverted DBV. */
	brightness = HX8874_BL_MAX_BRIGHTNESS - brightness;

	dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	ret = mipi_dsi_dcs_set_display_brightness_large(dsi, brightness);
	if (ret < 0)
		return ret;

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	return 0;
}

static const struct backlight_ops hx8874_bl_ops = {
	.update_status = hx8874_bl_update_status,
};

static struct backlight_device *hx8874_create_backlight(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	const struct backlight_properties props = {
		.type = BACKLIGHT_RAW,
		.brightness = HX8874_BL_DEF_BRIGHTNESS,
		.max_brightness = HX8874_BL_MAX_BRIGHTNESS,
		.scale = BACKLIGHT_SCALE_NON_LINEAR,
	};

	return devm_backlight_device_register(dev, dev_name(dev), dev, dsi,
					      &hx8874_bl_ops, &props);
}

/*
 * Fill in the DSC configuration.  Only the panel-specific parameters are set
 * here; the remaining rate-control parameters are computed by the DRM DSC
 * helpers (matching what the DSI host would otherwise derive):
 *
 *   DSC 1.1, slice 1200x4, 1 slice/line, 10bpc source, 8bpp compressed,
 *   block prediction enabled, RGB (non native 4:2:x).
 */
static int hx8874_dsc_setup(struct hx8874 *ctx)
{
	struct drm_dsc_config *dsc = &ctx->dsc;
	int ret;

	dsc->dsc_version_major = 1;
	dsc->dsc_version_minor = 1;

	dsc->slice_height = 4;
	dsc->slice_width = 1200;
	/* Per-link picture is 1200x1504; one slice per line. */
	dsc->slice_count = 1200 / dsc->slice_width;
	dsc->pic_width = 1200;
	dsc->pic_height = 1504;

	dsc->bits_per_component = 10;
	dsc->bits_per_pixel = 8 << 4; /* 8bpp, 4 fractional bits */
	dsc->block_pred_enable = true;

	/* RGB input, no native chroma subsampling. */
	dsc->convert_rgb = true;
	dsc->native_422 = false;
	dsc->native_420 = false;
	dsc->simple_422 = false;
	dsc->vbr_enable = false;
	dsc->line_buf_depth = dsc->bits_per_component + 1;

	/*
	 * Derive the remaining RC parameters.  DRM_DSC_1_1_PRE_SCR selects
	 * the legacy DSC 1.1 rate-control tables (scr_version 0), matching
	 * the downstream qcom,mdss-dsc-scr-version = <0x0>.
	 */
	drm_dsc_set_const_params(dsc);
	drm_dsc_set_rc_buf_thresh(dsc);

	ret = drm_dsc_setup_rc_params(dsc, DRM_DSC_1_1_PRE_SCR);
	if (ret)
		return ret;

	return drm_dsc_compute_rc_parameters(dsc);
}

static int hx8874_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct device_node *dsi1;
	struct mipi_dsi_host *dsi1_host;
	struct hx8874 *ctx;
	const struct mipi_dsi_device_info info = {
		.type = "hx8874-dsi1",
		.channel = 0,
		.node = NULL,
	};
	int i, ret;

	ctx = devm_kzalloc(dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	ctx->vddio = devm_regulator_get(dev, "vddio");
	if (IS_ERR(ctx->vddio))
		return dev_err_probe(dev, PTR_ERR(ctx->vddio),
				     "failed to get vddio regulator\n");

	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(ctx->reset_gpio),
				     "failed to get reset gpio\n");

	/* Register the secondary (DSI1) link. */
	dsi1 = of_graph_get_remote_node(dsi->dev.of_node, 1, -1);
	if (!dsi1) {
		dev_err(dev, "cannot get secondary DSI node\n");
		return -ENODEV;
	}

	dsi1_host = of_find_mipi_dsi_host_by_node(dsi1);
	of_node_put(dsi1);
	if (!dsi1_host)
		return dev_err_probe(dev, -EPROBE_DEFER,
				     "cannot get secondary DSI host\n");

	ctx->dsi[1] = devm_mipi_dsi_device_register_full(dev, dsi1_host, &info);
	if (IS_ERR(ctx->dsi[1]))
		return dev_err_probe(dev, PTR_ERR(ctx->dsi[1]),
				     "cannot get secondary DSI device\n");

	ctx->dsi[0] = dsi;
	mipi_dsi_set_drvdata(dsi, ctx);

	drm_panel_init(&ctx->panel, dev, &hx8874_panel_funcs,
		       DRM_MODE_CONNECTOR_DSI);

	ret = of_drm_get_panel_orientation(dev->of_node, &ctx->orientation);
	if (ret < 0)
		return dev_err_probe(dev, ret, "failed to get orientation\n");

	ctx->panel.prepare_prev_first = true;

	ctx->panel.backlight = hx8874_create_backlight(dsi);
	if (IS_ERR(ctx->panel.backlight))
		return dev_err_probe(dev, PTR_ERR(ctx->panel.backlight),
				     "failed to create backlight\n");

	ret = hx8874_dsc_setup(ctx);
	if (ret)
		return dev_err_probe(dev, ret, "failed to setup DSC\n");

	drm_panel_add(&ctx->panel);

	for (i = 0; i < HX8874_DSI_NUM; i++) {
		ctx->dsi[i]->lanes = 4;
		ctx->dsi[i]->format = MIPI_DSI_FMT_RGB888;
		ctx->dsi[i]->mode_flags = MIPI_DSI_MODE_VIDEO_BURST |
					  MIPI_DSI_CLOCK_NON_CONTINUOUS |
					  MIPI_DSI_MODE_LPM;

		/* The panel only ever runs with DSC enabled. */
		ctx->dsi[i]->dsc = &ctx->dsc;

		ret = devm_mipi_dsi_attach(dev, ctx->dsi[i]);
		if (ret < 0) {
			drm_panel_remove(&ctx->panel);
			return dev_err_probe(dev, ret,
					     "cannot attach to DSI%d host\n", i);
		}
	}

	return 0;
}

static void hx8874_remove(struct mipi_dsi_device *dsi)
{
	struct hx8874 *ctx = mipi_dsi_get_drvdata(dsi);

	drm_panel_remove(&ctx->panel);
}

static const struct of_device_id hx8874_of_match[] = {
	{ .compatible = "himax,hx8874" },
	{ }
};
MODULE_DEVICE_TABLE(of, hx8874_of_match);

static struct mipi_dsi_driver hx8874_driver = {
	.probe = hx8874_probe,
	.remove = hx8874_remove,
	.driver = {
		.name = "panel-himax-hx8874",
		.of_match_table = hx8874_of_match,
	},
};
module_mipi_dsi_driver(hx8874_driver);

MODULE_AUTHOR("np05j port");
MODULE_DESCRIPTION("DRM driver for Himax HX8874 dual-DSI DSC command-mode OLED panel");
MODULE_LICENSE("GPL");

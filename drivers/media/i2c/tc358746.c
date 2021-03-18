/*
 * tc358746 - Toshiba parallel to CSI-2 bridge
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/swab.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/of_graph.h>
#include <linux/videodev2.h>
#include <linux/workqueue.h>
#include <linux/v4l2-dv-timings.h>
#include <linux/hdmi.h>
#include <media/v4l2-dv-timings.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>
#include <media/i2c/tc358746.h>
#include <media/i2c/wu10cam.h>

#include "tc358746_regs.h"

static int debug;
module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "debug level (0-3)");

MODULE_DESCRIPTION("Toshiba TC358746 parallel to CSI-2 bridge driver");
MODULE_LICENSE("GPL");

#define I2C_MAX_XFER_SIZE  (128 + 2)

static const struct v4l2_dv_timings_cap tc358746_timings_cap = {
	.type = V4L2_DV_BT_656_1120,
	/* keep this initialization for compatibility with GCC < 4.4.6 */
	.reserved = { 0 },
	/* Pixel clock from REF_01 p. 20. Min/max height/width are unknown */
	V4L2_INIT_BT_TIMINGS(1, 10000, 1, 10000, 0, 165000000,
			V4L2_DV_BT_STD_CEA861 | V4L2_DV_BT_STD_DMT |
			V4L2_DV_BT_STD_GTF | V4L2_DV_BT_STD_CVT,
			V4L2_DV_BT_CAP_PROGRESSIVE |
			V4L2_DV_BT_CAP_REDUCED_BLANKING |
			V4L2_DV_BT_CAP_CUSTOM)
};

struct tc358746_state {
	struct tc358746_platform_data pdata;
	struct v4l2_fwnode_bus_mipi_csi2 bus;
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_ctrl_handler ctrl_hdl;
	struct i2c_client *i2c_client;
	struct i2c_client* i2c_client_fpga;
	/* CONFCTL is modified in ops and tc358746_hdmi_sys_int_handler */
	struct mutex confctl_mutex;

	/* controls */

	struct work_struct work_i2c_poll;

	struct v4l2_dv_timings timings;
	u32 mbus_fmt_code;
	u8 csi_lanes_in_use;

	struct gpio_desc *reset_gpio;

	bool controls_initialized;
	struct device* dev_folder;
};

static inline struct tc358746_state *to_state(struct v4l2_subdev *sd)
{
	return container_of(sd, struct tc358746_state, sd);
}

#define WU10CAM_GPIO_IN_REG	0x00
#define WU10CAM_VIDEO_SELECT_REG	0x00
#define WU10CAM_GPIO_OUT_REG	0x01

#define to_tc358746_sd(_ctrl) (&container_of(_ctrl->handler,		\
					    struct tc358746_state,	\
					    ctrl_hdl)->sd)

u16 myswab16(u16 x)
{
	return ((x >> 8) & 0xFF) | ((x << 8) & 0xFF00);
}

static int wu10cam_write_reg_device(
	struct tc358746_state* sensor,
	u8 dev_addr,
	u8 reg,
	u8 val)
{
	if (sensor->controls_initialized)
	{
		struct i2c_client* client = sensor->i2c_client_fpga;
		struct i2c_msg msg;
		u8 buf[2];
		int ret;

		buf[0] = reg;
		buf[1] = val;

		msg.addr = dev_addr;
		msg.flags = client->flags;
		msg.buf = buf;
		msg.len = sizeof(buf);

		/* must use i2c_transfer() here,
		 * i2c_smbus_write_byte_data() doesn't work
		 * */
		ret = i2c_transfer(client->adapter, &msg, 1);
		if (ret < 0) {
			dev_err(&client->dev, "%s: error: reg=%x, val=%x\n",
				__func__, reg, val);
			return ret;
		}

		return 0;
	}

	return -ENODEV;
}

static int wu10cam_read_reg_device(
	struct tc358746_state* sensor,
	u8 dev_addr,
	u8 reg,
	u8* val)
{
	if (sensor->controls_initialized)
	{
		struct i2c_client* client = sensor->i2c_client_fpga;

		/* must use i2c_smbus_read_byte_data() here,
		 * i2c_transfer() doesn't work
		 * */
		*val = i2c_smbus_read_byte_data(client, reg & 0xff);

		return 0;
	}

	return -ENODEV;
}

static int wu10cam_write_reg32_device(
	struct tc358746_state* sensor,
	u8 dev_addr,
	u16 reg,
	u32 val)
{
	if (sensor->controls_initialized)
	{
		struct i2c_client* client = sensor->i2c_client_fpga;
		struct i2c_msg msg;
		u8 regbuf[2];
		u8 buf[6];
		int ret;

		memcpy(&regbuf[0], &reg, 2);
		buf[0] = regbuf[1];
		buf[1] = regbuf[0];

		memcpy(&buf[2], &val, 4);

		msg.addr = dev_addr;
		msg.flags = client->flags;
		msg.buf = buf;
		msg.len = sizeof(buf);

		/* must use i2c_transfer() here,
		 * i2c_smbus_write_byte_data() doesn't work
		 * */
		ret = i2c_transfer(client->adapter, &msg, 1);
		if (ret < 0) {
			dev_err(&client->dev, "%s: error: reg=%x, val=%x\n",
				__func__, reg, val);
			return ret;
		}

		return 0;
	}

	return -ENODEV;
}

static int wu10cam_read_reg32_device(
	struct tc358746_state* sensor,
	u8 dev_addr,
	u16 reg,
	u32* val)
{
	if (sensor->controls_initialized)
	{
		struct i2c_client* client = sensor->i2c_client_fpga;
		struct i2c_msg msg[2];
		u8 bufreg[2];
		u8 buf[5];
		u8 buf2[5];
		int ret;

		memcpy(bufreg, &reg, 2);
		buf[0] = bufreg[1];
		buf[1] = bufreg[0];
		//buf[1] = val;

		msg[0].addr = dev_addr;
		msg[0].flags = client->flags;
		msg[0].buf = buf;
		msg[0].len = 2;
		msg[1].addr = dev_addr;
		msg[1].flags = client->flags | I2C_M_RD;
		msg[1].buf = buf2;
		msg[1].len = 4;

		/* must use i2c_transfer() here,
		 * i2c_smbus_write_byte_data() doesn't work
		 * */
		ret = i2c_transfer(client->adapter, &msg[0], 2);
		if (ret < 0) {
			dev_err(&client->dev, "%s: error: reg=%x, val=%x\n",
				__func__, reg, *val);
			return ret;
		}
		memcpy(val, &buf2[0], 4);

		return 0;
	}

	return -ENODEV;
}

static void i2c_rd(struct v4l2_subdev *sd, u16 reg, u8 *values, u32 n)
{
	struct tc358746_state *state = to_state(sd);
	struct i2c_client *client = state->i2c_client;
	int err;
	int i_retry_cnt = 0;
	u8 buf[2] = { reg >> 8, reg & 0xff };
	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 2,
			.buf = buf,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = n,
			.buf = values,
		},
	};

retry_read:
	err = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (err != ARRAY_SIZE(msgs))
	{
		v4l2_err(sd, "%s: reading register 0x%x from 0x%x failed, err=%d\n",
				__func__, reg, client->addr, err);
		++i_retry_cnt;
		if (i_retry_cnt < 5)
			goto retry_read;
	}
}

static void i2c_wr(struct v4l2_subdev *sd, u16 reg, u8 *values, u32 n)
{
	struct tc358746_state *state = to_state(sd);
	struct i2c_client *client = state->i2c_client;
	int err, i;
	struct i2c_msg msg;
	u8 data[I2C_MAX_XFER_SIZE];
	int n_retries = 0;

	if ((2 + n) > I2C_MAX_XFER_SIZE) {
		n = I2C_MAX_XFER_SIZE - 2;
		v4l2_warn(sd, "i2c wr reg=%04x: len=%d is too big!\n",
			  reg, 2 + n);
	}

	msg.addr = client->addr;
	msg.buf = data;
	msg.len = 2 + n;
	msg.flags = 0;

	data[0] = reg >> 8;
	data[1] = reg & 0xff;

	for (i = 0; i < n; i++)
		data[2 + i] = values[i];

retry:
	err = i2c_transfer(client->adapter, &msg, 1);
	if (err != 1) {
		v4l2_err(sd, "%s: writing register 0x%x from 0x%x failed err=%d\n",
				__func__, reg, client->addr, err);
		if (n_retries < 5)
		{
			++n_retries;
			v4l2_err(sd, "%s: retry\n",
				__func__);
			usleep_range(10000, 20000);
			goto retry;
		}
		return;
	}

	if (debug < 3)
		return;

	switch (n) {
	case 1:
		v4l2_info(sd, "I2C write 0x%04x = 0x%02x",
				reg, data[2]);
		break;
	case 2:
		v4l2_info(sd, "I2C write 0x%04x = 0x%02x%02x",
//				reg, data[3], data[2]);
		reg, data[2], data[3]);
		break;
	case 4:
		v4l2_info(sd, "I2C write 0x%04x = 0x%02x%02x%02x%02x",
				reg, data[5], data[4], data[3], data[2]);
		break;
	default:
		v4l2_info(sd, "I2C write %d bytes from address 0x%04x\n",
				n, reg);
	}
}

static noinline u32 i2c_rdreg(struct v4l2_subdev *sd, u16 reg, u32 n)
{
	__le32 val = 0;

	i2c_rd(sd, reg, (u8 __force *)&val, n);

	return le32_to_cpu(val);
}

static noinline void i2c_wrreg(struct v4l2_subdev *sd, u16 reg, u32 val, u32 n)
{
	__le32 raw = cpu_to_le32(val);

	i2c_wr(sd, reg, (u8 __force *)&raw, n);
}

static u16 i2c_rd16(struct v4l2_subdev *sd, u16 reg)
{
	u16 val = i2c_rdreg(sd, reg, 2);
	val = myswab16(val);	// dvm
	return val;
}

static void i2c_wr16(struct v4l2_subdev *sd, u16 reg, u16 val)
{
	val = myswab16(val);	// dvm
	i2c_wrreg(sd, reg, val, 2);
}

static u32 i2c_rd32(struct v4l2_subdev *sd, u16 reg)
{
	return i2c_rdreg(sd, reg, 4);
}

/* --------------- INIT --------------- */

static inline void enable_stream(struct v4l2_subdev *sd, bool enable)
{
}

/* --------------- CORE OPS --------------- */

static int tc358746_log_status(struct v4l2_subdev *sd)
{
	return 0;
}

static int tc358746_subscribe_event(struct v4l2_subdev *sd, struct v4l2_fh *fh,
				    struct v4l2_event_subscription *sub)
{
	switch (sub->type) {
	case V4L2_EVENT_SOURCE_CHANGE:
		return v4l2_src_change_event_subdev_subscribe(sd, fh, sub);
	case V4L2_EVENT_CTRL:
		return v4l2_ctrl_subdev_subscribe_event(sd, fh, sub);
	default:
		return -EINVAL;
	}
}

/* --------------- VIDEO OPS --------------- */

static int tc358746_g_input_status(struct v4l2_subdev *sd, u32 *status)
{
	*status = 0;
	v4l2_dbg(1, debug, sd, "%s: status = 0x%x\n", __func__, *status);

	return 0;
}

static int tc358746_s_dv_timings(struct v4l2_subdev *sd,
				 struct v4l2_dv_timings *timings)
{
	struct tc358746_state *state = to_state(sd);

	if (!timings)
		return -EINVAL;

	if (debug)
		v4l2_print_dv_timings(sd->name, "tc358746_s_dv_timings: ",
				timings, false);

	if (v4l2_match_dv_timings(&state->timings, timings, 0, false)) {
		v4l2_dbg(1, debug, sd, "%s: no change\n", __func__);
		return 0;
	}

	if (!v4l2_valid_dv_timings(timings,
				&tc358746_timings_cap, NULL, NULL)) {
		v4l2_dbg(1, debug, sd, "%s: timings out of range\n", __func__);
		return -ERANGE;
	}

	state->timings = *timings;

	enable_stream(sd, false);

	return 0;
}

static int tc358746_g_dv_timings(struct v4l2_subdev *sd,
				 struct v4l2_dv_timings *timings)
{
	struct tc358746_state *state = to_state(sd);

	*timings = state->timings;

	return 0;
}

static int tc358746_enum_dv_timings(struct v4l2_subdev *sd,
				    struct v4l2_enum_dv_timings *timings)
{
	if (timings->pad != 0)
		return -EINVAL;

	return v4l2_enum_dv_timings_cap(timings,
			&tc358746_timings_cap, NULL, NULL);
}

static int tc358746_query_dv_timings(struct v4l2_subdev *sd,
		struct v4l2_dv_timings *timings)
{
	if (debug)
		v4l2_print_dv_timings(sd->name, "tc358746_query_dv_timings: ",
				timings, false);

	if (!v4l2_valid_dv_timings(timings,
				&tc358746_timings_cap, NULL, NULL)) {
		v4l2_dbg(1, debug, sd, "%s: timings out of range\n", __func__);
		return -ERANGE;
	}

	return 0;
}

static int tc358746_dv_timings_cap(struct v4l2_subdev *sd,
		struct v4l2_dv_timings_cap *cap)
{
	if (cap->pad != 0)
		return -EINVAL;

	*cap = tc358746_timings_cap;

	return 0;
}

static int tc358746_g_mbus_config(struct v4l2_subdev *sd,
			     struct v4l2_mbus_config *cfg)
{
	cfg->type = V4L2_MBUS_CSI2;

//	cfg->flags = V4L2_MBUS_CSI2_CONTINUOUS_CLOCK;
	cfg->flags |= V4L2_MBUS_CSI2_1_LANE;

	return 0;
}

static int tc358746_s_stream(struct v4l2_subdev *sd, int enable)
{
	enable_stream(sd, enable);

	return 0;
}

/* --------------- PAD OPS --------------- */

static int tc358746_enum_mbus_code(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_mbus_code_enum *code)
{
	switch (code->index) {
	case 0:
		code->code = MEDIA_BUS_FMT_UYVY8_2X8;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int tc358746_get_fmt(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_format *format)
{
	struct tc358746_state *state = to_state(sd);
// dvm	u8 vi_rep = i2c_rd8(sd, VI_REP);
	u8 vi_rep = MASK_VOUT_COLOR_709_YCBCR_FULL;

	if (format->pad != 0)
		return -EINVAL;

	format->format.code = state->mbus_fmt_code;
	format->format.width = state->timings.bt.width;
	format->format.height = state->timings.bt.height;
	format->format.field = V4L2_FIELD_NONE;

	switch (vi_rep & MASK_VOUT_COLOR_SEL) {
	case MASK_VOUT_COLOR_RGB_FULL:
	case MASK_VOUT_COLOR_RGB_LIMITED:
		format->format.colorspace = V4L2_COLORSPACE_SRGB;
		break;
	case MASK_VOUT_COLOR_601_YCBCR_LIMITED:
	case MASK_VOUT_COLOR_601_YCBCR_FULL:
		format->format.colorspace = V4L2_COLORSPACE_SMPTE170M;
		break;
	case MASK_VOUT_COLOR_709_YCBCR_FULL:
	case MASK_VOUT_COLOR_709_YCBCR_LIMITED:
		format->format.colorspace = V4L2_COLORSPACE_REC709;
		break;
	default:
		format->format.colorspace = 0;
		break;
	}

	return 0;
}

static int tc358746_set_fmt(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_format *format)
{
	struct tc358746_state *state = to_state(sd);

	u32 code = format->format.code; /* is overwritten by get_fmt */
	int ret = tc358746_get_fmt(sd, cfg, format);

	format->format.code = code;

	if (ret)
		return ret;

	if (format->which == V4L2_SUBDEV_FORMAT_TRY)
		return 0;

	state->mbus_fmt_code = format->format.code;

	enable_stream(sd, false);

	return 0;
}

/* -------------------------------------------------------------------------- */

/*
 * Controls
 *
 * Handling them here allows us to do some video processing in FPGA,
 * as well as some in the ADV7280.
 *
 * Currently following are done by FPGA:
 * - none
 *
 * Following are done by 7280:
 * - brightness
 * - contrast
 * - hue
 * - saturation
 * - fast switching
 * - set input
 * - set pattern
 */

extern int adv7180_wu10_command(int command, int param1, int param2);

static int tc358746_s_ctrl(struct v4l2_ctrl* ctrl)
{
	struct v4l2_subdev* sd = to_tc358746_sd(ctrl);
	struct tc358746_state* state = to_state(sd);

	if (state->controls_initialized)
	{
		return adv7180_wu10_command(WU10_CMD_SET_CONTROL, ctrl->id, ctrl->val);
	}

	return 0;
}

static const struct v4l2_ctrl_ops tc358746_ctrl_ops = {
	.s_ctrl = tc358746_s_ctrl,
};

static const struct v4l2_ctrl_config tc358746_ctrl_fast_switch = {
	.ops = &tc358746_ctrl_ops,
	.id = V4L2_CID_ADV_FAST_SWITCH,
	.name = "fast_switching",
	.type = V4L2_CTRL_TYPE_BOOLEAN,
	.min = 0,
	.max = 1,
	.step = 1,
	.def = 0,
	.flags = V4L2_CTRL_FLAG_VOLATILE | V4L2_CTRL_FLAG_EXECUTE_ON_WRITE,
};

static const struct v4l2_ctrl_config tc358746_ctrl_set_input = {
	.ops = &tc358746_ctrl_ops,
	.id = V4L2_CID_ADV_SET_INPUT,
	.name = "set_input",
	.type = V4L2_CTRL_TYPE_INTEGER,
	.min = 0,
	.max = 15,
	.step = 1,
	.def = 0,
	.flags = V4L2_CTRL_FLAG_VOLATILE | V4L2_CTRL_FLAG_EXECUTE_ON_WRITE,
};

static const struct v4l2_ctrl_config tc358746_ctrl_set_pattern = {
	.ops = &tc358746_ctrl_ops,
	.id = V4L2_CID_ADV_SET_PATTERN,
	.name = "set_pattern",
	.type = V4L2_CTRL_TYPE_INTEGER,
	.min = 0,
	.max = 15,
	.step = 1,
	.def = 0,
	.flags = V4L2_CTRL_FLAG_VOLATILE | V4L2_CTRL_FLAG_EXECUTE_ON_WRITE,
};

/* Contrast */
#define ADV7180_CON_MIN		0
#define ADV7180_CON_DEF		128
#define ADV7180_CON_MAX		255
/* Brightness*/
#define ADV7180_BRI_MIN		-128
#define ADV7180_BRI_DEF		0
#define ADV7180_BRI_MAX		127
/* Hue */
#define ADV7180_HUE_MIN		-127
#define ADV7180_HUE_DEF		0
#define ADV7180_HUE_MAX		128
/* Saturation */
#define ADV7180_SAT_MIN		0
#define ADV7180_SAT_DEF		128
#define ADV7180_SAT_MAX		255

static int tc358746_init_controls(struct tc358746_state* sensor)
{
	v4l2_ctrl_handler_init(&sensor->ctrl_hdl, 7);

	v4l2_ctrl_new_std(&sensor->ctrl_hdl, &tc358746_ctrl_ops,
		V4L2_CID_BRIGHTNESS, ADV7180_BRI_MIN,
		ADV7180_BRI_MAX, 1, ADV7180_BRI_DEF);
	v4l2_ctrl_new_std(&sensor->ctrl_hdl, &tc358746_ctrl_ops,
		V4L2_CID_CONTRAST, ADV7180_CON_MIN,
		ADV7180_CON_MAX, 1, ADV7180_CON_DEF);
	v4l2_ctrl_new_std(&sensor->ctrl_hdl, &tc358746_ctrl_ops,
		V4L2_CID_SATURATION, ADV7180_SAT_MIN,
		ADV7180_SAT_MAX, 1, ADV7180_SAT_DEF);
	v4l2_ctrl_new_std(&sensor->ctrl_hdl, &tc358746_ctrl_ops,
		V4L2_CID_HUE, ADV7180_HUE_MIN,
		ADV7180_HUE_MAX, 1, ADV7180_HUE_DEF);
	v4l2_ctrl_new_custom(&sensor->ctrl_hdl, &tc358746_ctrl_fast_switch, NULL);
	v4l2_ctrl_new_custom(&sensor->ctrl_hdl, &tc358746_ctrl_set_input, NULL);
	v4l2_ctrl_new_custom(&sensor->ctrl_hdl, &tc358746_ctrl_set_pattern, NULL);

	sensor->sd.ctrl_handler = &sensor->ctrl_hdl;
	if (sensor->ctrl_hdl.error) {
		int err = sensor->ctrl_hdl.error;

		v4l2_ctrl_handler_free(&sensor->ctrl_hdl);
		return err;
	}
	v4l2_ctrl_handler_setup(&sensor->ctrl_hdl);

	return 0;
}

static void wu10cam_exit_controls(struct tc358746_state* sensor)
{
	v4l2_ctrl_handler_free(&sensor->ctrl_hdl);
}

/* -------------------------------------------------------------------------- */

static const struct v4l2_subdev_core_ops tc358746_core_ops = {
	.log_status = tc358746_log_status,
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register = tc358746_g_register,
	.s_register = tc358746_s_register,
#endif
	.interrupt_service_routine = NULL,
	.subscribe_event = tc358746_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
};

static const struct v4l2_subdev_video_ops tc358746_video_ops = {
	.g_input_status = tc358746_g_input_status,
	.s_dv_timings = tc358746_s_dv_timings,
	.g_dv_timings = tc358746_g_dv_timings,
	.query_dv_timings = tc358746_query_dv_timings,
	.g_mbus_config = tc358746_g_mbus_config,
	.s_stream = tc358746_s_stream,
};

static const struct v4l2_subdev_pad_ops tc358746_pad_ops = {
	.enum_mbus_code = tc358746_enum_mbus_code,
	.set_fmt = tc358746_set_fmt,
	.get_fmt = tc358746_get_fmt,
	.enum_dv_timings = tc358746_enum_dv_timings,
	.dv_timings_cap = tc358746_dv_timings_cap,
};

static const struct v4l2_subdev_ops tc358746_ops = {
	.core = &tc358746_core_ops,
	.video = &tc358746_video_ops,
	.pad = &tc358746_pad_ops,
};

/* --------------- PROBE / REMOVE --------------- */

#ifdef CONFIG_OF
static void tc358746_gpio_reset(struct tc358746_state *state)
{
	usleep_range(5000, 10000);
	gpiod_set_value(state->reset_gpio, 1);
	usleep_range(1000, 2000);
	gpiod_set_value(state->reset_gpio, 0);
	msleep(20);
}

static int tc358746_probe_of(struct tc358746_state *state)
{
	struct device *dev = &state->i2c_client->dev;
	struct v4l2_fwnode_endpoint *endpoint;
	struct device_node *ep;
	struct clk *refclk;
	u32 bps_pr_lane;
	int ret = -EINVAL;

	refclk = devm_clk_get(dev, "refclk");
	if (IS_ERR(refclk)) {
		if (PTR_ERR(refclk) != -EPROBE_DEFER)
			dev_err(dev, "failed to get refclk: %ld\n",
				PTR_ERR(refclk));
		return PTR_ERR(refclk);
	}

	ep = of_graph_get_next_endpoint(dev->of_node, NULL);
	if (!ep) {
		dev_err(dev, "missing endpoint node\n");
		return -EINVAL;
	}

	endpoint = v4l2_fwnode_endpoint_alloc_parse(of_fwnode_handle(ep));
	if (IS_ERR(endpoint)) {
		dev_err(dev, "failed to parse endpoint\n");
		return PTR_ERR(endpoint);
	}

	if (endpoint->bus_type != V4L2_MBUS_CSI2 ||
	    endpoint->bus.mipi_csi2.num_data_lanes == 0 ||
	    endpoint->nr_of_link_frequencies == 0) {
		dev_err(dev, "missing CSI-2 properties in endpoint\n");
		goto free_endpoint;
	}

	state->bus = endpoint->bus.mipi_csi2;

	ret = clk_prepare_enable(refclk);
	if (ret) {
		dev_err(dev, "Failed! to enable clock\n");
		goto free_endpoint;
	}

	/* A FIFO level of 16 should be enough for 2-lane 720p60 at 594 MHz. */
	state->pdata.fifo_level = 16;

	/*
	 * The CSI bps per lane must be between 62.5 Mbps and 1 Gbps.
	 * The default is 594 Mbps for 4-lane 1080p60 or 2-lane 720p60.
	 */
	bps_pr_lane = 2 * endpoint->link_frequencies[0];
	if (bps_pr_lane < 62500000U || bps_pr_lane > 1000000000U) {
		dev_err(dev, "unsupported bps per lane: %u bps\n", bps_pr_lane);
		goto disable_clk;
	}

	/*
	 * FIXME: These timings are from REF_02 for 594 Mbps per lane (297 MHz
	 * link frequency). In principle it should be possible to calculate
	 * them based on link frequency and resolution.
	 */
	if (bps_pr_lane != 594000000U)
		dev_warn(dev, "untested bps per lane: %u bps\n", bps_pr_lane);
	state->pdata.lineinitcnt = 0xe80;
	state->pdata.lptxtimecnt = 0x003;
	/* tclk-preparecnt: 3, tclk-zerocnt: 20 */
	state->pdata.tclk_headercnt = 0x1403;
	state->pdata.tclk_trailcnt = 0x00;
	/* ths-preparecnt: 3, ths-zerocnt: 1 */
	state->pdata.ths_headercnt = 0x0103;
	state->pdata.twakeup = 0x4882;
	state->pdata.tclk_postcnt = 0x008;
	state->pdata.ths_trailcnt = 0x2;
	state->pdata.hstxvregcnt = 0;

	state->reset_gpio = devm_gpiod_get_optional(dev, "reset",
						    GPIOD_OUT_LOW);
	if (IS_ERR(state->reset_gpio)) {
		dev_err(dev, "failed to get reset gpio\n");
		ret = PTR_ERR(state->reset_gpio);
		goto disable_clk;
	}

	if (state->reset_gpio)
		tc358746_gpio_reset(state);

	ret = 0;
	goto free_endpoint;

disable_clk:
	clk_disable_unprepare(refclk);
free_endpoint:
	v4l2_fwnode_endpoint_free(endpoint);
	return ret;
}
#else
static inline int tc358746_probe_of(struct tc358746_state *state)
{
	return -ENODEV;
}
#endif

struct v4l2_subdev* g_sd = NULL;

void i2c1_cplb_write16(u16 reg, u16 val)
{
	i2c_wr16(g_sd, reg, val);
}

void Waitx1us(int us)
{
	usleep_range(us, us << 1);
}

void tc358746_initialize_registers(void)
{
	// *********************************************
	// Start up sequence
	// *********************************************
	// **************************************************
	// TC358746(A)XBG Software Reset
	// **************************************************
	i2c1_cplb_write16(0x0002, 0x0001); // SYSctl, S/W Reset
	Waitx1us(10);
	i2c1_cplb_write16(0x0002, 0x0000); // SYSctl, S/W Reset release
	// **************************************************
	// TC358746(A)XBG PLL,Clock Setting
	// **************************************************
	i2c1_cplb_write16(0x0016, 0x0013); // PLL Control Register 0 (PLL_PRD,PLL_FBD)
	i2c1_cplb_write16(0x0018, 0x0603); // PLL_FRS,PLL_LBWS, PLL oscillation enable
	Waitx1us(1000);
	i2c1_cplb_write16(0x0018, 0x0613); // PLL_FRS,PLL_LBWS, PLL clock out enable
	// **************************************************
	// TC358746(A)XBG DPI Input Control
	// **************************************************
	i2c1_cplb_write16(0x0006, 0x0064); // FIFO Control Register
	i2c1_cplb_write16(0x0008, 0x0060); // Data Format setting
	i2c1_cplb_write16(0x0022, 0x0500); // Word Count
	// **************************************************
	// TC358746XBG MCLK Output
	// **************************************************
	// **************************************************
	// TC358746(A)XBG GPIO2,1 Control (Example)
	// **************************************************
	// **************************************************
	// TC358746(A)XBG D-PHY Setting
	// **************************************************
	i2c1_cplb_write16(0x0140, 0x0000); // D-PHY Clock lane enable
	i2c1_cplb_write16(0x0142, 0x0000); // 
	i2c1_cplb_write16(0x0144, 0x0000); // D-PHY Data lane0 enable
	i2c1_cplb_write16(0x0146, 0x0000); // 
	i2c1_cplb_write16(0x0148, 0x0001); // D-PHY Data lane1 enable
	i2c1_cplb_write16(0x014A, 0x0000); // 
	i2c1_cplb_write16(0x014C, 0x0001); // D-PHY Data lane2 enable
	i2c1_cplb_write16(0x014E, 0x0000); // 
	i2c1_cplb_write16(0x0150, 0x0001); // D-PHY Data lane3 enable
	i2c1_cplb_write16(0x0152, 0x0000); // 
	// **************************************************
	// TC358746(A)XBG CSI2-TX PPI Control
	// **************************************************
	i2c1_cplb_write16(0x0210, 0x07D0); // LINEINITCNT
	i2c1_cplb_write16(0x0212, 0x0000); // 
	i2c1_cplb_write16(0x0214, 0x0002); // LPTXTIMECNT
	i2c1_cplb_write16(0x0216, 0x0000); // 
	i2c1_cplb_write16(0x0218, 0x0801); // TCLK_HEADERCNT
	i2c1_cplb_write16(0x021A, 0x0000); // 
	i2c1_cplb_write16(0x021C, 0x0000); // TCLK_TRAILCNT
	i2c1_cplb_write16(0x021E, 0x0000); // 
	i2c1_cplb_write16(0x0220, 0x0001); // THS_HEADERCNT
	i2c1_cplb_write16(0x0222, 0x0000); // 
	i2c1_cplb_write16(0x0224, 0x3A98); // TWAKEUPCNT
	i2c1_cplb_write16(0x0226, 0x0000); // 
	i2c1_cplb_write16(0x0228, 0x0006); // TCLK_POSTCNT
	i2c1_cplb_write16(0x022A, 0x0000); // 
	i2c1_cplb_write16(0x022C, 0x0000); // THS_TRAILCNT
	i2c1_cplb_write16(0x022E, 0x0000); // 
	i2c1_cplb_write16(0x0230, 0x0005); // HSTXVREGCNT
	i2c1_cplb_write16(0x0232, 0x0000); // 
	i2c1_cplb_write16(0x0234, 0x0003); // HSTXVREGEN enable
	i2c1_cplb_write16(0x0236, 0x0000); // 
	i2c1_cplb_write16(0x0238, 0x0000); // DSI clock Enable/Disable during LP
	i2c1_cplb_write16(0x023A, 0x0000); // 
	i2c1_cplb_write16(0x0204, 0x0001); // STARTCNTRL
	i2c1_cplb_write16(0x0206, 0x0000); // 
	i2c1_cplb_write16(0x0518, 0x0001); // CSI Start
	i2c1_cplb_write16(0x051A, 0x0000); // 
	// **************************************************
	// Set to HS mode
	// **************************************************
	i2c1_cplb_write16(0x0500, 0x8081); // CSI2 lane setting, CSI2 mode=HS
	i2c1_cplb_write16(0x0502, 0xA300); // bit set
	i2c1_cplb_write16(0x0004, 0x0040); // Configuration Control Register
}

const struct reg_data
{
	u16 addr;
	char* name;
	u8 bits;
	char* comment;
} regs[] =
{
	{ 0x0000, "ChipID", 16, "TC358746AXBG / TC358748XBG / TC358748IXBG Chip and Revision ID" },
	{ 0x0002, "SysCtl", 16, "System Control Register" },
	{ 0x0004, "ConfCtl", 16, "Configuration Control Register" },
	{ 0x0006, "FiFoCtl", 16, "FiFo Control Register" },
	{ 0x0008, "DataFmt", 16, "Data Format Control Register" },
	{ 0x000C, "MclkCtl", 16, "Mclk control register" },
	{ 0x000E, "GPIOEn", 16, "GPIO Enable Control Register" },
	{ 0x0010, "GPIODir", 16, "GPIO Pin Direction Control Register" },
	{ 0x0012, "GPIOIn", 16, "GPIO Input Pin Value" },
	{ 0x0014, "GPIOOut", 16, "GPIO Output Pin Value" },
	{ 0x0016, "PLLCtl0", 16, "PLL control Register 0" },
	{ 0x0018, "PLLCtl1", 16, "PLL control Register 1" },
	{ 0x0020, "CLKCtrl", 16, "Clock Control Register" },
	{ 0x0022, "WordCnt", 16, "Word Count Register" },
	{ 0x0032, "PP_MISC", 16, "Parallel Input Port Miscellaneous Register" },
	{ 0x0050, "CSITX_DT", 16, "User Defined CSI Tx Data Type" },
	{ 0x0056, "PHYClkCtl", 16, "CSI2RX PHY clock control Register" },
	{ 0x0058, "PHYData0Ctl", 16, "CSI2RX PHY data 0 control Register" },
	{ 0x005A, "PHYData1Ctl", 16, "CSI2RX PHY data 1 control Register" },
	{ 0x005C, "PHYData2Ctl", 16, "CSI2RX PHY data 2 control Register" },
	{ 0x005E, "PHYData3Ctl", 16, "CSI2RX PHY data 3 control Register" },
	{ 0x0060, "PHYTimDly", 16, "CSI2RXPHY Time delay Register" },
	{ 0x0062, "PHYSta", 16, "CSI2RX PHY status Register" },
	{ 0x0064, "CSIStatus", 16, "CSI2RX Error status Register" },
	{ 0x0066, "CSIErrEn", 16, "CSI2RX Error Enable Register" },
	{ 0x0068, "MDLSynErr", 16, "CSI2RX Multi - Data Lane Sync Byte Detected Error Register" },
	{ 0x006A, "CSIDID", 16, "CSI2RX data Type ID Register" },
	{ 0x006C, "CSIDIDErr", 16, "CSI2RX Data Type ID Error Register" },
	{ 0x006E, "CSIPktLen", 16, "CSI2RX data length Register" },
	{ 0x0070, "CSIRX_DPCtl", 16, "CSI2RX Dphy control Register" },

	// CSI - 2 - RX Status Counters (16 - bit Address)
	{ 0x0080, "FrmErrCnt", 16, "CSI2RX Frame error counter" },
	{ 0x0082, "CRCErrCnt", 16, "CSI2RX CRC error counter" },
	{ 0x0084, "CorErrCnt", 16, "CSI2RX Recoverable Packet Header error counter" },
	{ 0x0086, "HdrErrCnt", 16, "CSI2RX Unrecoverable Packet Header error counter" },
	{ 0x0088, "EIDErrCnt", 16, "CSI2RX Unsupported Packet ID error counter" },
	{ 0x008A, "CtlErrCnt", 16, "CSI2RX Escape Mode error counter" },
	{ 0x008C, "SotErrCnt", 16, "CSI2RX Recoverable Sync Byte error counter" },
	{ 0x008E, "SynErrCnt", 16, "CSI2RX unrecoverable Sync Byte error counter" },
	{ 0x0090, "MDLErrCnt", 16, "CSI2RX Multi - Data Lane Sync Byte error counter" },
	{ 0x00F8, "FIFOStatus", 16, "FiFo Underflow / Overflow Status Register" },

	// TX PHY (32 - bit Address)
	{ 0x0100, "CLW_DPHYCONTTX", 16, "Clock Lane DPHY Tx Control Register" },
	{ 0x0102, "CLW_DPHYCONTTX", 16, "Clock Lane DPHY Tx Control Register2" },
	{ 0x0104, "D0W_DPHYCONTTX ", 16, "Data Lane0 DPHY Tx Control Register" },
	{ 0x0106, "D0W_DPHYCONTTX ", 16, "Data Lane0 DPHY Tx Control Register2" },
	{ 0x0108, "D1W_DPHYCONTTX ", 16, "Data Lane1 DPHY Tx Control Register" },
	{ 0x010A, "D1W_DPHYCONTTX ", 16, "Data Lane1 DPHY Tx Control Register2" },
	{ 0x010C, "D2W_DPHYCONTTX ", 16, "Data Lane2 DPHY Tx Control Register" },
	{ 0x010E, "D2W_DPHYCONTTX ", 16, "Data Lane2 DPHY Tx Control Register2" },
	{ 0x0110, "D3W_DPHYCONTTX ", 16, "Data Lane3 DPHY Tx Control Register TC358746AXBG / TC358748XBG / TC358748IXBG" },
	{ 0x0112, "D3W_DPHYCONTTX ", 16, "Data Lane3 DPHY Tx Control Register TC358746AXBG / TC358748XBG / TC358748IXBG2" },
	{ 0x0140, "CLW_CNTRL", 16, "Clock Lane DPHY Control Register" },
	{ 0x0142, "CLW_CNTRL", 16, "Clock Lane DPHY Control Register2" },
	{ 0x0144, "D0W_CNTRL", 16, "Data Lane 0 DPHY Control Register" },
	{ 0x0146, "D0W_CNTRL", 16, "Data Lane 0 DPHY Control Register2" },
	{ 0x0148, "D1W_CNTRL", 16, "Data Lane 1 DPHY Control Register" },
	{ 0x014A, "D1W_CNTRL", 16, "Data Lane 1 DPHY Control Register2" },
	{ 0x014C, "D2W_CNTRL", 16, "Data Lane 2 DPHY Control Register" },
	{ 0x014E, "D2W_CNTRL", 16, "Data Lane 2 DPHY Control Register2" },
	{ 0x0150, "D3W_CNTRL", 16, "Data Lane 3 DPHY Control Register" },
	{ 0x0152, "D3W_CNTRL", 16, "Data Lane 3 DPHY Control Register2" },
	{ 0x0204, "STARTCNTRL ", 16, "CSI - 2 - TX Start Control Register" },
	{ 0x0206, "STARTCNTRL ", 16, "CSI - 2 - TX Start Control Register2" },
	{ 0x0208, "STATUS ", 16, "CSI - 2 - TX Status Register" },
	{ 0x020A, "STATUS ", 16, "CSI - 2 - TX Status Register2" },
	{ 0x0210, "LINEINITCNT ", 16, "CSI - 2 - TX Line Initialization Control Register" },
	{ 0x0212, "LINEINITCNT ", 16, "CSI - 2 - TX Line Initialization Control Register2" },
	{ 0x0214, "LPTXTIMECNT", 16, "SYSLPTX Timing Generation Counter" },
	{ 0x0216, "LPTXTIMECNT", 16, "SYSLPTX Timing Generation Counter2" },
	{ 0x0218, "TCLK_HEADERCNT", 16, "TCLK_ZERO and TCLK_PREPARE Counter" },
	{ 0x021A, "TCLK_HEADERCNT", 16, "TCLK_ZERO and TCLK_PREPARE Counter2" },
	{ 0x021C, "TCLK_TRAILCNT", 16, "TCLK_TRAIL Counter" },
	{ 0x021E, "TCLK_TRAILCNT", 16, "TCLK_TRAIL Counter2" },
	{ 0x0220, "THS_HEADERCNT", 16, "THS_ZERO and THS_PREPARE Counter" },
	{ 0x0222, "THS_HEADERCNT", 16, "THS_ZERO and THS_PREPARE Counter2" },
	{ 0x0224, "TWAKEUP", 16, "TWAKEUP Counter" },
	{ 0x0226, "TWAKEUP", 16, "TWAKEUP Counter2" },
	{ 0x0228, "TCLK_POSTCNT", 16, "TCLK_POST Counter" },
	{ 0x022A, "TCLK_POSTCNT", 16, "TCLK_POST Counter2" },
	{ 0x022C, "THS_TRAILCNT", 16, "THS_TRAIL Counter" },
	{ 0x022E, "THS_TRAILCNT", 16, "THS_TRAIL Counter2" },
	{ 0x0230, "HSTXVREGCNT", 16, "TX Voltage Regulator setup Wait Counter" },
	{ 0x0232, "HSTXVREGCNT", 16, "TX Voltage Regulator setup Wait Counter2" },
	{ 0x0234, "HSTXVREGEN", 16, "Voltage regulator enable for HSTX Data Lanes" },
	{ 0x0236, "HSTXVREGEN", 16, "Voltage regulator enable for HSTX Data Lanes2" },

	// TX CTRL (32 - bit Address)
	{ 0x040C, "CSI_CONTROL", 16, "CSI2TXControl Register" },
	{ 0x040E, "CSI_CONTROL", 16, "CSI2TXControl Register2" },
	{ 0x0410, "CSI_STATUS", 16, "CSI2TXStatus Register" },
	{ 0x0412, "CSI_STATUS", 16, "CSI2TXStatus Register2" },
	{ 0x0414, "CSI_INT ", 16, "CSI2TX – Presents interrupts currently being held" },
	{ 0x0416, "CSI_INT ", 16, "CSI2TX – Presents interrupts currently being held2" },
	{ 0x0418, "CSI_INT_ENA", 16, "CSI2TX – Enables CSI_INT interrupt source" },
	{ 0x041A, "CSI_INT_ENA", 16, "CSI2TX – Enables CSI_INT interrupt source2" },
	{ 0x044C, "CSI_ERR", 16, "CSI2TX – transfer general errors" },
	{ 0x044E, "CSI_ERR", 16, "CSI2TX – transfer general errors2" },
	{ 0x0450, "CSI_ERR_INTENA", 16, "CSI2TX – interrupt enable bits of the CSI_ERR register" },
	{ 0x0452, "CSI_ERR_INTENA", 16, "CSI2TX – interrupt enable bits of the CSI_ERR register2" },
	{ 0x0454, "CSI_ERR_HALT", 16, "CSI2TX – stop on error bit set in the CSI_ERR register" },
	{ 0x0456, "CSI_ERR_HALT", 16, "CSI2TX – stop on error bit set in the CSI_ERR register2" },
	{ 0x0500, "CSI_CONFW", 16, "CSI TX Configure Write Register" },
	{ 0x0502, "CSI_CONFW", 16, "CSI TX Configure Write Register2" },
	{ 0x0504, "CSI_RESET", 16, "CSI2TX – reset he module and the Receive FIFO content" },
	{ 0x0506, "CSI_RESET", 16, "CSI2TX – reset he module and the Receive FIFO content2" },
	{ 0x050C, "CSI_INT_CLR", 16, "CSI2TX – Clears particular bits of the CSI_INT register" },
	{ 0x050E, "CSI_INT_CLR", 16, "CSI2TX – Clears particular bits of the CSI_INT register2" },
	{ 0x0518, "CSI_START", 16, "CSI2 - TX – Starts CSI - 2 - TX operation" },
	{ 0x051A, "CSI_START", 16, "CSI2 - TX – Starts CSI - 2 - TX operation2" },

	// Debug Tx (Color Bar, 16 - bit Address))
	{ 0x00e0, "DBG_LCNT", 16, "Color Bar Generation Setting for Line Count" },
	{ 0x00e2, "DBG_WIDTH", 16, "Color Bar Generation Setting for Line Width" },
	{ 0x00e4, "DBG_VBlank", 16, "Color Bar Generation Setting for Vertical Blank lines" },
	{ 0x00e8, "DBG_Data", 16, "Color Bar Generation Setting for Data Written into FIFO" },

	// end marker
	{ 0, NULL, 0, NULL }
};

void dump_regs(void)
{
	int i = 0;

	printk("---- TC358746 registers ----\n");
	for (i = 0; regs[i].bits != 0; ++i)
	{
		switch (regs[i].bits)
		{
		case 0:
		default:
			goto end_loop;
		case 16:
			printk("Reg %04x(%s) = %04x; %s\n", regs[i].addr, regs[i].name, i2c_rd16(g_sd, regs[i].addr), regs[i].comment);
			break;
		case 32:
			printk("Reg %04x(%s) = %08x; %s\n", regs[i].addr, regs[i].name, i2c_rd32(g_sd, regs[i].addr), regs[i].comment);
			break;
		}
	}
end_loop:
	printk("---- end registers ----\n");
	return;
}

/* sys fs stuff *******************************************************/

/* We provide 4 'files' in /sys/devices/wu10cam/ folder:
 * - gpio_in = 4 inputs
 * - gpio_out = 4 outputs
 * - reg = FPGA I2C register
 * - val = FPGA I2C value
 *
 * gpio_in:
 * - bits 3..0
 * - read-only
 *
 * gpio_out:
 * - bits 7..4 = mask, which lower/value bits to use (allows control
 * of single outputs, instead of always all at once)
 * - bits 3..0 = values
 * - read and write
 * E.g. 0xFF activates all 4 outputs, 0x11 activates only one.
 *
 * Decimal format supported for reading;
 * decimal, hex and octal for writing.
 *
 * Example:
 * cat /sys/devices/wu10cam/gpio_in
 * ... will return:
 * 15 (assuming all inputs are active)
 *
 * If permissions are not appropriate, change them with chmod; here
 * where the 'files' are created, Linux allows only a limited subset of
 * permissions to be assigned.
 * */

 /* 4 inputs */

static ssize_t wu10cam_gpio_in_show(
	struct device* dev,
	struct device_attribute* attr,
	char* buf)
{
	struct tc358746_state* sensor = dev_get_drvdata(dev);
	struct i2c_client* client = sensor->i2c_client;
	int len = 0;
	u8 value = 0;
	int ret = wu10cam_read_reg_device(
		sensor,
		client->addr,
		WU10CAM_GPIO_IN_REG,
		&value);
	value &= 0x0F;

	if (ret)
	{
		dev_err(dev, "Error %d reading fpga i2c\n", ret);
		return -EIO;
	}

	len = sprintf(buf, "%u\n", (unsigned int)value);
	if (len <= 0)
	{
		dev_err(dev, "Invalid sprintf len: %d\n", len);
		return -EINVAL;
	}

	return len;
}
static DEVICE_ATTR(gpio_in, S_IRUGO, wu10cam_gpio_in_show, NULL);

/* 4 outputs **********************************************************/

static ssize_t wu10cam_gpio_out_show(
	struct device* dev,
	struct device_attribute* attr,
	char* buf)
{
	struct tc358746_state* sensor = dev_get_drvdata(dev);
	struct i2c_client* client = sensor->i2c_client;
	int len = 0;
	u8 value = 0;
	int ret = wu10cam_read_reg_device(
		sensor,
		client->addr,
		WU10CAM_GPIO_OUT_REG,
		&value);

	if (ret)
	{
		dev_err(dev, "Error %d reading fpga i2c\n", ret);
		return -EIO;
	}

	len = sprintf(buf, "%u\n", (unsigned int)value);
	if (len <= 0)
	{
		dev_err(dev, "Invalid sprintf len: %d\n", len);
		return -EINVAL;
	}

	return len;
}

static ssize_t wu10cam_gpio_out_store(
	struct device* dev,
	struct device_attribute* attr,
	const char* buf,
	size_t count)
{
	struct tc358746_state* sensor = dev_get_drvdata(dev);
	struct i2c_client* client = sensor->i2c_client_fpga;
	unsigned int value = 0;
	int ret = 0;

	ret = kstrtouint(buf, 0, &value);
	if (ret)
	{
		dev_err(dev, "Error %d at %s:%d\n", ret, __FUNCTION__, __LINE__);
		return -EINVAL;
	}

	ret = wu10cam_write_reg_device(
		sensor,
		client->addr,
		WU10CAM_GPIO_OUT_REG,
		(u8)value);
	if (ret)
	{
		dev_err(dev, "Error %d writing fpga i2c\n", ret);
		return -EIO;
	}

	return count;
}

/* cannot assign arbitrary permissions here, if these are not
 * appropriate, 'chmod' them afterwards.
 * */
static DEVICE_ATTR(gpio_out, S_IRUGO | S_IWUSR, wu10cam_gpio_out_show, wu10cam_gpio_out_store);

/* register **********************************************************/

unsigned int gi_reg = 0;

static ssize_t wu10cam_reg_show(
	struct device* dev,
	struct device_attribute* attr,
	char* buf)
{
	int len = 0;

	len = sprintf(buf, "%u\n", (unsigned int)gi_reg);
	if (len <= 0)
	{
		dev_err(dev, "Invalid sprintf len: %d\n", len);
		return -EINVAL;
	}

	return len;
}

static ssize_t wu10cam_reg_store(
	struct device* dev,
	struct device_attribute* attr,
	const char* buf,
	size_t count)
{
	int ret = 0;

	ret = kstrtouint(buf, 0, &gi_reg);
	if (ret)
	{
		dev_err(dev, "Error %d at %s:%d\n", ret, __FUNCTION__, __LINE__);
		return -EINVAL;
	}

	return count;
}

static DEVICE_ATTR(reg, S_IRUGO | S_IWUSR, wu10cam_reg_show, wu10cam_reg_store);

/* value **********************************************************/

unsigned int gi_val = 0;

static ssize_t wu10cam_val_show(
	struct device* dev,
	struct device_attribute* attr,
	char* buf)
{
	struct tc358746_state* sensor = dev_get_drvdata(dev);
	struct i2c_client* client = sensor->i2c_client;
	int len = 0;
	u32 value = 0;
	int ret = wu10cam_read_reg32_device(
		sensor,
		client->addr,
		gi_reg,
		&value);

	if (ret)
	{
		dev_err(dev, "Error %d reading fpga i2c\n", ret);
		return -EIO;
	}

	len = sprintf(buf, "0x%x\n", (unsigned int)value);
	if (len <= 0)
	{
		dev_err(dev, "Invalid sprintf len: %d\n", len);
		return -EINVAL;
	}

	return len;
}

static ssize_t wu10cam_val_store(
	struct device* dev,
	struct device_attribute* attr,
	const char* buf,
	size_t count)
{
	struct tc358746_state* state = dev_get_drvdata(dev);
	unsigned int value = 0;
	int ret = 0;

	ret = kstrtouint(buf, 0, &value);
	if (ret)
	{
		dev_err(dev, "Error %d at %s:%d\n", ret, __FUNCTION__, __LINE__);
		return -EINVAL;
	}

	ret = wu10cam_write_reg32_device(
		state,
		state->i2c_client_fpga->addr,
		gi_reg,
		(u32)value);
	if (ret)
	{
		dev_err(dev, "Error %d writing fpga i2c\n", ret);
		return -EIO;
	}

	return count;
}

/* cannot assign arbitrary permissions here, if these are not
 * appropriate, 'chmod' them afterwards.
 * */
static DEVICE_ATTR(val, S_IRUGO | S_IWUSR, wu10cam_val_show, wu10cam_val_store);


static struct attribute* wu10cam_attrs[] =
{
	&dev_attr_gpio_in.attr,
	&dev_attr_gpio_out.attr,
	&dev_attr_reg.attr,
	&dev_attr_val.attr,
	NULL
};

static struct attribute_group wu10cam_group =
{
	.name = NULL, // do not create new folder, we already have one
	.attrs = wu10cam_attrs,
};

static int tc358746_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
{
//	static struct v4l2_dv_timings default_timing =
//		V4L2_DV_BT_CEA_640X480P59_94;
	static struct v4l2_dv_timings default_timing =
	{
		.type = V4L2_DV_BT_656_1120,
		.reserved = { 0 },
		.bt.width = 640,
		.bt.height = 480,
		.bt.interlaced = V4L2_DV_PROGRESSIVE,
		.bt.pixelclock = 25000000,
		.bt.hfrontporch = 16,
		.bt.hsync = 96,
		.bt.hbackporch = 48,
		.bt.vfrontporch = 2,
		.bt.vsync = 6,
		.bt.vbackporch = 1,
		.bt.standards = V4L2_DV_BT_STD_CVT | V4L2_DV_BT_STD_GTF,
		.bt.flags = 0,
		.bt.reserved = { 0 }
	};
	struct tc358746_state *state;
	struct tc358746_platform_data *pdata = client->dev.platform_data;
	struct v4l2_subdev *sd;
	int err;
	u16 val16;

//	debug = 3;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA))
	{
		v4l_err(client, "i2c_check_func fail\n");
		return -EIO;
	}
	v4l_dbg(1, debug, client, "chip found @ 0x%x (%s)\n",
		client->addr << 1, client->adapter->name);

	state = devm_kzalloc(&client->dev, sizeof(struct tc358746_state),
			GFP_KERNEL);
	if (!state)
		return -ENOMEM;

	state->i2c_client = client;

	{
		struct i2c_board_info ibi_fpga = { I2C_BOARD_INFO("fpga", 0x55) };
		state->i2c_client_fpga = i2c_new_device(client->adapter, &ibi_fpga);
		if (state->i2c_client_fpga == NULL)
		{
			v4l_err(client, "failed to create new fpga i2c device\n");
		}
	}

	/* platform data */
	if (pdata) {
		state->pdata = *pdata;
		state->bus.flags = V4L2_MBUS_CSI2_CONTINUOUS_CLOCK;
	} else {
		err = tc358746_probe_of(state);
		if (err == -ENODEV)
			v4l_err(client, "No platform data!\n");
		if (err)
			return err;
	}

	sd = &state->sd;
	v4l2_i2c_subdev_init(sd, client, &tc358746_ops);
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE /*| V4L2_SUBDEV_FL_HAS_EVENTS*/;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;

	val16 = i2c_rd16(sd, CHIPID);
	if ((val16 & MASK_CHIPID) != 0x4400) {
		v4l2_info(sd, "not a tc358746 on address 0x%x\n",
			  client->addr /* << 1*/);
		return -ENODEV;
	}

	state->pad.flags = MEDIA_PAD_FL_SOURCE;
	err = media_entity_pads_init(&sd->entity, 1, &state->pad);
	if (err < 0)
		goto err_hdl;

	state->mbus_fmt_code = MEDIA_BUS_FMT_UYVY8_2X8;	// dvm

	/* control handlers */
	tc358746_init_controls(state);

	sd->ctrl_handler = &state->ctrl_hdl;
	if (state->ctrl_hdl.error) {
		err = state->ctrl_hdl.error;
		goto err_hdl;
	}

	sd->dev = &client->dev;
	err = v4l2_async_register_subdev(sd);
	if (err < 0)
		goto err_hdl;

	mutex_init(&state->confctl_mutex);

	tc358746_s_dv_timings(sd, &default_timing);

	err = v4l2_ctrl_handler_setup(sd->ctrl_handler);
	if (err)
		goto err_work_queues;

	v4l2_info(sd, "%s found @ 0x%x (%s)\n", client->name,
		  client->addr << 1, client->adapter->name);

	g_sd = sd;
	tc358746_initialize_registers();
#if 0	// enable to have a printout of all chip's registers
	usleep_range(1000 * 1000, 2000 * 1000);
	dump_regs();
#endif

	state->dev_folder = root_device_register("wu10cam");
	if (IS_ERR(state->dev_folder)) {
		v4l_err(client, "sysfs folder creation failed\n");
		goto err_work_queues;
	}
	dev_set_drvdata(state->dev_folder, state);

	err = sysfs_create_group(&state->dev_folder->kobj, &wu10cam_group);
	if (err) {
		v4l_err(client, "sysfs attributes creation failed\n");
		goto err_work_queues;
	}

	state->controls_initialized = true;

	err = wu10cam_write_reg32_device(
		state,
		state->i2c_client_fpga->addr,
		WU10CAM_VIDEO_SELECT_REG,
		(u32)0x01 << 7);
	if (err)
	{
		v4l_err(client, "Error %d writing fpga i2c\n", err);
		//return -EIO;
	}

	return 0;

err_work_queues:
	if (!state->i2c_client->irq)
		flush_work(&state->work_i2c_poll);
	mutex_destroy(&state->confctl_mutex);
err_hdl:
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(&state->ctrl_hdl);
	return err;
}

static int tc358746_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct tc358746_state *state = to_state(sd);
	struct device* dev = &client->dev;

	sysfs_remove_group(&dev->kobj, &wu10cam_group);
	if (state->dev_folder)
		root_device_unregister(state->dev_folder);

	v4l2_async_unregister_subdev(sd);
	v4l2_device_unregister_subdev(sd);
	mutex_destroy(&state->confctl_mutex);
	media_entity_cleanup(&sd->entity);
	//v4l2_ctrl_handler_free(&state->ctrl_hdl);
	wu10cam_exit_controls(state);

	if (state->i2c_client_fpga)
		i2c_unregister_device(state->i2c_client_fpga);

	return 0;
}

static const struct i2c_device_id tc358746_id[] = {
	{"tc358746", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, tc358746_id);

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id tc358746_of_match[] = {
	{ .compatible = "toshiba,tc358746" },
	{},
};
MODULE_DEVICE_TABLE(of, tc358746_of_match);
#endif

static struct i2c_driver tc358746_driver = {
	.driver = {
		.name = "tc358746",
		.of_match_table = of_match_ptr(tc358746_of_match),
	},
	.probe = tc358746_probe,
	.remove = tc358746_remove,
	.id_table = tc358746_id,
};

module_i2c_driver(tc358746_driver);

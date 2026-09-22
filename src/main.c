#include <stdio.h>
#include <stdbool.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/drivers/led_strip.h>
#include <zephyr/drivers/display.h>
#include <zephyr/drivers/fuel_gauge.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/display/cfb.h>
#include <zephyr/input/input.h>
#include <zephyr/kvss/nvs.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/util.h>

#define STRIP_NODE       DT_ALIAS(led_strip)
#define STRIP_NUM_PIXELS DT_PROP(STRIP_NODE, chain_length)
#define DISPLAY_NODE     DT_CHOSEN(zephyr_display)
#define FUEL_GAUGE_NODE  DT_NODELABEL(max17048)
#define DISTANCE_NODE    DT_NODELABEL(tk50)

/* MAX17048 VCELL floats near 0V with no cell attached; a real LiPo's
 * protection circuit cuts off well above this, leaving a wide margin.
 */
#define BATTERY_PRESENT_THRESHOLD_UV 2000000

/* Stop distance is user-editable (see button handling below), in tenths of
 * a foot so each button push moves it by exactly 0.1ft with no rounding
 * drift; it's converted to mm only when compared against a sensor reading.
 * The slow threshold is always this many feet farther than the stop distance.
 */
#define DISTANCE_NEAR_DEFAULT_TENTHS_FT 5
#define DISTANCE_NEAR_MIN_TENTHS_FT     1
#define DISTANCE_NEAR_MAX_TENTHS_FT     60
#define DISTANCE_NEAR_STEP_TENTHS_FT    1
#define DISTANCE_SLOW_OFFSET_TENTHS_FT  20

#define LOOP_DELAY K_MSEC(200)

/* How long to stay active after the SR602's OUT pin drops low. */
#define MOTION_HOLD_MS 10000

/* How long to wait after a single button press before treating it as a
 * single up/down step, so a two-button chord isn't seen as a step + toggle.
 */
#define COMBO_WINDOW_MS 150

#define NVS_ID_DISTANCE_NEAR_TENTHS_FT 1

enum app_mode {
	MODE_NORMAL = 0,
	MODE_EDIT,
};

struct color_step {
	struct led_rgb rgb;
	const char *label;
};

static const struct color_step steps[] = {
	{ { .r = 0x22, .g = 0x00, .b = 0x00 }, "STOP" },
	{ { .r = 0x11, .g = 0x11, .b = 0x00 }, "SLOW" },
	{ { .r = 0x00, .g = 0x22, .b = 0x00 }, "CLEAR" },
};

static const struct led_rgb edit_rgb = { .r = 0x00, .g = 0x00, .b = 0x22 };

static const struct device *const strip = DEVICE_DT_GET(STRIP_NODE);
static const struct device *const display = DEVICE_DT_GET(DISPLAY_NODE);
static const struct device *const fuel_gauge = DEVICE_DT_GET(FUEL_GAUGE_NODE);
static const struct device *const distance_sensor = DEVICE_DT_GET(DISTANCE_NODE);

static struct nvs_fs nvs;
static int32_t distance_near_tenths_ft = DISTANCE_NEAR_DEFAULT_TENTHS_FT;

/* SR602 PIR sensor: OUT stays high for the duration of the sensor's own
 * retriggerable hold time, so its level doubles as our active/idle state.
 */
static K_SEM_DEFINE(wake_sem, 0, 1);
static atomic_t motion_active = ATOMIC_INIT(0);
static atomic_t app_mode = ATOMIC_INIT(MODE_NORMAL);

static bool up_held;
static bool down_held;

static void save_distance_near_tenths_ft(void)
{
	nvs_write(&nvs, NVS_ID_DISTANCE_NEAR_TENTHS_FT, &distance_near_tenths_ft,
		  sizeof(distance_near_tenths_ft));
}

static void toggle_edit_mode(void)
{
	if (atomic_get(&app_mode) == MODE_NORMAL) {
		atomic_set(&app_mode, MODE_EDIT);
		k_sem_give(&wake_sem);
	} else {
		atomic_set(&app_mode, MODE_NORMAL);
		save_distance_near_tenths_ft();
	}
}

static void up_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	if (atomic_get(&app_mode) == MODE_EDIT) {
		distance_near_tenths_ft = MIN(distance_near_tenths_ft + DISTANCE_NEAR_STEP_TENTHS_FT,
					      DISTANCE_NEAR_MAX_TENTHS_FT);
	}
}

static void down_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	if (atomic_get(&app_mode) == MODE_EDIT) {
		distance_near_tenths_ft = MAX(distance_near_tenths_ft - DISTANCE_NEAR_STEP_TENTHS_FT,
					      DISTANCE_NEAR_MIN_TENTHS_FT);
	}
}

static K_WORK_DELAYABLE_DEFINE(up_work, up_work_handler);
static K_WORK_DELAYABLE_DEFINE(down_work, down_work_handler);

/* A press while the other button is already held is a chord: cancel any
 * pending single-step action from the first press and toggle edit mode.
 * A press on its own is queued, so a chord's first half doesn't register
 * as a spurious step before the second half arrives.
 */
static void handle_button(bool is_up, bool pressed)
{
	bool other_held = is_up ? down_held : up_held;

	if (is_up) {
		up_held = pressed;
	} else {
		down_held = pressed;
	}

	if (!pressed) {
		return;
	}

	if (other_held) {
		k_work_cancel_delayable(&up_work);
		k_work_cancel_delayable(&down_work);
		toggle_edit_mode();
		return;
	}

	k_work_schedule(is_up ? &up_work : &down_work, K_MSEC(COMBO_WINDOW_MS));
}

static void input_event_cb(struct input_event *evt, void *user_data)
{
	ARG_UNUSED(user_data);

	switch (evt->code) {
	case INPUT_KEY_WAKEUP:
		atomic_set(&motion_active, evt->value);
		if (evt->value) {
			k_sem_give(&wake_sem);
		}
		break;
	case INPUT_KEY_UP:
		handle_button(true, evt->value);
		break;
	case INPUT_KEY_DOWN:
		handle_button(false, evt->value);
		break;
	default:
		break;
	}
}
INPUT_CALLBACK_DEFINE(NULL, input_event_cb, NULL);

static void nvs_setup(void)
{
	struct flash_pages_info info;

	nvs.flash_device = PARTITION_DEVICE(storage_partition);
	nvs.offset = PARTITION_OFFSET(storage_partition);

	if (!device_is_ready(nvs.flash_device) ||
	    flash_get_page_info_by_offs(nvs.flash_device, nvs.offset, &info) != 0) {
		return;
	}

	nvs.sector_size = info.size;
	nvs.sector_count = 4U;

	if (nvs_mount(&nvs) != 0) {
		return;
	}

	nvs_read(&nvs, NVS_ID_DISTANCE_NEAR_TENTHS_FT, &distance_near_tenths_ft,
		 sizeof(distance_near_tenths_ft));
}

static void set_idle_outputs(void)
{
	struct led_rgb pixels[STRIP_NUM_PIXELS] = { 0 };

	led_strip_update_rgb(strip, pixels, STRIP_NUM_PIXELS);
	display_blanking_on(display);
}

static void format_battery_lines(char *line1, size_t len1, char *line2, size_t len2)
{
	union fuel_gauge_prop_val voltage;
	union fuel_gauge_prop_val soc;

	if (!device_is_ready(fuel_gauge) ||
	    fuel_gauge_get_prop(fuel_gauge, FUEL_GAUGE_VOLTAGE, &voltage) != 0 ||
	    voltage.voltage <= BATTERY_PRESENT_THRESHOLD_UV) {
		snprintf(line1, len1, "batt: none");
		line2[0] = '\0';
		return;
	}

	fuel_gauge_get_prop(fuel_gauge, FUEL_GAUGE_RELATIVE_STATE_OF_CHARGE, &soc);
	snprintf(line1, len1, "batt %u%%", soc.relative_state_of_charge);
	snprintf(line2, len2, "%dmV", voltage.voltage / 1000);
}

/* Reads the current range. Returns false (treated as "clear") if the
 * sensor isn't ready or has no valid target in range.
 */
static bool read_distance_mm(int32_t *distance_mm)
{
	struct sensor_value value;

	if (!device_is_ready(distance_sensor) || sensor_sample_fetch(distance_sensor) != 0) {
		return false;
	}

	sensor_channel_get(distance_sensor, SENSOR_CHAN_DISTANCE, &value);
	*distance_mm = (int32_t)sensor_value_to_milli(&value);
	return true;
}

static const struct color_step *band_for_distance(bool valid, int32_t distance_mm,
						   int32_t near_mm, int32_t mid_mm)
{
	if (!valid || distance_mm > mid_mm) {
		return &steps[2];
	}
	if (distance_mm > near_mm) {
		return &steps[1];
	}
	return &steps[0];
}

/* 1 ft = 304.8 mm. Uses integer math only, since printf float support
 * isn't enabled.
 */
static int32_t tenths_ft_to_mm(int32_t tenths_ft)
{
	return (tenths_ft * 3048) / 100;
}

static void format_tenths_ft(char *buf, size_t len, int32_t tenths_ft)
{
	snprintf(buf, len, "%d.%d ft", tenths_ft / 10, tenths_ft % 10);
}

static void format_feet(char *buf, size_t len, int32_t mm)
{
	format_tenths_ft(buf, len, (mm * 100 + 1524) / 3048);
}

static void render_edit_screen(uint8_t font_height, int32_t near_tenths_ft)
{
	struct led_rgb pixels[STRIP_NUM_PIXELS];
	char value[16];
	char line[24];

	for (size_t p = 0; p < STRIP_NUM_PIXELS; p++) {
		pixels[p] = edit_rgb;
	}
	led_strip_update_rgb(strip, pixels, STRIP_NUM_PIXELS);

	format_tenths_ft(value, sizeof(value), near_tenths_ft);

	cfb_framebuffer_clear(display, false);
	cfb_print(display, "EDIT", 0, 0);
	snprintf(line, sizeof(line), "stop %s", value);
	cfb_print(display, line, 0, font_height);
	cfb_framebuffer_finalize(display);
}

int main(void)
{
	struct led_rgb pixels[STRIP_NUM_PIXELS];
	uint8_t font_width;
	uint8_t font_height;
	char distance_line[16];
	char battery_line1[16];
	char battery_line2[16];

	if (!device_is_ready(strip) || !device_is_ready(display)) {
		return 0;
	}

	if (display_set_pixel_format(display, PIXEL_FORMAT_MONO10) != 0 &&
	    display_set_pixel_format(display, PIXEL_FORMAT_MONO01) != 0) {
		return 0;
	}

	if (cfb_framebuffer_init(display)) {
		return 0;
	}

	cfb_get_font_size(display, 0, &font_width, &font_height);

	nvs_setup();

	set_idle_outputs();

	while (1) {
		/* Idle: everything off, thread blocked until motion or a
		 * button chord wakes it.
		 */
		k_sem_take(&wake_sem, K_FOREVER);
		display_blanking_off(display);

		/* Active: run the sense/display/LED loop while the SR602's
		 * OUT pin is asserted, plus a hold period after it drops
		 * low. Renewed motion cancels the hold. Edit mode stays
		 * active regardless of motion until the buttons exit it.
		 */
		int64_t idle_deadline = 0;

		while (true) {
			if (atomic_get(&app_mode) == MODE_EDIT) {
				idle_deadline = 0;
				render_edit_screen(font_height, distance_near_tenths_ft);
				k_sleep(LOOP_DELAY);
				continue;
			}

			if (atomic_get(&motion_active)) {
				idle_deadline = 0;
			} else if (idle_deadline == 0) {
				idle_deadline = k_uptime_get() + MOTION_HOLD_MS;
			} else if (k_uptime_get() >= idle_deadline) {
				break;
			}

			int32_t distance_mm = 0;
			bool valid = read_distance_mm(&distance_mm);
			int32_t near_mm = tenths_ft_to_mm(distance_near_tenths_ft);
			int32_t mid_mm = tenths_ft_to_mm(distance_near_tenths_ft +
							  DISTANCE_SLOW_OFFSET_TENTHS_FT);
			const struct color_step *band =
				band_for_distance(valid, distance_mm, near_mm, mid_mm);

			for (size_t p = 0; p < STRIP_NUM_PIXELS; p++) {
				pixels[p] = band->rgb;
			}
			led_strip_update_rgb(strip, pixels, STRIP_NUM_PIXELS);

			if (valid) {
				format_feet(distance_line, sizeof(distance_line), distance_mm);
			} else {
				snprintf(distance_line, sizeof(distance_line), "-- ft");
			}
			format_battery_lines(battery_line1, sizeof(battery_line1),
					     battery_line2, sizeof(battery_line2));

			cfb_framebuffer_clear(display, false);
			cfb_print(display, band->label, 0, 0);
			cfb_print(display, distance_line, 0, font_height);
			cfb_print(display, battery_line1, 0, 2 * font_height);
			cfb_print(display, battery_line2, 0, 3 * font_height);
			cfb_framebuffer_finalize(display);

			k_sleep(LOOP_DELAY);
		}

		set_idle_outputs();
	}

	return 0;
}

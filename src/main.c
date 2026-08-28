#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/led_strip.h>
#include <zephyr/sys/util.h>

#define STRIP_NODE       DT_ALIAS(led_strip)
#define STRIP_NUM_PIXELS DT_PROP(STRIP_NODE, chain_length)

#define COLOR_DELAY K_SECONDS(2)

static const struct led_rgb colors[] = {
	{ .r = 0x22, .g = 0x00, .b = 0x00 }, /* red */
	{ .r = 0x11, .g = 0x11, .b = 0x00 }, /* yellow */
	{ .r = 0x00, .g = 0x22, .b = 0x00 }, /* green */
};

static const struct device *const strip = DEVICE_DT_GET(STRIP_NODE);

int main(void)
{
	struct led_rgb pixels[STRIP_NUM_PIXELS];

	if (!device_is_ready(strip)) {
		return 0;
	}

	while (1) {
		for (size_t i = 0; i < ARRAY_SIZE(colors); i++) {
			for (size_t p = 0; p < STRIP_NUM_PIXELS; p++) {
				pixels[p] = colors[i];
			}

			led_strip_update_rgb(strip, pixels, STRIP_NUM_PIXELS);
			k_sleep(COLOR_DELAY);
		}
	}

	return 0;
}

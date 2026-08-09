// ESP-IDF wiring for the GPIO9 mode button. All timing logic lives in rt_button_sm.c.

#include "rt_button.h"

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "rt_button";

#define POLL_PERIOD_MS 10

typedef struct {
    rt_button_cb_t cb;
    void          *ctx;
} args_t;

static void button_task(void *pv)
{
    args_t *args = (args_t *)pv;

    rt_button_state_t st;
    rt_button_state_init(&st, (uint32_t)(esp_timer_get_time() / 1000));

    for (;;) {
        const uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
        // Active low: the external pull-up holds it high, the button pulls it to ground.
        const bool pressed = gpio_get_level(RT_BUTTON_GPIO) == 0;

        const rt_button_event_t ev = rt_button_update(&st, pressed, now_ms);
        if (ev != RT_BUTTON_NONE && args->cb != NULL) {
            args->cb(ev, args->ctx);
        }

        vTaskDelay(pdMS_TO_TICKS(POLL_PERIOD_MS));
    }
}

void rt_button_start(rt_button_cb_t cb, void *ctx)
{
    static args_t args;
    args.cb  = cb;
    args.ctx = ctx;

    const gpio_config_t io = {
        .pin_bit_mask = 1ULL << RT_BUTTON_GPIO,
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io));

    xTaskCreate(button_task, "rt_button", 3072, &args, 5, NULL);
    ESP_LOGI(TAG, "GPIO%d: tap = next mode, long press (%dms) = back to normal",
             RT_BUTTON_GPIO, RT_BUTTON_LONG_PRESS_MS);
}

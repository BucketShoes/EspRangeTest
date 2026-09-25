// ESP32-C6 range tester.
//
// All three radios transmit a small numbered packet on a timer and listen the rest of the
// time. Every couple of seconds the results table goes out over serial: RSSI and packet
// loss per peer, per radio. Flash two boards, walk away with one, watch the numbers.
//
// Every test is its own switch, off at boot (RT_TEST_* in rt.h), set from the phone. GPIO9 (the
// BOOT button) steps through each test on its own, and holding it puts everything back to how
// it booted. The radios share one antenna and arbitrate for it, so running them together costs
// something; how much is one of the things worth measuring.

#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "esp_chip_info.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "soc/soc_caps.h"

#include "rt.h"

static const char *TAG = "rt";

// 2s was the upper bound of usable, not a target - at that rate a walk is hard to read as it
// happens. The measurement packets themselves run at 2-4Hz, so this only ever governed how
// often the table is summarised, never what was captured. "pdr now" has its own time window
// (RT_PDR_WINDOW_MS in rt.h) and does not depend on this.
#define REPORT_MS    1000
#define BUTTON_GPIO  9
#define WIFI_CHAN    1

// XIAO ESP32C6 RF switch power. Drive LOW to put the antenna on the air at all - but only
// once every radio's transmit power has been set: see antenna_switch_on() below.
//
// The module's antenna path runs through an FM8625H SPDT switch whose VDD comes from a P-FET
// (Q3) with a 10k pull-up on its gate and its pull-down not populated. Gate high means the FET
// is off, so **out of reset the RF switch has no supply**: the antenna is not connected to the
// radio, and the only coupling left is the switch's off-isolation. Measured cost of that:
// about 70 dB, which is what had two boards a metre apart delivering 3 packets out of 5000.
//
// So this is not an optimisation or a board variant to support politely. Without it the
// hardware does not radiate, and every radio on the chip is equally affected.
//
// GPIO14 is VCTL, the port select: low picks RF1 (the onboard ceramic antenna), high picks RF2
// (the U.FL connector). R24 holds it low, so the internal antenna is the powered-up default
// even with nothing driving it - but it is now driven explicitly rather than left to the
// pulldown, so the selection is a known state rather than an assumed one.
//
// Harmless on the DevKitC/DevKitM, where GPIO3 is an ordinary unused pin - and GPIO3 is not a
// strapping pin on the C6 (those are 8, 9 and 15), so driving it out of reset is safe.
#define ANT_PWR_GPIO    3
#define ANT_SEL_GPIO    14    // VCTL: 0 = RF1 = chip antenna, 1 = RF2 = U.FL
#define ANT_SETTLE_MS   100   // the vendor example waits before using the switch; so do we
#define ANT_PWR_WAIT_MS 3000  // backstop on waiting for BLE to report its power

// The XIAO's user LED on GPIO15.
//
// Active low, confirmed on the bench rather than read off a schematic. LED_ON_LEVEL is the
// level driven while the LED is lit and is the only place in this project where a level
// appears at all - everything else, firmware and web UI alike, speaks in off/on/blink.
//
// GPIO15 is a strapping pin (the C6's are 8, 9 and 15), which is part of why the LED defaults
// to floating rather than being configured at boot: the pin is left exactly as reset found it
// until someone asks for the LED. "Off" returns it to that same high-Z state rather than
// driving the inactive level, so off is the boot condition itself and not a lookalike.
//
// The blink is a software timer toggling the pin, not LEDC. LEDC cannot reach 2Hz at any
// useful resolution - its slowest source is the 8MHz RC oscillator into a 20-bit counter,
// which bottoms out around 7.6Hz - and at 2Hz there is nothing for hardware PWM to buy: PWM
// only differs from a toggle when the eye is meant to integrate it, and at half a second per
// phase nobody's does. What is wanted here is a blink, and a blink is what this is.
#define LED_GPIO      15
#define LED_ON_LEVEL  0
#define LED_OFF_LEVEL (!LED_ON_LEVEL)
#define LED_HALF_MS   125   // 2Hz at 50% duty: lit for one half period, dark for the other

// The button has exactly two gestures, and no third is allowed to appear.
//
// Tap = next mode, hold = restore. That is it. HOLD_MS is not a window you have to release
// inside - it is a floor, with no ceiling, and the restore fires the moment you cross it while
// still holding. Anything you keep doing after that changes nothing.
//
// The reason is that this has to work blind, with a board in a pocket or at arm's length in
// the dark, by someone who has just lost every other way to control it. A tap and a hold are
// distinguishable without a clock. A "medium" hold is not: adding a third, longer tier would
// silently turn the middle one into a bounded window you have to time by guesswork, which is
// exactly the property the recovery gesture must not have. So the LR toggle, which used to sit
// on a 3s super-hold, now lives on the command channel instead (see RT_CMD_LR_* in rt.h) -
// where getting it wrong is recoverable by, precisely, holding this button.
//
// The tap is not a duration anyone has to hit. It is just "released before the hold", which
// doubles as the debounce.
#define HOLD_MS 800

// g_lr / rt_set_lr / rt_apply_test_radios / rt_wifi_active live outside the RT_STAGE>=1 guard
// below because button_task (and rt_set_tests, which it drives) run at every stage - a board on
// RT_STAGE=0 still has a button task that can call rt_set_tests, which always calls
// rt_apply_test_radios. Below stage 1 there is no Wi-Fi driver to start or stop, so rt_set_lr
// just tracks the flag and rt_wifi_active is always false.
volatile bool g_lr;

// Set once at boot and then reported in every periodic report, not just the startup banner.
static const char *s_reset_reason = "?";
static uint8_t     s_reset_code;   // same thing as a number, for the packed report

const char *rt_reset_reason(void)
{
    return s_reset_reason;
}

uint8_t rt_reset_code(void)
{
    return s_reset_code;
}

#if RT_STAGE >= 1
// 11b and nothing else.
//
// This used to be 11B|11G|11N|11AX - the set ESP-IDF would pick by default - which let rate
// control choose anything up to HE, and rate sets range. On an instrument whose only metric is
// distance, letting the driver silently pick a faster, shorter-range modulation is an unknown
// in the middle of the measurement. 11b is the long-range one and the only one this project
// has ever been interested in; g/n/ax are dropped outright rather than merely deprioritised.
//
// LR is deliberately not here. It goes in only when g_lr is set, because its mere presence in
// the list makes the SoftAP invisible to anything that is not an ESP (see LR mode in
// CONTEXT.md) - the flag is not a preference, it is a different world.
#define WIFI_PROTO_BASE (WIFI_PROTOCOL_11B)

// What the driver is actually doing right now, so wifi_apply() can tell a real change from a
// repeat and leave a working AP alone when nothing needs to move.
static bool    s_wifi_on;
static uint8_t s_wifi_proto;
static bool    s_wifi_ap;

// Set once in wifi_start(), and put back every time the AP comes up again after a spell in
// STA-only mode, rather than trusting the driver to have kept it across the mode change.
static wifi_config_t s_ap_cfg;

// Held on through boot, whatever the tests say, because esp_now_init() wants a started driver.
// Released in app_main once ESP-NOW is up. That start happens before the antenna switch is
// powered, which is the same place the old always-on boot started it - see antenna_switch_on().
static bool s_wifi_boot_hold = true;

// Whether any test needs the Wi-Fi radio at all.
//
// This is the fix for the thing low contention never used to do. "Isolate 802.15.4" meant
// "stop calling esp_now_send()", which changes nothing about the antenna: the SoftAP goes on
// beaconing every ~100ms and keeps its receiver up continuously to hear probes, and both of
// those outrank 802.15.4 in the coex arbiter - which hardcodes ordinary 802.15.4 tx/rx at its
// lowest priority tier. The result was ESP_IEEE802154_TX_ERR_COEXIST on every single frame,
// in a mode whose entire purpose was to give 802.15.4 a clear run. So the driver stops.
//
// With the driver stopped there is no interface to send on in the first place, which is why
// nothing here needs to care about the ESP-NOW tx gate or the FTM task.
static bool wifi_wanted(void)
{
    return s_wifi_boot_hold || (g_tests & RT_TEST_WIFI) != 0;
}

static uint8_t wifi_proto(void)
{
    return WIFI_PROTO_BASE | (g_lr ? WIFI_PROTOCOL_LR : 0);
}

// The AP only while the AP test is on - it beacons every ~100ms and listens continuously, and
// ESP-NOW and the FTM initiator both work from the station side alone. Not during the boot hold
// either: nothing has asked for it yet.
static bool ap_wanted(void)
{
    return (g_tests & RT_TEST_AP) != 0;
}

// Idempotent: brings the driver to whatever g_tests and g_lr currently ask for. Both a switch
// and an LR toggle route through here, because both want the same stop/reconfigure/start
// sequence and doing it twice for one button press would bounce the AP for no reason.
//
// LR needs that full reinit rather than a live protocol change (CONTEXT.md, established by the
// owner's earlier testing). Merely having WIFI_PROTOCOL_LR in the list is enough to blind a
// phone to the SoftAP even though it does not stop other ESPs reaching it - so it is applied
// to both interfaces together, matching that they share one radio. The AP coming and going
// rides the same restart.
static void wifi_apply(void)
{
    const bool    want  = wifi_wanted();
    const bool    ap    = want && ap_wanted();
    const uint8_t proto = wifi_proto();

    if (s_wifi_on == want && (!want || (s_wifi_proto == proto && s_wifi_ap == ap))) {
        return;
    }

    if (s_wifi_on) {
        RT_TRY(TAG, esp_wifi_stop());
        s_wifi_on = false;
    }

    if (!want) {
        // Not an error and not a failure to configure something: the radio is off on purpose,
        // and saying so plainly is the whole point - a silent Wi-Fi stack is exactly what the
        // old code looked like while it was still holding the antenna.
        ESP_LOGI(TAG, "wifi stopped - no Wi-Fi test is on");
        return;
    }

    RT_TRY(TAG, esp_wifi_set_mode(ap ? WIFI_MODE_APSTA : WIFI_MODE_STA));
    RT_TRY(TAG, esp_wifi_set_protocol(WIFI_IF_STA, proto));
    if (ap) {
        RT_TRY(TAG, esp_wifi_set_protocol(WIFI_IF_AP, proto));
        RT_TRY(TAG, esp_wifi_set_config(WIFI_IF_AP, &s_ap_cfg));
    }

    RT_TRY(TAG, esp_wifi_start());
    s_wifi_on = true;

    // Immediately, before the channel or anything else: esp_wifi_set_max_tx_power() may only
    // be called after start, and until it is, the driver is at its own default - maximum - and
    // the SoftAP has already begun beaconing. The window cannot be closed entirely through
    // this API, only made as short as possible.
    rt_wifi_apply_power();

    RT_TRY(TAG, esp_wifi_set_channel(WIFI_CHAN, WIFI_SECOND_CHAN_NONE));
    s_wifi_proto = proto;
    s_wifi_ap    = ap;
    rt_espnow_resume();  // no-op until rt_espnow_start() has run

    ESP_LOGI(TAG, "wifi started: %s, lr=%s", ap ? "AP + station" : "station only",
             g_lr ? "on" : "off");
}
#endif

// Applying a power level is only meaningful while the driver is up, and every path that starts
// it calls this - so a level chosen while Wi-Fi was stopped is applied when it comes back.
void rt_wifi_apply_power(void)
{
#if RT_STAGE >= 1
    // Report the request straight away, before trying to program anything. A stopped radio
    // still has a power it *will* use - g_pwr_dbm holds it and wifi_apply() programs it on the
    // next start - and if nothing reported that, changing Wi-Fi's power while it was stopped
    // looked exactly like the setting being ignored. The read-back below overwrites this with
    // the truth whenever the hardware is actually there to ask.
    rt_power_set_actual(CH_ESPNOW, rt_power_dbm(CH_ESPNOW));

    if (!s_wifi_on) {
        return;  // applied by wifi_apply() when the driver next starts
    }
    RT_TRY(TAG, esp_wifi_set_max_tx_power(rt_power_dbm(CH_ESPNOW) * 4));

    // Read back, because the hardware quantises the request onto a fixed ladder (see
    // esp_wifi.h) and what we asked for is usually not what we got. Truncating the quarter-dBm
    // toward zero is deliberate: reporting a power lower than the radio is really using would
    // be the dangerous direction to round.
    int8_t q = 0;
    if (esp_wifi_get_max_tx_power(&q) == ESP_OK) {
        rt_power_set_actual(CH_ESPNOW, (int8_t)((q + 3) / 4));
        ESP_LOGI(TAG, "wifi tx power %d.%02d dBm (asked %d)", q / 4, (q % 4) * 25,
                 rt_power_dbm(CH_ESPNOW));
    }
#endif
}

bool rt_wifi_active(void)
{
#if RT_STAGE >= 1
    return s_wifi_on;
#else
    return false;
#endif
}

void rt_set_lr(bool lr)
{
    if (lr == g_lr) {
        return;
    }
    g_lr = lr;
#if RT_STAGE >= 1
    wifi_apply();
#endif
    // The table is left alone - see RT_CMD_STATS_RESET in rt.h. This changes how this board
    // transmits, and what it had received is not this toggle's to discard.
}

// Called by rt_set_tests() after every change - see the comment on the declaration in rt.h for
// what a switch is actually supposed to mean at the radio level.
void rt_apply_test_radios(void)
{
#if RT_STAGE >= 1
    wifi_apply();
#endif
}

// The one gesture that must always work: back to exactly how the board boots.
//
// The BLE control link has no switch any more, so no test can turn it off - but the tests can
// still crowd it: every one of them takes antenna time from it, and a board with everything on
// at the edge of range is a board the phone struggles to reconnect to. Everything off is the
// state in which the control link has the antenna to itself, which makes it the restore. The
// page can then turn on whatever was wanted, from a link that works.
//
// LR goes too, since it is the setting that hides the AP from anything that is not an ESP, and
// the Wi-Fi AP is the control path this project expects to add next.
//
// It is the *only* hold, so there is nothing to overshoot into and nothing to release in time.
static void restore_control(void)
{
    ESP_LOGW(TAG, "button restore: every test off, LR off, coded PHY - as booted");
    rt_set_lr(false);
    // 2M is the short-range choice, so it is a way to lose the phone by walking away from it -
    // which makes putting it back part of what "restore every control channel" means. Same
    // rule as LR and the antenna: no command may strand the board somewhere this button
    // cannot reach.
    rt_set_conn_phy(false);
    // A board left muted by a command nobody remembers sending reads as a dead channel the
    // moment a test is turned back on.
    rt_set_tx_mute(false);
    rt_set_tests(0);
}

// What a tap selects next: nothing, then each test on its own, then nothing again. Single tests
// only - a combination is a thing to choose on the page, where you can see what you chose;
// counting taps in a pocket is for getting one radio going on a bench without a phone.
static unsigned next_solo(unsigned tests)
{
    static const uint8_t cycle[] = { 0, RT_TEST_ESPNOW, RT_TEST_BLE_ADV, RT_TEST_154,
                                     RT_TEST_FTM, RT_TEST_AP };
    const int n = sizeof(cycle) / sizeof(cycle[0]);
    for (int i = 0; i < n; i++) {
        if (cycle[i] == tests) {
            return cycle[(i + 1) % n];
        }
    }
    return 0;   // a combination from the page: a tap clears it, the next one starts the cycle
}

#if RT_STAGE >= 1
static void wifi_start(void)
{
    // The Wi-Fi driver dumps ~30 lines of per-MCS TX power tables and HE/iTWT chatter at
    // WARN on every esp_wifi_start(). That was tolerable when it happened once at boot; now
    // that a mode change stops and starts the driver, it buries the report every time you
    // press the button. Errors still come through.
    esp_log_level_set("wifi", ESP_LOG_ERROR);

    RT_TRY(TAG, esp_netif_init());
    RT_TRY(TAG, esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    RT_TRY(TAG, esp_wifi_init(&cfg));
    RT_TRY(TAG, esp_wifi_set_storage(WIFI_STORAGE_RAM));
    // The AP's config, kept for whenever the AP test turns it on - see wifi_apply(). The mode
    // itself is set there too: APSTA while the AP test is on, STA otherwise.
    wifi_config_t *ap = &s_ap_cfg;
    char ssid[sizeof(ap->ap.ssid) + 1];
    snprintf(ssid, sizeof(ssid), "%s", rt_node_name());
    memcpy(ap->ap.ssid, ssid, strlen(ssid));
    ap->ap.ssid_len       = strlen(ssid);
    ap->ap.channel        = WIFI_CHAN;
    ap->ap.authmode       = WIFI_AUTH_OPEN;   // throwaway instrument, not a product
    ap->ap.max_connection = 4;
    ap->ap.ftm_responder  = true;             // needs CONFIG_ESP_WIFI_FTM_ENABLE (sdkconfig.defaults)

    wifi_apply();  // started for ESP-NOW's init; app_main stops it again right after
}
#endif

// Power the antenna switch - last, deliberately, after every radio is up and every transmit
// power has been programmed.
//
// Ordering is the whole point. Until this runs, the switch is unpowered and the RF path is
// heavily attenuated: that is the state the boards have already spent hours in, so whatever it
// does to a PA, it has already done and been survived. Enabling it turns a clamped path into
// an open one, and doing that *before* power is set would put the one transmit this code
// cannot control - the few milliseconds between esp_wifi_start() and
// esp_wifi_set_max_tx_power(), which the API will not let us close - at up to +20dBm into a
// path whose match is still unproven. After this point every Wi-Fi restart still has that
// window, but by then it is into a known antenna, which is just ordinary transmitting.
//
// So: attenuated path takes the uncontrolled burst, and the good path only ever sees powers
// this firmware chose.
//
// Waiting on BLE specifically because rt_ble_apply_power() runs from the NimBLE sync callback,
// which is asynchronous - init returning does not mean the controller has been told anything.
// The timeout is a backstop, not an expectation; if it ever fires, that is worth knowing.
//
// GPIO14 (port select) is still never touched: R24 holds it at ground for RF1, the onboard
// ceramic antenna, which is the only one fitted.
// Always false at boot and never persisted - see the note in rt.h.
volatile bool g_ant_ext;

// Set once the pin has been configured, so a command arriving before the switch is up cannot
// drive a pin that is still an input.
static bool s_ant_ready;

// settle: only at boot, where the switch has just been given power and the vendor's own
// sequence waits before using it. A later port change does not need it - the switch itself
// settles in microseconds - and rt_set_antenna() runs on the NimBLE host task, which should
// not be blocked for 100ms in the middle of a GATT write.
static void apply_antenna(bool settle)
{
    if (!s_ant_ready) {
        return;
    }
    gpio_set_level(ANT_SEL_GPIO, g_ant_ext ? 1 : 0);
    if (settle) {
        vTaskDelay(pdMS_TO_TICKS(ANT_SETTLE_MS));
    }
    ESP_LOGI(TAG, "antenna: %s", g_ant_ext ? "external (U.FL)" : "internal (chip)");
}

void rt_set_antenna(bool external)
{
    if (external == g_ant_ext) {
        return;
    }
    g_ant_ext = external;
    apply_antenna(false);
    // A different antenna is a different link, and the numbers on either side of this are not
    // comparable - but saying so is the operator's job now, with the reset button, not
    // something to do to their table on their behalf. See RT_CMD_STATS_RESET in rt.h.
}

// ---- User LED ----------------------------------------------------------------------------
//
// Off, on, or blinking at 2Hz. Off at boot, and off means floating rather than driven - see
// LED_GPIO at the top of this file for that and for the polarity.
//
// Unlike the antenna there is no readiness flag and no ordering constraint: GPIO15 goes
// nowhere near the RF path, so the pin can be configured the moment it is first asked for and
// never needs touching before that.
volatile uint8_t g_led;

// Created on first use and then kept, because the cycle is a button someone is pressing: a
// timer torn down and rebuilt on every pass through blink is three allocations per cycle for
// no gain, and the handle is four bytes.
static esp_timer_handle_t s_led_timer;
static bool               s_led_lit;   // which half of the blink period the pin is in

// Runs on the esp_timer task. One gpio_set_level and nothing else - anything that could block
// does not belong on that task, which every other timer on the board shares.
static void led_blink_cb(void *pv)
{
    (void)pv;
    s_led_lit = !s_led_lit;
    gpio_set_level(LED_GPIO, s_led_lit ? LED_ON_LEVEL : LED_OFF_LEVEL);
}

// Named in one place, so the log line and the serial report cannot drift apart or disagree
// about what mode 2 is called.
const char *rt_led_name(int mode)
{
    switch (mode) {
    case RT_LED_ON:    return "on";
    case RT_LED_BLINK: return "2Hz";
    default:           return "off";
    }
}

static void apply_led(void)
{
    // Stopping first means every mode is entered from the same place, whichever one it is
    // leaving. Not-running is not an error worth reporting here.
    if (s_led_timer != NULL) {
        esp_timer_stop(s_led_timer);
    }

    const gpio_config_t io = {
        .pin_bit_mask = 1ULL << LED_GPIO,
        // Driven in both lit modes, including the dark half of a blink: while the blink is
        // running the pin is in use, and releasing it every other half period would hand the
        // LED to whatever leakage is nearby instead of turning it off. Only the off mode
        // releases the pin, so off is byte-for-byte the state the board powered up in.
        .mode         = g_led == RT_LED_OFF ? GPIO_MODE_INPUT : GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&io);

    if (g_led != RT_LED_OFF) {
        // Both lit modes start lit, so a press always answers immediately - a blink that began
        // on its dark half would look for a quarter second like the press did nothing.
        s_led_lit = true;
        gpio_set_level(LED_GPIO, LED_ON_LEVEL);
    }

    if (g_led == RT_LED_BLINK) {
        if (s_led_timer == NULL) {
            const esp_timer_create_args_t args = {
                .callback = led_blink_cb,
                .name     = "led",
            };
            if (esp_timer_create(&args, &s_led_timer) != ESP_OK) {
                // Steady light is the honest fallback: the LED is lit, which is what the
                // report will say, rather than claiming a blink that is not happening.
                ESP_LOGW(TAG, "no timer for the LED blink; leaving it lit");
                g_led = RT_LED_ON;
            }
        }
        if (s_led_timer != NULL) {
            esp_timer_start_periodic(s_led_timer, (uint64_t)LED_HALF_MS * 1000);
        }
    }

    ESP_LOGI(TAG, "led: %s", rt_led_name(g_led));
}

void rt_set_led(int mode)
{
    // Out of range is ignored rather than clamped, same as every other command byte: a byte
    // this firmware does not understand should do nothing, not the nearest thing to something.
    if (mode < 0 || mode >= RT_LED_COUNT || (uint8_t)mode == g_led) {
        return;
    }
    g_led = (uint8_t)mode;
    apply_led();
}

static void antenna_switch_on(void)
{
    const uint32_t deadline = rt_ms() + ANT_PWR_WAIT_MS;
    while (!rt_ble_power_ready() && (int32_t)(rt_ms() - deadline) < 0) {
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    if (!rt_ble_power_ready()) {
        ESP_LOGW(TAG, "BLE never reported its tx power; powering the antenna anyway");
    }

    const gpio_config_t ant = {
        .pin_bit_mask = 1ULL << ANT_PWR_GPIO,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&ant);
    gpio_set_level(ANT_PWR_GPIO, 0);
    vTaskDelay(pdMS_TO_TICKS(ANT_SETTLE_MS));

    // Port select after the switch has power, matching the vendor's own sequence. VCTL is a
    // logic input on a part whose supply has only just come up, and driving an input of an
    // unpowered device is the kind of thing that is usually fine and occasionally is not.
    // R24 holds it low meanwhile, so the switch powers into the internal antenna regardless.
    const gpio_config_t sel = {
        .pin_bit_mask = 1ULL << ANT_SEL_GPIO,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&sel);
    s_ant_ready = true;
    apply_antenna(true);

    ESP_LOGI(TAG, "RF switch powered (GPIO%d low), tx power already set: "
                  "espnow %ddBm, ble %ddBm, 154 %ddBm", ANT_PWR_GPIO,
             rt_power_actual(CH_ESPNOW), rt_power_actual(CH_BLE_ADV), rt_power_actual(CH_154));
}

// Tap (release before HOLD_MS) steps to the next single test - see next_solo(); hold past HOLD_MS
// restores. Two gestures, no windows - see the HOLD_MS comment at the top of the file.
// Polled rather than interrupt-driven, same as always: a button needs debouncing anyway.
static void button_task(void *pv)
{
    (void)pv;
    const gpio_config_t io = {
        .pin_bit_mask = 1ULL << BUTTON_GPIO,
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&io);

    bool     down = false;
    uint32_t t_down = 0;
    bool     fired = false;

    for (;;) {
        const bool now_down = gpio_get_level(BUTTON_GPIO) == 0;

        if (now_down && !down) {
            down   = true;
            fired  = false;
            t_down = rt_ms();
        } else if (now_down && down && !fired && (rt_ms() - t_down) > HOLD_MS) {
            // Fires while the button is still down. Keep holding as long as you like; there is
            // no release to time and nothing further happens.
            fired = true;
            restore_control();
        } else if (!now_down && down) {
            down = false;
            if (!fired) {
                rt_set_tests(next_solo(g_tests));
            }
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void app_main(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        err = nvs_flash_init();
    }
    if (err != ESP_OK) {
        // Nothing here stores anything - Wi-Fi config is kept in RAM - so a bad NVS
        // partition is worth a complaint, not a boot loop.
        ESP_LOGW(TAG, "nvs_flash_init -> %s (continuing)", esp_err_to_name(err));
    }

    esp_chip_info_t info;
    esp_chip_info(&info);
    ESP_LOGI(TAG, "%s rev v%d.%d, node %06lX", CONFIG_IDF_TARGET,
             info.revision / 100, info.revision % 100, (unsigned long)rt_node_id());

    // Why the last reset happened, printed before anything else can obscure it.
    //
    // A board that resets mid-test looks identical from the outside whatever the cause - the
    // link drops, it comes back on defaults, and the numbers stop making sense. This is the
    // one place that says which it was. BROWNOUT in particular is the answer to "does it
    // reset at maximum transmit power", and it is a question guessing cannot settle: a
    // transmit-current brownout, a watchdog and a crash all present the same way.
    // Name and code are set together, and the page holds the matching list. Adding a reason
    // means touching both halves of this and docs/index.html - which is why they are one
    // statement each rather than two tables that can drift apart.
    const esp_reset_reason_t why = esp_reset_reason();
    switch (why) {
    case ESP_RST_POWERON:   s_reset_reason = "power-on";   s_reset_code = 1;  break;
    case ESP_RST_SW:        s_reset_reason = "sw-restart"; s_reset_code = 2;  break;
    case ESP_RST_PANIC:     s_reset_reason = "PANIC";      s_reset_code = 3;  break;
    case ESP_RST_INT_WDT:   s_reset_reason = "INT-WDT";    s_reset_code = 4;  break;
    case ESP_RST_TASK_WDT:  s_reset_reason = "TASK-WDT";   s_reset_code = 5;  break;
    case ESP_RST_WDT:       s_reset_reason = "WDT";        s_reset_code = 6;  break;
    case ESP_RST_BROWNOUT:  s_reset_reason = "BROWNOUT";   s_reset_code = 7;  break;
    case ESP_RST_EXT:       s_reset_reason = "ext-pin";    s_reset_code = 8;  break;
    case ESP_RST_DEEPSLEEP: s_reset_reason = "deepsleep";  s_reset_code = 9;  break;
    case ESP_RST_USB:       s_reset_reason = "usb";        s_reset_code = 10; break;
    default:                s_reset_reason = "unknown";    s_reset_code = 0;  break;
    }
    ESP_LOGW(TAG, "last reset: %s (%d)", s_reset_reason, (int)why);

    ESP_LOGI(TAG, "stage %d (raise RT_STAGE in platformio.ini to add radios)", RT_STAGE);

#if RT_STAGE >= 1
    // The packet log first, before any radio has allocated anything, so it is one clean block
    // that nothing later can fragment around. Fixed size - see RT_LOG_BYTES.
    ESP_LOGI(TAG, "init: log");
    rt_log_init();
#endif

    // Brought up one at a time with a line before each, so if anything does take the board
    // down the last line printed names the culprit.
#if RT_STAGE >= 1
    ESP_LOGI(TAG, "init: wifi");
    wifi_start();
    ESP_LOGI(TAG, "init: espnow");
    rt_espnow_start();
    // Every test is off at boot, so the driver has no reason to stay up past ESP-NOW's init.
    s_wifi_boot_hold = false;
    wifi_apply();
#endif
#if RT_STAGE >= 2
    ESP_LOGI(TAG, "init: ble");
    rt_ble_start();
#endif
#if RT_STAGE >= 3
#if SOC_IEEE802154_SUPPORTED
    ESP_LOGI(TAG, "init: 802.15.4");
    rt_154_start();
#else
    ESP_LOGW(TAG, "no 802.15.4 radio on this target - that channel is disabled");
#endif
#endif
#if RT_STAGE >= 1
    ESP_LOGI(TAG, "init: ftm");
    rt_ftm_start();
    ESP_LOGI(TAG, "init: gnss");
    rt_gnss_start();
#endif
    ESP_LOGI(TAG, "init: done");

    antenna_switch_on();

    xTaskCreate(button_task, "button", 3072, NULL, 5, NULL);

    ESP_LOGI(TAG, "running, every test off. GPIO9: tap = next single test, hold = RESTORE "
                  "(every test off, LR off). Combinations are set from the phone.");

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(REPORT_MS));
#if RT_STAGE == 0
        // No radios to report on yet - just prove the board is alive and stays alive.
        ESP_LOGI(TAG, "alive %lus (stage 0, no radios)",
                 (unsigned long)(rt_ms() / 1000));
#else
        rt_report();
#endif
#if RT_STAGE >= 4
        rt_ui_notify();
#endif
    }
}

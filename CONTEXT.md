# EspRangeTest — design context and hard-won findings

Written so any Claude session (or person) picking this up has the reasoning, not just the
code. Most of what follows was established by the project owner or by testing, and is not
recoverable by reading the source.

---

## The point of the project

**Which of the ESP32-C6's radios and PHYs still gets a nonzero amount of data through at the
greatest distance?**

It is a throwaway instrument, not a product. It should be testable in the real world as early
as possible — flash two boards, walk away with one, read numbers. No test suites, no
frameworks, no abstraction that does not pay for itself immediately.

**The only metric that matters is nonzero at max range** — a barely-there signal that still
gets a packet through beats a strong one that doesn't reach as far. Throughput, data rate,
and link speed are not goals here and never have been; nothing in this project should be
read as caring about them, including on channels (BLE coded, 802.15.4) whose own design
trades rate for range. This applies uniformly across every link kind measured, including
future ones like FTM — the question is always "did anything arrive," never "how much" or
"how fast."

### The central question: BLE coded PHY S=8

The working hypothesis is that BLE coded S=8 wins. It has a trap:

- S=8 is only reliably negotiated **after a connection is established**.
- A connection can only be **established while close**.
- So if you walk out, lose the connection, and adverts are not really going out at S=8, that
  loss is **permanent** — and BLE coded is not usable for the thing we want it for.

Therefore the pivotal test is whether **adverts alone**, with no connection, carry data at
S=8 range. That is why `ble_adv` is a non-connectable beacon and is measured separately from
anything connection-oriented.

**We cannot read back which coding was used.** Distinguishing S=2 from S=8 in a received
advert requires Bluetooth 5.4 Advertising Coding Selection; the C6 is 5.3. The controller
reports only "coded". **Measured range is the ground truth, not a status field.** If
`ble_adv` outlives `espnow` and `154` by a wide margin it is doing S=8; if it dies at a
similar distance, it is not.

---

## Decisions the owner made (do not silently revisit these)

**Scope**
- Stateless broadcasts are the primary measurement: ESP-NOW, BLE adverts, AP beacons, FTM,
  802.15.4. Send on a timer, listen the rest of the time.
- Connection-oriented links are worth measuring too, but separately: 90% loss looks very
  different at a 500 ms advert interval than at a 30 ms connection interval where the link
  layer is silently retransmitting.
- Connection cycling exists because **connection slots are scarce**, not for record-keeping.
  "Working means working" — success is the metric, not volume. Must work with 2 boards and
  scale to more peers than can be connected at once.

**Radio isolation**
- Low-contention mode means "turn off everything but X" (or X plus a UI channel) — and
  "everything" means anything that touches the radio, not just the other two channels' own
  measurement packets. That includes **stopping the Wi-Fi driver**, not merely muting ESP-NOW's
  send loop. (This drifted repeatedly: the code called it "solo mode" and for a long time only
  gated the three TX loops, leaving BLE's continuous scanner, the phone UI, and then a
  beaconing SoftAP running unchanged underneath. Fixed over three passes — see Coexistence
  below. If you see "solo" anywhere, it's stale, not a second concept.)
- The one exception is the BLE phone link, which is the control channel and has no separate
  physical toggle, so it stays up in every mode except `LC_WIFI_UI` — throttled (slow advert,
  slow connection interval, half-rate notifications) rather than silenced.
- `rt_tx_enabled()` is **not** the definition of a mode; it answers only the narrow question
  the three TX loops ask. `rt_apply_lc_radios()` in `main.c` is the definition. Reading the
  former as the latter is precisely the bug above.
- **Operator-commanded only. No scheduler, no automatic cycling.** A mode stays until
  changed. The workflow is: command a switch, run a test, command another.
- Most useful modes are "just X plus phone BLE" — first establish whether BLE-on hurts X at
  all, then do the real testing with BLE on.
- GPIO9 (BOOT button): tap cycles low-contention mode, long press returns to all-on, very long
  hold (`LR_HOLD_MS`) toggles LR. One button, three tiers — there is no separate BLE toggle,
  which is why the phone link is throttled rather than switched off.

**802.15.4**
- Raw frames only. **No Thread, no Zigbee.** Both ride the identical PHY (2.4 GHz O-QPSK,
  250 kbps), so the range result is the same. The stacks only add addressing, routing and a
  join procedure that can fail at the far end for reasons unrelated to radio range. The owner
  does not care about mesh or multi-hop.

**What gets measured**
- Per **peer**, per **channel** (channel = link kind, not RF channel): latest RSSI, rolling
  average RSSI, and missed sequence numbers in a rolling window — including when the last
  reception is old, because silence must read as loss, not as absence of data.
- **Contention matters on the receiver**, especially for packets that did *not* arrive.
  TX-side contention is moot on a packet that was successfully sent.
- No flash logging: flash writes stall the system, which is its own contention source. Keep
  the last ~20 observations per peer per channel in RAM.
- Persistent logs are useless without knowing where you were, so history belongs in the
  browser alongside the GPS fix. Focus is realtime while walking.

**UI**
- BLE is the primary transport. Wi-Fi exists as a UI transport only for testing with BLE off
  (including testing without a phone, since the phone's own BLE connection is contention).
  Do not treat them as different capability tiers.
- Browser geolocation is wanted. Most whole-network state lives in JS; the boards are dumb
  and report only what they directly hear.
- The page must be able to connect to **several boards at once**.
- Node "role" is purely a UI display concept: place manually, "pick up" (node follows the
  phone's GPS as you carry it), or "put down" (stamp its position to the current fix).
- BLE **legacy** advertising is not a measurement. It exists only so Chrome can discover a
  board — Chrome's scanner is legacy-only and cannot see extended or coded adverts.

**Future, explicitly not now**
- Relaying another board's observations over a single ESP-NOW / 802.15.4 / BLE hop. For now
  each board reports only its own direct observations.

---

## Hardware

| | |
|---|---|
| ESP32-C6-DevKitC-1 | WROOM-1 module, COM23, `-e devkitc` |
| ESP32-C6-DevKitM-1 | MINI-1 module, COM25, `-e devkitm` |

The two modules have **different antenna designs, and comparing them is a goal** — so board
identity matters in results. A third C6 is arriving; C3 and S3 boards exist as spares (no
802.15.4 radio on those). A phone is an inconsistent third point; nothing may depend on it.

---

## Platform findings

**LR mode.** If `WIFI_PROTOCOL_LR` is in the protocol list at init — merely *present*, not
LR-only — the SoftAP becomes invisible to phones. Established by the owner's earlier testing.
Entering or leaving LR needs a Wi-Fi reinit, not a live protocol change. Corollary: whenever
a phone can associate to the AP, ESP-NOW was running non-LR. Other ESPs can still reach an
LR AP. LR is a PHY affecting both Wi-Fi and ESP-NOW, which share the stack.

**Coexistence.** Wi-Fi, BLE and 802.15.4 share one RF path and one antenna, arbitrated by
time (software PTI arbitration, not a physical switch). The installed ESP-IDF
(`esp_ieee802154_util.c`) hardcodes 802.15.4's own ordinary TX/RX at the *lowest* of four PTI
tiers (`IEEE802154_LOW`, below `MIDDLE` and `HIGH`) — so any higher-priority WiFi or BLE
request wins the antenna over it, every time, not probabilistically. The per-scene priority API
(`esp_ieee802154_set_coex_config()`) **is now used** — see the coex-priority note further down.

Measured, not just expected: with ESP-NOW and BLE both running, **every single 802.15.4
transmit is rejected**, immediately, from the very first packet - `esp_ieee802154_transmit_failed`
fires with `ESP_IEEE802154_TX_ERR_COEXIST` at 100%, not intermittently. This was invisible
until now because that callback runs in ISR context (`ieee802154_isr` calls it directly) and
the original code discarded the error silently - the same "swallowed failure looks identical
to a genuinely untested channel" trap as everywhere else in this project. In `all` mode,
802.15.4 is not "disadvantaged", it is **completely absent**.

**The dedup trap in the fix above:** `rt_154.c`'s ISR handler only logs when the error *type*
changes (`if (error != s_last_tx_err)`), on purpose — `ESP_LOGW` from ISR context takes a
newlib lock and would eventually hang. Side effect: once coexist starts failing, it logs
**once** and then goes quiet even though every subsequent transmit is still failing — the
running `tx: ... 154=N(N failed)` counter in the periodic report is the only place that
keeps counting. Reading "I only saw the error once" off the log line, instead of the
counter, looks exactly like a one-time init fault that stopped happening. It didn't stop;
the log just stopped repeating itself.

**Low contention didn't actually isolate anything — this was the real bug.** The mode existed
(then called "solo") but only gated the three measurement TX loops via `rt_tx_enabled()`,
leaving continuous, unrelated sources of radio time running underneath regardless of which
channel was selected. Three separate passes were needed; the first two were incomplete.

*Pass 1 — the BLE scanner.* `rt_ble.c`'s coded-PHY scanner (`ble_gap_ext_disc` with
`window == interval`) is a **100% duty-cycle receiver** — it asks for the antenna permanently,
not occasionally, unlike everything else on this board (adverts every 500ms, ESP-NOW every
250ms). Fixed: the scanner runs only while `ble_adv` is the thing actually being measured
(`rt_tx_enabled(CH_BLE_ADV)`), same condition as the coded beacon's own TX gate, and `on_sync`
applies the same gate rather than arming it unconditionally at boot.

*Pass 2 — the phone UI.* The legacy advert/GATT connection (`rt_ui.c`) is needed for the
interface/control path regardless of what's under test, so it stays up rather than stopping —
but at a slower advert interval and requesting a slower/longer connection interval whenever any
channel is isolated. Also (added in pass 3) it now sends its notification burst every *other*
report in those modes: one report is up to 19 notifications, and a slower connection interval
does not reduce that traffic, it only bunches it into fewer, longer connection events. A laggy
phone link is fine; low-contention testing of anything other than `ble_adv` is realistically a
bench-test, serial-output affair, not a live walk with the phone connected.

*Pass 3 — Wi-Fi was never actually stopped, and the SoftAP made that much worse.* Gating
`esp_now_send()` does nothing to the antenna: the driver stays up, and since the AP + LR work
below made Wi-Fi `APSTA`, the SoftAP beacons every ~100ms and keeps its receiver on
continuously to hear probes — both above 802.15.4 in the coex arbiter. The earlier note here
claiming "idle WiFi STA ... asks for nothing at all between actual TX/RX events" was true of
STA-only and became wrong the moment the AP was added, in the same commit; 802.15.4 stayed at
`TX_ERR_COEXIST` 100% even in "154-only" mode. Fixed: `main.c`'s `wifi_apply()` **stops the
Wi-Fi driver outright** (`esp_wifi_stop()`) in any mode that isolates a non-Wi-Fi channel, and
starts it again on the way back. It is idempotent and is the single path for both mode changes
and the LR toggle, since both want the same stop/reconfigure/start and doing it twice for one
button press would needlessly bounce the AP. `rt_espnow_resume()` re-adds the broadcast peer
after each start — the peer list belongs to the ESP-NOW module and does survive
`esp_wifi_stop()`, so this normally returns `ESP_ERR_ESPNOW_EXIST`; it is there so the code
does not *depend* on that being true.

The report header now prints `wifi=on/off` — the Wi-Fi **driver**, distinct from the ESP-NOW
`(off)` tx-gate marker on the line below. The failure mode this project keeps hitting is a
radio quietly still on while the numbers imply otherwise, so that state has to be visible.

**802.15.4's coex priority is now raised explicitly.** `rt_154.c`'s `apply_coex_pti()` calls
`esp_ieee802154_set_coex_config()`: `IEEE802154_HIGH` while 802.15.4 is the channel under test,
`IEEE802154_MIDDLE` otherwise, instead of the IDF default `IEEE802154_LOW`. Stopping the other
radios is the actual fix; this is the belt to those braces, so that what necessarily stays up
(the phone control link) cannot shut 802.15.4 out again. Note what it does *not* mean: "all
radios" numbers are still contended, just contended rather than pre-decided — a run where every
802.15.4 frame is rejected measures the arbiter, not the range.

Renamed solo → **low contention** throughout (code, docs, UI) to match how it was described in
planning and avoid two names drifting apart again. `g_solo`/`rt_set_solo` are now
`g_lc`/`rt_set_lc`.

**Wi-Fi AP + LR toggle + `LC_WIFI_UI`.** WiFi is now APSTA, not STA-only: an open `ESPRT-XX`
AP (`ftm_responder = true`) exists alongside the STA side ESP-NOW already used, so a phone can
reach a board over Wi-Fi instead of BLE. `LC_WIFI_UI` is a fifth low-contention state (GPIO9
tap cycles through it) that keeps ESP-NOW/the AP up, forces BLE fully off, and forces LR off
(the AP is invisible to phones with `WIFI_PROTOCOL_LR` present at all — see LR mode above) -
`rt_ui.c`'s `start_adv()` now refuses to arm BLE advertising in this state so nothing can
re-enable it from underneath. LR itself is a separate, independent long-hold toggle
(`LR_HOLD_MS`), not part of the low-contention cycle, since it needs a full Wi-Fi
stop/reconfigure/start (see LR mode above) and is meant to be an infrequent, deliberate choice
rather than something every mode switch touches.

**FTM.** Errata WIFI-9686 says the C6 cannot be an FTM initiator (T3 unreadable). The owner
reports it working on this hardware, which is v0.2 silicon. Treat initiator support as
present but verify per board.

**Web Bluetooth / hosting.** Web Bluetooth needs a secure context, so a `file://` page will
not work — use GitHub Pages or `localhost`. An HTTPS page **cannot** reach the board over
`ws://` *or* `http://`; mixed content blocks both. That is why the two transports imply two
origins: HTTPS page → BLE only; board-hosted HTTP page → WebSocket/HTTP only.

**PlatformIO.** The widely repeated "PlatformIO does not support the ESP32-C6" is about the
**Arduino** framework. Both devkits have official board definitions listing `espidf` as
supported, so the official `platform = espressif32` works. The pioarduino fork is only needed
for Arduino.

---

## Build-configuration traps (all of these cost real time)

Every failure on this project so far has been in the build-config layer, not the radio code.
The pattern is always the same: **PlatformIO and ESP-IDF each think they own a setting.**

1. **Console routing.** `CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG` sends the app log *and* panic
   backtraces to the native USB socket. Monitoring the UART bridge then shows only ROM output
   — which looks exactly like a silent reboot loop. Now `CONFIG_ESP_CONSOLE_UART_DEFAULT`
   with USB-JTAG as secondary.
2. **Partition table.** PlatformIO takes it from the board manifest and **ignores**
   `CONFIG_PARTITION_TABLE_*`. The default gave a 1 MB app partition for a ~1.15 MB image;
   the bootloader rejected it and reset forever. Now an explicit `partitions.csv` that *both*
   `platformio.ini` and `sdkconfig.defaults` point at.
3. **Flash size / MMU page size.** The C6 has a configurable flash MMU page size that ESP-IDF
   derives from the flash size. The bootloader validates the app header against its own page
   size and aborts with `Invalid app image header` on a mismatch. Flash size is now pinned in
   `sdkconfig.defaults` and matched by `board_build.flash_size` in `platformio.ini`. Do **not**
   also pin the page size — let it derive, so both sides derive the same value.
4. **`ESP_ERROR_CHECK` everywhere.** One unhappy init call became a panic and a boot loop that
   named nothing. Use `RT_TRY` (in `rt.h`): log the failing call and continue. Two working
   radios still make a useful walk.
5. After changing `sdkconfig.defaults` or `partitions.csv`, run `pio run -e <env> -t fullclean`.
   The bootloader is built from the same config, and a stale bootloader with a fresh app is
   its own class of failure.

**`RT_STAGE`** in `platformio.ini` exists because of the above: 0 heartbeat only, 1 +Wi-Fi and
ESP-NOW, 2 +BLE, 3 +802.15.4, 4 +phone UI. Raise one step at a time. **If stage 0 fails, no
radio code is running at all — the problem is build configuration.**

---

## Working method

The cloud session writing this **cannot compile, flash, or read a serial port**. Every build
error and boot failure above cost a full round trip through the owner. Any session that *can*
run `pio run` should do the build-flash-read loop; it collapses hours into seconds.

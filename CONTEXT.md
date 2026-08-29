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
- The one exception is the BLE phone link: it is the control channel, so it stays up in every
  mode except `LC_WIFI_UI` — throttled (slow advert, slow connection interval) rather than
  silenced, and still carrying every report. Modes that kill a control channel are allowed
  precisely because the button restores them.
- `rt_tx_enabled()` is **not** the definition of a mode; it answers only the narrow question
  the three TX loops ask. `rt_apply_lc_radios()` in `main.c` is the definition. Reading the
  former as the latter is precisely the bug above.
- **Operator-commanded only. No scheduler, no automatic cycling.** A mode stays until
  changed. The workflow is: command a switch, run a test, command another.
- Most useful modes are "just X plus phone BLE" — first establish whether BLE-on hurts X at
  all, then do the real testing with BLE on.
- **GPIO9 (BOOT button) is the recovery path and that is its first duty.** Every other control
  can be taken away by the modes the button itself selects: `LC_WIFI_UI` turns BLE off, the
  `ble_adv` and `154` modes stop Wi-Fi, LR makes the SoftAP invisible to a phone, and the web
  UI can command any of them. It is fine for the website to break the control channels — the
  button is what brings them back. So a hold is a full **restore**: all radios on *and* LR
  cleared. Clearing LR is part of it, not a bonus; without it a restore leaves the phone still
  unable to reach the AP.
- **Two gestures, and never a third.** Tap (= released before `HOLD_MS`; the tap duration is
  just debounce, not something to hit) cycles low-contention mode. Hold past `HOLD_MS` fires
  the restore *while still held*, with no upper bound. `HOLD_MS` is a floor, not a window.
  A third, longer tier is forbidden, because adding one silently converts the middle tier into
  a bounded window that has to be timed by guesswork — and this has to work blind, in a
  pocket, by someone who has just lost every other control. Long and short are distinguishable
  without a clock; "medium" is not.
- Consequently the LR toggle is **not** on the button any more (it was a 3s super-hold). It is
  a command-channel byte, `RT_CMD_LR_OFF` / `RT_CMD_LR_ON` (0x80/0x81) written to the GATT
  command characteristic. Safe there precisely because holding the button undoes it: no
  command can strand a board that the physical control cannot take back. Values outside
  `0..LC_COUNT-1` and the two LR bytes are ignored, so an older web UI keeps working.

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
LR-only — the SoftAP becomes invisible to phones and to non-ESP devices generally. Established
and re-confirmed by the owner. This is why LR is a control-channel hazard, not just a PHY
option: Wi-Fi is the fallback control path for when BLE is off, so LR on + BLE off = no way in
except the button.
Entering or leaving LR needs a Wi-Fi reinit, not a live protocol change. Corollary: whenever
a phone can associate to the AP, ESP-NOW was running non-LR. Other ESPs can still reach an
LR AP. LR is a PHY affecting both Wi-Fi and ESP-NOW, which share the stack.

**Coexistence.** Wi-Fi, BLE and 802.15.4 share one RF path and one antenna, arbitrated by
time (software PTI arbitration, not a physical switch). The installed ESP-IDF
(`esp_ieee802154_util.c`) hardcodes 802.15.4's own ordinary TX/RX at the *lowest* of four PTI
tiers (`IEEE802154_LOW`, below `MIDDLE` and `HIGH`) — so any higher-priority WiFi or BLE
request wins the antenna over it, every time, not probabilistically. The per-scene priority API
(`esp_ieee802154_set_coex_config()`) exists and is **deliberately not used** — see
"Priorities stay at the IDF default" below.

Treat Espressif's coexistence duty-cycle figures (the "50% each" numbers) as unverified here.
The owner's read is that they predate the C6's 802.15.4 radio and probably describe 50% AP /
50% STA / 50% BLE as separate claims. Nothing in this project should be justified by them.

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
interface/control path regardless of what's under test, so it stays up rather than stopping.
**It runs slow in exactly two modes — `espnow` and `154` — and fast everywhere else.** Those
two are the only ones whose point is handing airtime to a non-BLE channel, and they are
bench-test modes where a laggy phone costs nothing.

An intermediate revision made slow the default in *every* mode, including `lc=0`. That was
wrong, for three separate reasons, all worth keeping written down:
- `lc=0` is the normal walking mode, where boards and phone **continually connect and drop**
  as they move in and out of range. Every reconnect starts with seeing an advert, so a 1s
  advert interval is a 1s-plus stall on each one. Constant re-establishment is the normal case
  here, not an edge case.
- `ble_adv` is the BLE test itself — partly about whether a connection can be established and
  held at all, and it asks for coded S=8 on the connection. Throttling it handicaps the thing
  under test.
- A slow connection interval means nothing can be sent *at all* until the next connection
  event. Slowing it where results are wanted promptly does not make reports cheaper, it makes
  them late or missing.

Note the two adverts are separate instances and are not affected by each other: `UI_INSTANCE`
in `rt_ui.c` is the 1M control link, `ADV_INSTANCE` in `rt_ble.c` is the coded-PHY measurement
beacon. Nothing in the UI cadence work has ever changed the beacon's interval.

**Reporting is never skipped.** An earlier revision of pass 3 dropped every other notification
burst in low-contention modes to save airtime. That was wrong and is reverted: a run whose
numbers were not delivered did not happen, and a low-contention mode whose results never
arrive is the most expensive kind of nothing. Airtime is bought by *slowing* the link
(intervals, advert rate), never by dropping data off it — carrying the results is the
cadence's job, not an optional extra. Budget: well under 5% duty is the target, ~1% if
practical, and a ~500ms connection interval is about right for that.

**What a report actually costs, measured from the format strings** (not from the array size,
which is where an earlier "1.2 kB" claim in conversation came from — `rt_snapshot_lines()`
only emits rows for peers actually seen, so `RT_MAX_PEERS * CH_COUNT` is capacity, not
traffic):

| setup | lines | bytes/report | rate at 2s | packets/s |
|---|---|---|---|---|
| 2 boards (1 peer × 3 channels + status) | 4 | 169 | 85 B/s | 2.0 |
| 6 peers (most the array allows) | 19 | 956 | 478 B/s | 9.5 |

Against a practical coded-PHY ceiling of ~5 kB/s that is ~1.7% for the two-board case and
~10% at six peers. So the format is not currently a problem at two boards, and packing is a
six-peer concern rather than an everyday one. If it is ever wanted: four lines fit in one
247-byte ATT notification, which would cut the two-board case from 4 packets to 1 — worth more
than the byte count on coded PHY, where per-packet cost dominates. It requires `docs/index.html`
to split incoming notifications on newline, and **the page must be redeployed before firmware
that packs is flashed**, or the UI silently stops parsing.

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

**Priorities stay at the IDF default — decided, not overlooked.** Raising 802.15.4's coex tier
was tried and reverted. The reasoning, from the owner: Wi-Fi and BLE are not greedy, they are
*scheduled* — a connection event or a beacon has to happen at its moment or the link degrades,
and 802.15.4's normal duty cycle is 100% receive with occasional transmits, so promoting it
does not share the antenna more fairly, it hands 802.15.4 everything. Isolation has to come
from other radios genuinely being off or backed off; changing priorities is "a different way to
achieve it" and a worse-behaved one — invisible, surfacing only as somebody else's link going
flaky, with no button that restores it. **Loss of control is game over.** If a mode needs the
antenna quiet, stop things explicitly or lengthen their intervals; do not re-rank the arbiter.

**Every rejection is now reported, continuously.** The old ISR handler latched only a *change*
of error type, so a radio being refused four times a second logged one line at boot and then
went silent — indistinguishable from a one-off hiccup, and useless. `rt_154.c` now counts
rejections per reason in the ISR (all it may safely do there) and `tx_task` prints totals plus
the per-window delta every 2s for as long as anything is still failing:
`W 154: tx rejected: coexist 128(+8)`. Silent only when the radio genuinely is not failing.

Renamed solo → **low contention** throughout (code, docs, UI) to match how it was described in
planning and avoid two names drifting apart again. `g_solo`/`rt_set_solo` are now
`g_lc`/`rt_set_lc`.

**Wi-Fi AP + LR toggle + `LC_WIFI_UI`.** WiFi is now APSTA, not STA-only: an open `ESPRT-XX`
AP (`ftm_responder = true`) exists alongside the STA side ESP-NOW already used, so a phone can
reach a board over Wi-Fi instead of BLE. `LC_WIFI_UI` is a fifth low-contention state (GPIO9
tap cycles through it) that keeps ESP-NOW/the AP up, forces BLE fully off, and forces LR off
(the AP is invisible to phones with `WIFI_PROTOCOL_LR` present at all — see LR mode above) -
`rt_ui.c`'s `start_adv()` now refuses to arm BLE advertising in this state so nothing can
re-enable it from underneath. LR itself is a separate, independent toggle, not part of the
low-contention cycle, since it needs a full Wi-Fi stop/reconfigure/start (see LR mode above)
and is meant to be an infrequent, deliberate choice rather than something every mode switch
touches. It lives on the command channel (`RT_CMD_LR_OFF` / `RT_CMD_LR_ON`), not the button —
see the button rules under Radio isolation.

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

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
- Low-contention mode means "turn off everything but X" (or X plus a UI channel).
- **Operator-commanded only. No scheduler, no automatic cycling.** A mode stays until
  changed. The workflow is: command a switch, run a test, command another.
- Most useful modes are "just X plus phone BLE" — first establish whether BLE-on hurts X at
  all, then do the real testing with BLE on.
- GPIO9 (BOOT button): tap cycles solo mode, long press returns to all-on.

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
time. 802.15.4 normal RX is assigned the *lowest* priority, so Wi-Fi and BLE steal the radio
whenever they want. Assume coex mostly works and BLE can stay on, but expect 802.15.4 to
suffer most when everything runs together — quantifying that is what solo mode is for.

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

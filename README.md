# EspRangeTest

An instrument for answering one question: **which of the ESP32-C6's radios and PHYs gets a
nonzero amount of data through at the greatest distance?**

The working hypothesis is BLE coded PHY at S=8, but S=8 has a trap. It is only reliably
negotiated *after* a connection is established, and connections can only be established
while close. If you walk out, lose the connection, and coded adverts don't actually go out
at S=8, the loss is permanent and BLE coded is not the answer. So the pivotal experiment is
whether **coded adverts alone carry data at S=8 range, with no connection**. Everything
else here is supporting apparatus.

## Shape of the system

The boards are dumb. Each reports only what it directly hears, keyed on **(peer, channel)**,
keeping the last 20 receptions per pair in RAM. There is no mesh, no relaying, no scheduler,
and no persistent logging — flash writes stall the system, which would be its own
contention source, and a log is useless without knowing where you were anyway. Position,
history and export live in the browser next to the GPS fix.

A *channel* here means a link kind, not an RF channel:

    ble_adv_coded  ble_conn  espnow  espnow_lr
    wifi_beacon    wifi_lr_beacon    ieee802154   ftm

BLE legacy advertising exists solely so Chrome can discover a board. It is never a range
measurement and has no channel slot.

### Modes

Radio isolation is operator-commanded — there is no scheduler and no automatic cycling. A
mode is locked in until you change it from the UI or the button. `normal` is everything on;
every other mode is "just X", with a keep-UI-BLE flag (default on) that leaves the phone
link up at slowed intervals. The intended method is to first find out whether BLE-on hurts
X at all, then do the real testing with BLE on.

**GPIO9**: tap advances to the next mode, long press returns to `normal`. The long press is
the only way back on a board whose mode has taken the UI link off the air — which is also
why `rt_mode` auto-reverts to `normal` after 10 minutes without UI contact.

## Status: M0 — observable skeleton

No radios yet. What exists is the spine: the observation table, the mode/radio truth table,
the GPIO9 button, and a serial dump of the peer × channel table.

    === node 0x42 | mode=just_ble epoch=1 ui_ble=on | up 4s ===
    peer chan             last   mean   miss stale     age  rx  phy
    0x11 ble_adv_coded    -73  -72.8      0     4   2450ms   8  ble_coded_s8
    0x11 espnow           -69  -68.8      0     9   2450ms   8  wifi_11b
    0x11 ieee802154       -96  -95.8     14     9   2450ms   8  154_oqpsk

`miss` counts sequence numbers absent *between* the receptions held in the window; `stale`
is how many are presumed missed since the last one arrived, from its age. Silence has to
read as loss, not as an absence of data — on a range walk that silence is the whole point.

### Remaining milestones

| | |
|---|---|
| M1 | BLE GATT transport, CBOR protocol, SPA with multi-peer connect, coded adverts end to end |
| M2 | **The S=8 experiment** — the project's central question |
| M3 | ESP-NOW + 802.15.4 |
| M4 | Connection manager (slot rotation — connection slots are scarce) |
| M5 | Map, GPS, pick up / put down, history |
| M6 | Wi-Fi channels, FTM, LR mode, and the HTTP/WS transport for BLE-off testing |

## Building

ESP-IDF, one build per target — C3 and S3 have no 802.15.4 radio, so channels are
compile-time gated and the boot log reports what this build actually has.

```sh
cd firmware
idf.py set-target esp32c6
idf.py build flash monitor
```

PlatformIO can be used for upload, but note that official `platform-espressif32` does not
support the C6 at all and PlatformIO is not merging community support for it — that path
needs the [pioarduino](https://github.com/pioarduino/platform-espressif32) fork. `idf.py`
is the guaranteed path.

`CONFIG_RT_SIM_PEER` (on by default) fabricates a peer that fades in and out, so the whole
observation path can be exercised on one board with nothing else on the bench. Every report
prints a line saying the data is simulated. Turn it off before collecting anything real.

## Host tests

All the logic worth testing is pure C with no ESP-IDF dependencies, so it runs without
hardware:

```sh
cd firmware/components/rt_core/test_host
cmake -S . -B build-host && cmake --build build-host
ctest --test-dir build-host --output-on-failure
```

Built with `-Werror -Wconversion -Wshadow`. Covers the wire format, the sliding observation
window (sequence gaps, 16-bit wraparound, duplicates, epoch resets, staleness, millisecond
rollover), the mode/radio isolation truth table, report rendering and truncation safety, and
button debounce and press timing.

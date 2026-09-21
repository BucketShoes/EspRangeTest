# EspRangeTest

A throwaway range tester for the ESP32-C6. Flash two boards, walk away with one, watch how
far each radio still gets packets through.

Every radio broadcasts a small numbered packet on a timer and listens the rest of the time.
Every 2 seconds the results go out over serial:

```
== node C5  up 42s  lc=off ==
  tx: espnow=168 ble_adv=84 154=168
  C6 espnow   rssi  -71 (avg  -68, -78..-59)  pdr  96% now /  98% all  rx 164 miss 3   251ms ago
  C6 ble_adv  rssi  -84 (avg  -81, -91..-70)  pdr  75% now /  82% all  rx  69 miss 15  502ms ago
  C6 154      rssi  -88 (avg  -85, -95..-77)  pdr  61% now /  70% all  rx 118 miss 74  253ms ago lqi 96
```

`pdr now` is a sliding 10-second window (`RT_PDR_WINDOW_MS` in `main/rt.h`), `pdr all` is since boot. Loss is counted from gaps
in the sequence numbers.

## Radios

| | |
|---|---|
| **ESP-NOW** | broadcast, no pairing, no ack — every 250ms |
| **BLE** | extended advertising on the **coded PHY**, and a coded-PHY scanner — every 500ms |
| **802.15.4** | raw frames, channel 26 — every 250ms |

Raw 802.15.4 rather than Thread or Zigbee on purpose: both of those ride this exact PHY
(2.4GHz O-QPSK, 250kbps), so the range is identical. The stacks only add addressing and
routing, plus a join procedure that can fail at the far end for reasons that have nothing
to do with radio range.

**On BLE coded:** S=8 is the long-range hypothesis, but it's only reliably negotiated after
a connection, and connections can only be made close up. If you walk out, lose it, and
adverts aren't really going out at S=8, that loss is permanent. So this tests adverts alone,
with no connection. Note that the C6 is Bluetooth 5.3 and the feature that would let a
receiver read back S=2 vs S=8 (Advertising Coding Selection) is 5.4 — so the controller
reports "coded" without saying which. Measured range is the answer here, not a status field.

## GPIO9

All three radios share one antenna and arbitrate for it, so running them together costs
something. The BOOT button switches low-contention mode, which isolates one channel so it
gets a clean run at the antenna:

- **tap** — cycle: all → espnow only → ble only → 154 only → all
- **hold** — back to all radios

Low contention does more than stop the other two channels' own packets: it also stops the
BLE coded-PHY scanner (a 100% duty-cycle receiver — the one continuous, always-on source of
contention this board creates on its own) and slows the phone-UI advert and connection
interval right down, trading a laggy link for airtime. Expect low-contention testing of
anything other than `ble_adv` to be a bench-test affair with serial output, not a live walk
with the phone connected.

Switching does **not** reset the counters, and neither does anything else — not power, LR or
antenna either. Each board's card in the phone UI has its own **reset stats** button, and that
is the only thing that clears it. Changing a setting on one board used to wipe that board's
record of what it had heard from the other one, which was backwards: the slider changes what
the board *transmits*, and the table is what it *received*.

## GNSS (optional)

Any board can carry a GNSS module — typically one on a drone, flown around a set of boards on
the ground. Wire it to the XIAO's UART pins:

| GNSS pin | board pin |
|---|---|
| **TX** | **GPIO20** (D9) — the one that matters |
| RX | GPIO19 (D8) — optional; nothing is sent to the module yet |
| VCC / GND | 3V3 / GND |

The baud rate is found automatically (9600, 38400, 115200, 57600, 4800, 19200). Any module
that outputs NMEA `GGA` works, which is nearly all of them by default. With nothing fitted the
board behaves exactly as before, and its card says `gnss no NMEA on GPIO20`.

With a fix:

- **Its packets carry its position** — lat/lon/alt, 22 bytes instead of 12. Every board that
  hears it knows where it was when it sent that packet. The longer packet is slightly easier to
  lose at the edge of range; that is the price.
- **Positions have momentum.** Fixes come once a second and packets four times, so between
  fixes each packet's position is pushed along the drone's own motion (from its last two fixes),
  by how many seqs have gone out since the fix — capped at 2 s. Marks spread out along the
  flight path instead of stacking four deep, and the same seq gets the same position everywhere.
- **It logs where it was when it heard things.** Every packet it receives is recorded against
  its own track, and its own fixes are recorded with its sequence counters, so it can later say
  exactly which position every packet it *sent* carried — including the ones nobody heard.
- **Every board logs packets that have a position attached** — from a GNSS sender, or heard
  by a GNSS receiver. The log is a RAM ring (no flash writes). At boot the board runs through
  every mode once, before the antenna is powered, and then takes what the heap's low-water mark
  says is spare, so no later mode change can run short. The card shows it as `log n/N blocks`.
  When it fills, the oldest goes first. A moving drone costs a board hearing it about 10 bytes
  a packet; the drone logs about 5 bytes per packet it hears, plus 30 a second for its fixes.

On the page, each GNSS board gets its own track and a ◆ marker with its altitude, separate from
the phone's. Everything is matched by sequence number — the one thing the sender and every
receiver already agree on. Connect to *any* board — the drone, or any board that heard it — and
the track is built from everything every connected board knows: if A heard packets 1, 3, 5 and
B heard 1, 2, 6, the track runs 1-2-3-5-6, with each receiver's misses drawn at the positions of
the packets it missed. Connecting to the drone fills in the rest.

A board that was out of range keeps logging, and sends its stockpile when you reconnect —
newest first, so the live picture is immediate, then the backlog as fast as the link takes it:
the rate grows while NimBLE accepts every notification and halves the moment it refuses one,
report or log. The card shows `fetching, n behind` until it is done.

**Every board needs v7 firmware and the page redeployed.** An older board discards the 22-byte
packets as foreign, so it stops hearing a GNSS board the moment that board gets a fix.

## If it won't boot: the bring-up ladder

`build_flags = -DRT_STAGE=n` in `platformio.ini` controls how much starts up:

| stage | brings up |
|---|---|
| 0 | heartbeat only — proves toolchain, partition table, flash config, console |
| 1 | + Wi-Fi and ESP-NOW |
| 2 | + BLE coded beacon and scanner |
| 3 | + 802.15.4 |
| 4 | + phone UI over BLE GATT (the finished thing) |

Start at 0 and raise it one step at a time, flashing after each. A board that dies at a known
stage names the layer that broke it. **If even stage 0 fails, the problem is build
configuration, not radio code** — every failure on this project so far has been in that
layer: console routed to the wrong USB socket, an app partition smaller than the image, and a
flash/MMU page size mismatch between what PlatformIO set and what ESP-IDF derived.

After changing anything in `sdkconfig.defaults` or `partitions.csv`, run
`pio run -e devkitm -t fullclean` first — the bootloader is built from the same config, and a
stale bootloader with a fresh app is its own class of failure.

## Build and flash

```sh
pio run -e devkitc -t upload -t monitor    # DevKitC-1 (WROOM-1, 8MB)
pio run -e devkitm -t upload -t monitor    # DevKitM-1 (MINI-1, 4MB)
```

With both boards plugged in, name the port: `--upload-port COM23`.

PlatformIO downloads its own ESP-IDF, so no separate install is needed. `idf.py build flash
monitor` works against the same tree if you have IDF set up. C3 and S3 also build — they
have no 802.15.4 radio, so that channel is skipped and the boot log says so.

## Phone UI

`docs/index.html` — connect over Web Bluetooth and watch the same numbers on a phone while
you walk. Each board advertises as `ESPRT-xxxxxx` (last three MAC bytes); tap **Connect a board** twice to watch both
at once. The low-contention buttons switch radio isolation remotely, and it reconnects by
itself when a board comes back into range.

Web Bluetooth needs a secure context, so a `file://` page will not work:

- **GitHub Pages** — repo Settings → Pages → deploy from branch, folder `/docs`.
- **Locally** — `python3 -m http.server` in `docs/`, then open `http://localhost:8000`
  (localhost counts as secure).

Chrome only. Its scanner cannot see extended or coded adverts, which is why the board runs a
separate plain legacy advert just for the browser — that one is not a measurement, it's the
window onto the measurements. On connect the board asks to move the phone link to coded S=8
and logs whether the phone accepted.

## Not done yet

Wi-Fi beacons, LR mode, and FTM ranging.

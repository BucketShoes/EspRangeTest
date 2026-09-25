# EspRangeTest

A throwaway range tester for the ESP32-C6. Flash two boards, walk away with one, watch how
far each radio still gets packets through.

Each test broadcasts a small numbered packet on a timer and listens the rest of the time; the
FTM test measures distance to the other boards instead. Every test is a switch of its own and
starts **off**. The results go out over serial every `REPORT_MS` (`main/main.c`):

```
== node C5  up 42s  tests=espnow+ble_adv+154 ==
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
| **FTM** | 802.11mc fine timing measurement to other boards' APs, one after another |

(Periods and channels as of writing; each file's `TX_PERIOD_MS` / `CHANNEL` / `ADV_PERIOD_MS`
has the real one.)

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

## Tests are switches

The radios share one antenna and arbitrate for it, so running them together costs something.
Real use is one thing under test, or a chosen few together the way they will be flown — say
802.15.4 to get close to a lost board and FTM for the last stretch — so each test is its own
switch (`RT_TEST_*` in `main/rt.h`), set from the phone, and all of them are off at boot:

| switch | what it runs |
|---|---|
| `espnow` | ESP-NOW packets out and in. Brings the Wi-Fi driver up. |
| `ble_adv` | the coded-PHY beacon and scanner. The scanner listens continuously when this is the only test on, and duty-cycles when sharing. |
| `154` | 802.15.4 frames out, and the receiver on. |
| `ftm` | the FTM initiator: scan our Wi-Fi channel for other boards' APs, range to each in turn. Brings the Wi-Fi driver up. |
| `ftm resp` | keeps the Wi-Fi driver up for nothing else, so this board's AP is there to be ranged. |

A switch turns off everything its radio does on its own schedule, not just its packets: with no
Wi-Fi test on the Wi-Fi driver stops, with `ble_adv` off the coded scanner stops, with `154` off
its receiver sleeps.

Whenever the Wi-Fi driver is up its AP is up too, and the AP is the FTM responder — so any board
with `espnow`, `ftm` or `ftm resp` on can be ranged to. (It is never STA-only: an unassociated
station power-saves and ESP-NOW goes deaf.)

The BLE link to the phone is **not** a test and has no switch — it is always on. It runs slow
(long advert and connection intervals) while some test other than `ble_adv` is on, to hand that
test the antenna, and fast otherwise. See `UI_SLOW()` in `main/rt_ui.c`.

## GPIO9

- **tap** — step through the tests one at a time: none → espnow → ble_adv → 154 → ftm →
  ftm resp → none. For a bench without a phone; combinations are set from the page. A tap on a
  combination clears it.
- **hold** — restore: every test off, LR off, control link back on coded PHY, tx unmuted —
  exactly as booted.

Switching does **not** reset the counters, and neither does anything else — not power, LR or
antenna either. Each board's card in the phone UI has its own **reset stats** button, and that
is the only thing that clears it. Changing a setting on one board used to wipe that board's
record of what it had heard from the other one, which was backwards: the slider changes what
the board *transmits*, and the table is what it *received*.

## FTM

With `ftm` on, a board scans its own Wi-Fi channel for APs named `ESPRT-xxxxxx` whose beacon
says they answer FTM, and ranges to them one session at a time with a randomised pause between
— no fixed rate. Timings, frame count and how long an unanswering board is kept on the list are
at the top of `main/rt_ftm.c`. It never ranges to anything that isn't one of ours.

Every session is a result, successful or not: it goes in the FTM rows of the report (distance,
recent average, RSSI of the FTM frames, success rate of the last 16) and into the packet log,
GNSS or not, so a board that ranged while out of reach of the phone hands its ranges over when
it is back, like its packets.

While FTM is on (either switch), Wi-Fi adds 11g/11n (HT20) to its 11b — every FTM exchange seen
so far ran with them, and whether an 11b-only AP answers at all is untried. ESP-NOW's own rate is
pinned separately and is unaffected.

On the page each range is a faint ring round where it was measured from; where they agree is
where the other board is, and once they come from more than one spot a cross marks the best fit.
The **ftm** button on the map cycles showing them over everything, on their own, or not at all.
A board with its own GNSS is placed by it; any other board is placed by tapping **carry** on its
card — it then follows the phone's GPS until tapped again, which leaves it where you stood.

Distances are raw: nothing is calibrated out yet. A ring from a drone is drawn at the ground
distance its slant range means, taking the lowest the drone has been as ground level.

## GNSS (optional)

Any board can carry a GNSS module — typically one on a drone, flown around a set of boards on
the ground. Wire the module's TX to the pin `GNSS_RX_GPIO` names at the top of `main/rt_gnss.c`,
and optionally its RX to `GNSS_TX_GPIO` (nothing is sent to the module yet). When this was
written those were GPIO18 and GPIO19; the serial report says which pin it is listening on.

The baud rate is found automatically, trying each rate in `BAUDS` in `rt_gnss.c`. Any module
that outputs NMEA `GGA` should work. With nothing fitted the board behaves as it does without
GNSS, and its card says `gnss no NMEA`.

With a fix:

- **Its packets carry its position**, appended to the measurement packet: the fraction of a
  degree of lat/lon plus altitude (layout and resolution under "Where the sender was" in
  `main/rt.h`). The page fills in the whole degrees from the phone's GPS, the GNSS board's own
  log, or the last position it saw. A longer packet is slightly easier to lose at the edge of
  range; that is the price.
- **What it carries between fixes is a setting** — the `pos:` button on its card: the latest
  **fix** as it is, **smoothed** (eases onto each new fix, lags, never overshoots), or
  **momentum** (also follows a tracked velocity — closer in steady motion, overshoots when a fix
  jumps). The boot default and the options are under "Momentum" in `rt.h`. A change takes effect
  from the next fix, and the position is withdrawn once there has been no fix for `STALE_MS`
  (`rt_gnss.c`). Whichever is in use, the same seq gets the same position everywhere.
- **It logs where it was when it heard things.** Every packet it receives is recorded against
  its own track, and its own fixes are recorded with its sequence counters, so it can later say
  exactly which position every packet it *sent* carried — including the ones nobody heard.
- **Every board logs packets that have a position attached** — from a GNSS sender, or heard by
  a GNSS receiver — in a RAM ring of `RT_LOG_BYTES` (`rt.h`), taken first thing at boot, no
  flash writes. When it fills, the oldest goes first. What each record costs is in the log
  format in `rt.h`; the serial report prints heap free, lowest and largest free block, which is
  what `RT_LOG_BYTES` should be set from.

On the page, each GNSS board gets its own track and a ◆ marker with its altitude, separate from
the phone's. Everything is matched by sequence number — the one thing the sender and every
receiver already agree on. Connect to *any* board — the drone, or any board that heard it — and
the track is built from everything every connected board knows: if A heard packets 1, 3, 5 and
B heard 1, 2, 6, the track runs 1-2-3-5-6, with each receiver's misses drawn at the positions of
the packets it missed. Connecting to the drone fills in the rest. Marks that land on the same
spot are spread slightly when drawn (`JIT_M` in the page), so they stay distinct.

A board that was out of range keeps logging, and sends its stockpile when you reconnect —
newest first, so the live picture is immediate, then the backlog as fast as the link takes it:
the rate grows while NimBLE accepts every notification and backs off the moment it refuses one,
report or log. The card shows `fetching, n behind` until it is done.

**Boards and page have to agree on the report version** (`RT_RPT_VER` in `rt.h`; the page lists
what it decodes). A board without GNSS support discards positioned packets as foreign, so it
stops hearing a GNSS board the moment that board gets a fix — reflash the whole set together.

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
at once. The test switches on each card turn tests on and off remotely, and it reconnects by
itself when a board comes back into range.

Web Bluetooth needs a secure context, so a `file://` page will not work:

- **GitHub Pages** — repo Settings → Pages → deploy from branch, folder `/docs`.
  Published at <https://bucketshoes.github.io/EspRangeTest/>.
- **Locally** — `python3 -m http.server` in `docs/`, then open `http://localhost:8000`
  (localhost counts as secure).

### Install it for the walk

The page is a **PWA**: open it once with a signal and the browser offers
"Install" / "Add to Home Screen". After that it launches and runs with no network — which
matters, because a range walk takes you away from signal by definition, and Web Bluetooth,
GPS and the screen wake lock all stop working the moment the page can't be loaded over
https.

Everything is cached: the page is one self-contained file, so there is nothing to fetch at
runtime and no chart library to lose. Your saved board names, base point, plot settings and
field setup live in `localStorage` and survive both the install and any update.

Updates are automatic — launch it once with a signal after a new version is pushed and it
offers a one-tap reload. If a cached build ever misbehaves out in a field, load
`…/?nosw` to wipe the offline copy and unregister the worker.

See `docs/PWA.md` for how the caching works.

Chrome only. Its scanner cannot see extended or coded adverts, which is why the board runs a
separate plain legacy advert just for the browser — that one is not a measurement, it's the
window onto the measurements. On connect the board asks to move the phone link to coded S=8
and logs whether the phone accepted.

## Not done yet

Wi-Fi beacons as a measurement, and FTM distance calibration. FTM has not been tried with LR on.

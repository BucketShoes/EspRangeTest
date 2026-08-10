# EspRangeTest

A throwaway range tester for the ESP32-C6. Flash two boards, walk away with one, watch how
far each radio still gets packets through.

Every radio broadcasts a small numbered packet on a timer and listens the rest of the time.
Every 2 seconds the results go out over serial:

```
== node C5  up 42s  solo=off ==
  tx: espnow=168 ble_adv=84 154=168
  C6 espnow   rssi  -71 (avg  -68, -78..-59)  pdr  96% now /  98% all  rx 164 miss 3   251ms ago
  C6 ble_adv  rssi  -84 (avg  -81, -91..-70)  pdr  75% now /  82% all  rx  69 miss 15  502ms ago
  C6 154      rssi  -88 (avg  -85, -95..-77)  pdr  61% now /  70% all  rx 118 miss 74  253ms ago lqi 96
```

`pdr now` is the current 2-second window, `pdr all` is since boot. Loss is counted from gaps
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
something. The BOOT button isolates them:

- **tap** — cycle solo mode: all → espnow only → ble only → 154 only → all
- **hold** — back to all radios

Switching resets the counters, since sequence numbers restart.

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
you walk. Each board advertises as `ESPRT-xx`; tap **Connect a board** twice to watch both
at once. The solo buttons switch radio isolation remotely, and it reconnects by itself when
a board comes back into range.

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

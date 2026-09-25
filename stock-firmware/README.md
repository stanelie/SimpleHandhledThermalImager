# Stock (vendor) firmware backup

`stock-firmware.bin` is a verbatim SWD dump of the **original vendor firmware**
from this camera, taken before anything was reflashed.

| | |
|---|---|
| size | 65536 bytes (the full 64 KiB flash) |
| load address | `0x08000000` |
| sha256 | `4180b0166754a40928f5911e09f9d93f04c0f8518e2dd48caa7a396251be7b5e` |
| reset vector | `0x08000148`, initial SP `0x20004ff8` |

**Both devices have now been reflashed, so this file is the only remaining copy.**
Verify it before relying on it:

```sh
sha256sum -c stock-firmware.bin.sha256
```

## Restore a device to stock

```sh
openocd -f interface/stlink.cfg -c "adapter speed 100" -f target/stm32f1x.cfg \
  -c "init" -c "reset halt" \
  -c "program stock-firmware.bin 0x08000000 verify" \
  -c "reset run" -c "shutdown"
```

## Re-dump from a device that still has stock firmware

```sh
openocd -f interface/stlink.cfg -c "adapter speed 100" -f target/stm32f1x.cfg \
  -c "init" -c "reset halt" \
  -c "dump_image stock-firmware.bin 0x08000000 0x10000" -c "shutdown"
```

## Note on distribution

This is the vendor's proprietary binary, so it is **excluded from git** (see
`.gitignore`) and is not published to the public repository. It therefore exists
only on the local machine — if you want it backed up off-machine, put it
somewhere private (a private repo, or a release asset on a private repo) rather
than committing it here.

The tools in `../tools/` read it via `STOCK_FW=/path/to/stock-firmware.bin`.

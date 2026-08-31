# Generated and vendored files

Everything in this directory is a build artifact or third-party code, not
hand-written source. The site fetches these alongside `website/app.html`
(see `assets` in the website repo's `content.config.ts`), so they are
committed rather than built at deploy time.

| File | What it is |
|---|---|
| `uti.js` | **Generated.** The three scalar profiles compiled to WebAssembly. Rebuild with `wasm/build.sh`, verify with `wasm/verify_parity.mjs`. See [`wasm/README.md`](../../wasm/README.md). |
| `webservodemo-atmega328p.hex` | **Generated.** `examples/WebServoDemo` compiled for `arduino:avr:uno`. |
| `avrbro.umd.js` | **Vendored**, unmodified. See below. |
| `avrbro.LICENSE` | avrbro's MIT license, as required. |

## avrbro

- Upstream: <https://github.com/kaelhem/avrbro>
- Version: **0.0.8**
- File: `dist/avrbro.umd.js` from the npm tarball, byte-for-byte
- License: MIT (`avrbro.LICENSE`)

Speaks STK500v1 over Web Serial, which is what an ATmega328P bootloader
wants. Vendored rather than loaded from a CDN because this app has no
build step and no runtime dependencies by design, and because a flasher
silently changing under the page is not a risk worth taking.

The UMD build was checked to run with no global `Buffer` (its `package.json`
declares a `buffer` dependency, but the bundle inlines what it needs), so it
works in a browser as-is.

Its `boardsHelper.getBoard()` names map to the app's board picker:

| Picker | avrbro board | Bootloader baud |
|---|---|---|
| Uno | `uno` | 115200 |
| Nano (new bootloader) | `nano (new bootloader)` | 115200 |
| Nano (old bootloader) | `nano` | 57600 |

**Not** `dbuezas/arduino-web-uploader`, which earlier notes suggested: it is a
GitHub project with no published npm package, so there is no versioned
tarball to vendor from.

## One hex for all three board options

`arduino:avr:uno` and `arduino:avr:nano:cpu=atmega328` produce a
**byte-identical** `.hex` — same ATmega328P at the same clock. Verified by
diffing the two compiles, and re-checked by the `firmware` CI job.

Only the *upload* differs between them, and that is an uploader-side
setting (bootloader protocol and baud), not something baked into the image.
So the picker above selects flashing parameters, not a different binary.

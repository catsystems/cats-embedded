# FatFs vendoring

This directory contains FatFs R0.16 with upstream patch-2 dated July 10, 2026.

- Upstream archive: `https://elm-chan.org/fsw/ff/arc/ff16.zip`
- Archive SHA-256: `99f7dc1f7e095356e4a9e3dbe29959090d8b948afe2bbc5441e52fdf4b85449e`
- Patch-1: `https://elm-chan.org/fsw/ff/patch/ff16p1.diff`
- Patch-1 SHA-256: `7996ddc3135f3d534a8153f7d75d587030e5c8551b003c3b7ee823ffb7fc64bf`
- Patch-2: `https://elm-chan.org/fsw/ff/patch/ff16p2.diff`
- Patch-2 SHA-256: `5cd39f1fc299f0f1dec9fa1ce544dc0bc048b2f6eb3b8161476ed20f9cf1e290`

`ff.c`, `ff.h`, `diskio.h`, and `ffunicode.c` come from that archive, with
patch-1 and patch-2 applied to `ff.c`. Line endings and incidental trailing
whitespace are normalized.
`ffconf.h` is based on the R0.16 template and retains the Ground Station's
R0.13c behavior: read/write, formatting and labels enabled, CP437, static LFN,
one auto-detected volume, 512-byte sectors, exFAT enabled, no RTC, no locking,
and no FatFs-internal reentrancy.

The application owns `VolToPart` as `{0, 0}` to preserve the existing
super-floppy format and eliminate the previous implicit resolution from
ESP-IDF's precompiled `libfatfs.a`. The disk callbacks remain in `src/utils.cpp`.

The compressed fixture under `tests/fixtures` was generated with the previous
vendored R0.13c sources and the same configuration. Its Unicode support came
from the official `ff13c.zip` archive (SHA-256
`39feb00dced8585d0ec8f1f6acf47a2ae3ea4123c077a4c0f310b303e89ec391`).
The uncompressed image SHA-256 is
`ea492f6864a67119e1d7e0b7f23e9dac4205933a9b0142a5951865fb62943a40`.
The compatibility harness
verifies read-only mounting, existing-file reads, writes, remounting, R0.16
formatting, and rejection of corrupted media without writes.

# YM2608 rhythm ROM tool

The YM2608 (OPNA) plays its six rhythm sounds from an 8 KB ROM inside the chip. libvgm's YM2608 core ships with a
free replacement ROM (the Tetorica OPNA rhythm sounds, see `../../emu/cores/ym2608_rhythm.h`), and a player can
load another one (it asks for `ym2608_adpcm_rom.bin`).

`ym2608rom` turns a rhythm ROM into six `.wav` files and back, so you can make your own drum set in any sample
editor (Furnace, Audacity, ...).

## Usage

```
ym2608rom decode <out_dir> [rom.bin]   write the six voices as .wav (no ROM: the built-in sounds)
ym2608rom encode <in_dir> <rom.bin>    build an 8 KB ROM from the six .wav files
```

1. Start from `template/` (the built-in sounds, decoded) or decode another ROM.
2. Replace or edit the `.wav` files. Keep the file names and the sample rates.
3. Run `ym2608rom encode <folder> my_rhythm.bin`.
4. Load `my_rhythm.bin` in your player. In Godot (gd_libvgm): *Project Settings → Libvgm Settings → Roms →
   Ym2608 Rhythm Rom*.

## The six voices

The chip plays each voice from a fixed part of the ROM at a fixed rate, so each one has a maximum length.

| File | Voice | Rate | Max samples | Max length |
|---|---|---|---|---|
| `1_bass_drum.wav` | Bass drum | 18489 Hz | 896 | 48 ms |
| `2_snare_drum.wav` | Snare drum | 18489 Hz | 1280 | 69 ms |
| `3_top_cymbal.wav` | Top cymbal | 18489 Hz | 11904 | 644 ms |
| `4_hi_hat.wav` | Hi-hat | 18489 Hz | 768 | 42 ms |
| `5_tom_tom.wav` | Tom tom | 9244 Hz | 1280 | 138 ms |
| `6_rim_shot.wav` | Rim shot | 9244 Hz | 256 | 28 ms |

- Rates are for the usual 7.9872 MHz clock (PC-98). Files up to 1% off are accepted, for example 18519 Hz from 8 MHz
  tools. Resample anything else first; the tool doesn't, because the pitch would change.
- `.wav` files must be 16-bit PCM, mono or stereo (stereo is mixed to mono).
- A shorter sample is padded with silence. A longer one is cut, with a warning.
- The chip always plays a voice to the end of its part of the ROM, so let each sample fade out to silence.
- Samples are stored as 4-bit ADPCM-A (12-bit precision), so quiet detail and very sharp attacks lose some quality.

## Furnace

Furnace can load and save `.wav` samples. Import a template file into a sample, edit it, set its rate to the one
in the table, and save it back as `.wav` over the template file name.

## Building

The tool is built with libvgm (CMake target `ym2608rom`, option `BUILD_YM2608ROM`). It is a single C file with no
dependencies, so this works too:

```
cc -O2 -o ym2608rom ym2608rom.c
```

## License

The files in `template/` are decoded from the Tetorica OPNA rhythm sounds (BSD-3-Clause, `template/LICENSE`).
Ship that notice with them, or with a ROM made from them. Yamaha's own rhythm ROM is copyrighted: don't
distribute it, or samples taken from it.

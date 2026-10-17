// YM2608 rhythm ROM tool: turns an 8 KB YM2608 rhythm (ADPCM-A) ROM into six .wav files and back.
//
//   ym2608rom decode <out_dir> [rom.bin]   write the six voices as .wav (no ROM: the built-in Tetorica sounds)
//   ym2608rom encode <in_dir> <rom.bin>    build a ROM from the six .wav files
//
// The chip plays each voice from a fixed address range at a fixed rate, so every voice has a maximum length.
// See README.md for the file names and limits.

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#ifdef _WIN32
#include <direct.h>	// for _mkdir
#define make_dir(path)	_mkdir(path)
#else
#include <sys/stat.h>	// for mkdir
#define make_dir(path)	mkdir(path, 0777)
#endif

#include "../../stdtype.h"
#include "../../emu/cores/ym2608_rhythm.h"	// the built-in rhythm sounds (YM2608_RHYTHM_ROM_SIZE)

typedef struct _rhythm_voice
{
	const char* fileName;
	UINT16 start;	// first byte in the ROM
	UINT16 end;		// last byte in the ROM (inclusive)
	UINT32 rate;	// playback rate in Hz with a 7987200 Hz clock (PC-98)
} RHYTHM_VOICE;

// Address ranges are fixed in the chip. The rhythm channels are clocked every 3 FM samples (clock / 432);
// the tom tom and rim shot only every 6 (clock / 864).
static const RHYTHM_VOICE VOICES[6] =
{
	{"1_bass_drum.wav", 0x0000, 0x01bf, 18489},
	{"2_snare_drum.wav", 0x01c0, 0x043f, 18489},
	{"3_top_cymbal.wav", 0x0440, 0x1b7f, 18489},
	{"4_hi_hat.wav", 0x1b80, 0x1cff, 18489},
	{"5_tom_tom.wav", 0x1d00, 0x1f7f, 9244},
	{"6_rim_shot.wav", 0x1f80, 0x1fff, 9244},
};

// ADPCM-A step sizes and step index changes, as decoded by the chip (and libvgm's YM2608 core)
static const UINT16 STEPS[49] =
{
	 16,  17,   19,   21,   23,   25,   28,
	 31,  34,   37,   41,   45,   50,   55,
	 60,  66,   73,   80,   88,   97,  107,
	118, 130,  143,  157,  173,  190,  209,
	230, 253,  279,  307,  337,  371,  408,
	449, 494,  544,  598,  658,  724,  796,
	876, 963, 1060, 1166, 1282, 1411, 1552
};
static const INT8 STEP_INC[8] = { -1, -1, -1, -1, 2, 5, 7, 9 };

typedef struct _adpcm_state
{
	INT32 accumulator;	// signed 12-bit value
	INT32 step_index;
} ADPCM_STATE;

// Apply one 4-bit code; returns the new signed 12-bit sample. The 12-bit accumulator wraps, as on the chip.
static INT32 adpcm_step(ADPCM_STATE* st, UINT8 code, int apply)
{
	INT32 delta;
	INT32 value;
	INT32 index;

	delta = (INT32)(2 * (code & 7) + 1) * STEPS[st->step_index] / 8;
	if (code & 8)
		delta = -delta;
	value = ((st->accumulator + delta + 2048) & 0xfff) - 2048;
	if (apply)
	{
		index = st->step_index + STEP_INC[code & 7];
		st->step_index = (index < 0) ? 0 : (index > 48) ? 48 : index;
		st->accumulator = value;
	}
	return value;
}

// Decode a voice into 16-bit samples; each key-on starts from a fresh decoder state.
static void decode_voice(const UINT8* rom, const RHYTHM_VOICE* v, INT16* out)
{
	ADPCM_STATE st = {0, 0};
	UINT32 pos;
	UINT8 code;

	for (pos = v->start; pos <= v->end; pos ++)
	{
		code = rom[pos] >> 4;	// high nibble first
		*out++ = (INT16)(adpcm_step(&st, code, 1) * 16);
		code = rom[pos] & 0x0f;
		*out++ = (INT16)(adpcm_step(&st, code, 1) * 16);
	}
	return;
}

// Encode 16-bit samples into a voice, picking the closest code for each sample.
static void encode_voice(const INT16* in, const RHYTHM_VOICE* v, UINT8* rom)
{
	ADPCM_STATE st = {0, 0};
	UINT32 count = (UINT32)(v->end + 1 - v->start) * 2;
	UINT32 i;
	INT32 target;
	INT32 error;
	INT32 bestError;
	UINT8 code;
	UINT8 bestCode;

	memset(&rom[v->start], 0x00, v->end + 1 - v->start);
	for (i = 0; i < count; i ++)
	{
		target = (in[i] + 8) >> 4;	// 16 to 12 bits, rounded
		if (target > 2047)
			target = 2047;
		bestCode = 0;
		bestError = 0x7fffffff;
		for (code = 0; code < 16; code ++)
		{
			error = adpcm_step(&st, code, 0) - target;
			if (error < 0)
				error = -error;
			if (error < bestError)
			{
				bestError = error;
				bestCode = code;
			}
		}
		adpcm_step(&st, bestCode, 1);
		rom[v->start + i / 2] |= (i & 1) ? bestCode : (UINT8)(bestCode << 4);
	}
	return;
}

static void put_le(UINT8* p, UINT32 value, int bytes)
{
	int i;

	for (i = 0; i < bytes; i ++)
		p[i] = (UINT8)(value >> (8 * i));
	return;
}

static UINT32 get_le(const UINT8* p, int bytes)
{
	UINT32 value = 0;
	int i;

	for (i = bytes - 1; i >= 0; i --)
		value = (value << 8) | p[i];
	return value;
}

static char* join_path(const char* dir, const char* name)
{
	size_t len = strlen(dir);
	char* path = (char*)malloc(len + strlen(name) + 2);

	if (path == NULL)
		return NULL;
	strcpy(path, dir);
	if (len > 0 && dir[len - 1] != '/' && dir[len - 1] != '\\')
		strcat(path, "/");
	strcat(path, name);
	return path;
}

// Write a mono 16-bit PCM .wav file.
static int write_wav(const char* path, const INT16* samples, UINT32 count, UINT32 rate)
{
	UINT8 header[44];
	FILE* f;
	UINT32 i;
	UINT8 s[2];

	memcpy(&header[0], "RIFF", 4);
	put_le(&header[4], 36 + count * 2, 4);
	memcpy(&header[8], "WAVEfmt ", 8);
	put_le(&header[16], 16, 4);
	put_le(&header[20], 1, 2);	// PCM
	put_le(&header[22], 1, 2);	// mono
	put_le(&header[24], rate, 4);
	put_le(&header[28], rate * 2, 4);
	put_le(&header[32], 2, 2);
	put_le(&header[34], 16, 2);
	memcpy(&header[36], "data", 4);
	put_le(&header[40], count * 2, 4);

	f = fopen(path, "wb");
	if (f == NULL)
		return 0;
	fwrite(header, 1, 44, f);
	for (i = 0; i < count; i ++)
	{
		put_le(s, (UINT16)samples[i], 2);
		fwrite(s, 1, 2, f);
	}
	fclose(f);
	return 1;
}

// Read a 16-bit PCM .wav file (mono, or stereo mixed to mono) into max samples; the rest stays silent.
// Returns the number of samples in the file, or -1 on error.
static long read_wav(const char* path, INT16* out, UINT32 max, UINT32* rate)
{
	FILE* f;
	UINT8* data;
	long size;
	UINT32 pos;
	UINT32 chunkSize;
	UINT32 channels = 0;
	UINT32 bits = 0;
	const UINT8* pcm = NULL;
	UINT32 pcmSize = 0;
	UINT32 frames;
	UINT32 i;
	INT32 sum;
	UINT32 ch;

	f = fopen(path, "rb");
	if (f == NULL)
	{
		fprintf(stderr, "Can't open %s.\n", path);
		return -1;
	}
	fseek(f, 0, SEEK_END);
	size = ftell(f);
	fseek(f, 0, SEEK_SET);
	data = (UINT8*)malloc(size > 0 ? size : 1);
	if (data == NULL || fread(data, 1, size, f) != (size_t)size)
	{
		fclose(f);
		free(data);
		fprintf(stderr, "Can't read %s.\n", path);
		return -1;
	}
	fclose(f);

	if (size < 12 || memcmp(&data[0], "RIFF", 4) || memcmp(&data[8], "WAVE", 4))
	{
		free(data);
		fprintf(stderr, "%s is not a .wav file.\n", path);
		return -1;
	}
	for (pos = 12; pos + 8 <= (UINT32)size; pos += 8 + chunkSize + (chunkSize & 1))
	{
		chunkSize = get_le(&data[pos + 4], 4);
		if (chunkSize > (UINT32)size - pos - 8)
			chunkSize = (UINT32)size - pos - 8;
		if (! memcmp(&data[pos], "fmt ", 4) && chunkSize >= 16)
		{
			if (get_le(&data[pos + 8], 2) != 1)
				channels = 0;	// not plain PCM
			else
				channels = get_le(&data[pos + 10], 2);
			*rate = get_le(&data[pos + 12], 4);
			bits = get_le(&data[pos + 22], 2);
		}
		else if (! memcmp(&data[pos], "data", 4))
		{
			pcm = &data[pos + 8];
			pcmSize = chunkSize;
		}
	}
	if (pcm == NULL || bits != 16 || channels < 1 || channels > 2)
	{
		free(data);
		fprintf(stderr, "%s must be a 16-bit PCM .wav file, mono or stereo.\n", path);
		return -1;
	}

	frames = pcmSize / (2 * channels);
	memset(out, 0x00, max * sizeof(INT16));
	for (i = 0; i < frames && i < max; i ++)
	{
		sum = 0;
		for (ch = 0; ch < channels; ch ++)
			sum += (INT16)get_le(&pcm[(i * channels + ch) * 2], 2);
		out[i] = (INT16)(sum / (INT32)channels);
	}
	free(data);
	return (long)frames;
}

static int cmd_decode(const char* outDir, const char* romPath)
{
	UINT8 rom[YM2608_RHYTHM_ROM_SIZE];
	INT16 samples[YM2608_RHYTHM_ROM_SIZE * 2];
	FILE* f;
	char* path;
	int v;
	UINT32 count;

	if (romPath == NULL)
	{
		memcpy(rom, ym2608_rhythm_rom, YM2608_RHYTHM_ROM_SIZE);
	}
	else
	{
		f = fopen(romPath, "rb");
		if (f == NULL)
		{
			fprintf(stderr, "Can't open %s.\n", romPath);
			return 1;
		}
		count = (UINT32)fread(rom, 1, YM2608_RHYTHM_ROM_SIZE, f);
		if (count != YM2608_RHYTHM_ROM_SIZE || fgetc(f) != EOF)
		{
			fclose(f);
			fprintf(stderr, "%s must be exactly %u bytes.\n", romPath, YM2608_RHYTHM_ROM_SIZE);
			return 1;
		}
		fclose(f);
	}

	make_dir(outDir);	// fails harmlessly if it exists; a real problem shows up when writing
	for (v = 0; v < 6; v ++)
	{
		count = (UINT32)(VOICES[v].end + 1 - VOICES[v].start) * 2;
		decode_voice(rom, &VOICES[v], samples);
		path = join_path(outDir, VOICES[v].fileName);
		if (path == NULL || ! write_wav(path, samples, count, VOICES[v].rate))
		{
			fprintf(stderr, "Can't write %s.\n", path ? path : VOICES[v].fileName);
			free(path);
			return 1;
		}
		printf("%s: %u samples at %u Hz\n", path, count, VOICES[v].rate);
		free(path);
	}
	return 0;
}

static int cmd_encode(const char* inDir, const char* romPath)
{
	UINT8 rom[YM2608_RHYTHM_ROM_SIZE];
	INT16 samples[YM2608_RHYTHM_ROM_SIZE * 2];
	FILE* f;
	char* path;
	int v;
	long count;
	UINT32 max;
	UINT32 rate;

	for (v = 0; v < 6; v ++)
	{
		max = (UINT32)(VOICES[v].end + 1 - VOICES[v].start) * 2;
		path = join_path(inDir, VOICES[v].fileName);
		if (path == NULL)
			return 1;
		rate = 0;
		count = read_wav(path, samples, max, &rate);
		if (count < 0)
		{
			free(path);
			return 1;
		}
		// the chip plays at a fixed rate; a different one would change the pitch (1% leaves room for 8 MHz rates)
		if (rate * 100 < VOICES[v].rate * 99 || rate * 100 > VOICES[v].rate * 101)
		{
			fprintf(stderr, "%s is %u Hz; resample it to %u Hz first.\n", path, rate, VOICES[v].rate);
			free(path);
			return 1;
		}
		if ((UINT32)count > max)
			printf("Warning: %s has %ld samples; only the first %u fit, the rest is cut.\n", path, count, max);
		encode_voice(samples, &VOICES[v], rom);
		printf("%s: %ld of %u samples used\n", path, (UINT32)count < max ? count : (long)max, max);
		free(path);
	}

	f = fopen(romPath, "wb");
	if (f == NULL || fwrite(rom, 1, YM2608_RHYTHM_ROM_SIZE, f) != YM2608_RHYTHM_ROM_SIZE)
	{
		if (f != NULL)
			fclose(f);
		fprintf(stderr, "Can't write %s.\n", romPath);
		return 1;
	}
	fclose(f);
	printf("Wrote %s (%u bytes).\n", romPath, YM2608_RHYTHM_ROM_SIZE);
	return 0;
}

int main(int argc, char* argv[])
{
	if (argc >= 3 && argc <= 4 && ! strcmp(argv[1], "decode"))
		return cmd_decode(argv[2], (argc == 4) ? argv[3] : NULL);
	if (argc == 4 && ! strcmp(argv[1], "encode"))
		return cmd_encode(argv[2], argv[3]);

	printf("YM2608 rhythm ROM tool\n");
	printf("Usage:\n");
	printf("  %s decode <out_dir> [rom.bin]   write the six voices as .wav (no ROM: the built-in sounds)\n", argv[0]);
	printf("  %s encode <in_dir> <rom.bin>    build an 8 KB ROM from the six .wav files\n", argv[0]);
	return 1;
}

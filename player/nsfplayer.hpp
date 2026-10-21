#ifndef __NSFPLAYER_HPP__
#define __NSFPLAYER_HPP__

#include "../stdtype.h"
#include "../emu/EmuStructs.h"
#include "../emu/Resampler.h"
#include "../utils/StrUtils.h"
#include "helper.h"
#include "playerbase.hpp"
#include "nsf_cpu6502.h"
#include "../utils/DataLoader.h"
#include <vector>
#include <map>
#include <string>


#define FCC_NSF 	0x4E534600

// NES Sound Format (.nsf, NSF2) and Extended NES Sound Format (.nsfe): the sound driver of a NES
// game, played by running a 6502 on the NSF memory map (see nsf_fileformat.md and nsfe_fileformat.md
// of the nesdev wiki). The sound chips are libvgm devices: the 2A03 APU (with the FDS), VRC6, VRC7
// (the YM2413 in VRC7 mode), MMC5, Namco 163 and Sunsoft 5B, which the file lists in its header.
//
// NSF header (the fields the player reads):
//	Ofs	Len	Description
//	000	05	"NESM", 1A
//	005	01	version (1 = NSF, 2 = NSF2)
//	006	01	number of songs
//	007	01	first song (1-based)
//	008	06	load, init and play address
//	00E	60	name, artist, copyright (32 bytes each)
//	06E	02	play speed in microseconds (NTSC)
//	070	08	initial banks (all 0 = no bankswitching)
//	078	02	play speed in microseconds (PAL)
//	07A	01	region bits (bit 0 = PAL, bit 1 = dual)
//	07B	01	sound chip bits (see NSF_CHIP_)
//	07C	01	NSF2 feature flags
//	07D	03	length of the program data (0 = up to the end of the file), metadata follows it
//	080	..	program data
//
// NSFe is a list of chunks (INFO, DATA, BANK, RATE, plst, time, fade, tlbl, auth, mixe, ...), and
// NSF2 may carry the same chunks (without INFO, DATA, BANK and NSF2) after the program data.
//
// The player runs the song at the CPU clock and renders the devices up to every write of the
// song to a sound register, so the register writes are as exact as the output sample allows.
// The song length comes from the 'time' and 'fade' chunks. A song without a length plays until
// the host stops it: GetTotalTicks() is 0 and the player never reaches PLAYSTATE_END.
//
// A tick is a millisecond.
//
// Not supported (a file that needs it does not load, or the feature is ignored as noted):
//	- NSF2 non-returning INIT (flag bit 5): the file does not load
//	- NSF2 IRQ timer and the APU IRQs (flag bit 4, ignored): the song plays without interrupts
//	- VT02+ sound (chip bit 6, ignored)
//	- seeking backwards or forwards is done by running the song from its start

#define NSF_CHIP_VRC6	0x01
#define NSF_CHIP_VRC7	0x02
#define NSF_CHIP_FDS	0x04
#define NSF_CHIP_MMC5	0x08
#define NSF_CHIP_N163	0x10
#define NSF_CHIP_5B		0x20
#define NSF_CHIP_VT02	0x40

#define NSF_FILE_NSF	0x00	// NSF (version 1)
#define NSF_FILE_NSF2	0x01	// NSF2
#define NSF_FILE_NSFE	0x02	// NSFe

#define NSF_REGION_NTSC		0
#define NSF_REGION_PAL		1
#define NSF_REGION_DENDY	2

struct NSF_HEADER
{
	UINT8 fileType;			// NSF_FILE_ constant
	UINT8 songCount;
	UINT8 startSong;		// 0-based
	UINT16 loadAddr;
	UINT16 initAddr;
	UINT16 playAddr;
	UINT16 speedNTSC;		// play speed in microseconds, 0 = the rate of the frame (NMI)
	UINT16 speedPAL;
	UINT16 speedDendy;
	UINT8 bankInit[8];
	UINT8 bankSwitched;
	UINT8 regionMask;		// supported regions, bit 0 = NTSC, bit 1 = PAL, bit 2 = Dendy
	UINT8 regionPref;		// preferred region (NSF_REGION_)
	UINT8 chips;			// NSF_CHIP_ bits
	UINT8 nsf2Flags;
};

struct NSF_PLAY_OPTIONS
{
	PLR_GEN_OPTS genOpts;
};


class NSFPlayer : public PlayerBase
{
private:
	// The devices of a song, by what they play. Their order is the order of _devices.
	enum
	{
		DEVKIND_APU = 0,	// the 2A03 APU, with the FDS when the file has it
		DEVKIND_VRC6,
		DEVKIND_VRC7,
		DEVKIND_MMC5,
		DEVKIND_N163,
		DEVKIND_5B,
		DEVKIND_COUNT
	};
	struct DevCfg
	{
		DEV_ID type;
		UINT8 kind;
		UINT16 volume;
		std::vector<UINT8> data;
	};
	struct NSF_CHIPDEV
	{
		VGM_BASEDEV base;
		DEVFUNC_WRITE_A8D8 write8;
		DEVFUNC_READ_A8D8 read8;
		DEVFUNC_WRITE_A16D8 write16;
		DEVFUNC_READ_A16D8 read16;
		DEVFUNC_WRITE_BLOCK memWrite;
	};
	enum
	{
		PHASE_INIT = 0,		// the INIT routine of the song is running
		PHASE_IDLE,			// waiting for the next call of PLAY
		PHASE_PLAY			// the PLAY routine is running
	};

public:
	NSFPlayer();
	~NSFPlayer();

	UINT32 GetPlayerType(void) const;
	const char* GetPlayerName(void) const;
	static UINT8 PlayerCanLoadFile(DATA_LOADER *dataLoader);
	UINT8 CanLoadFile(DATA_LOADER *dataLoader) const;
	UINT8 LoadFile(DATA_LOADER *dataLoader);
	UINT8 UnloadFile(void);
	const NSF_HEADER* GetFileHeader(void) const;

	const char* const* GetTags(void);
	UINT8 GetSongInfo(PLR_SONG_INFO& songInf);
	UINT8 GetSongDeviceInfo(std::vector<PLR_DEV_INFO>& devInfList) const;
	UINT8 SetDeviceOptions(UINT32 id, const PLR_DEV_OPTS& devOpts);
	UINT8 GetDeviceOptions(UINT32 id, PLR_DEV_OPTS& devOpts) const;
	UINT8 SetDeviceMuting(UINT32 id, const PLR_MUTE_OPTS& muteOpts);
	UINT8 GetDeviceMuting(UINT32 id, PLR_MUTE_OPTS& muteOpts) const;
	UINT8 SetPlayerOptions(const NSF_PLAY_OPTIONS& playOpts);
	UINT8 GetPlayerOptions(NSF_PLAY_OPTIONS& playOpts) const;

	// songs: 0-based numbers. SelectSong() restarts the player with the song (when it is playing,
	// the song starts at once, otherwise with Start()). It also changes the tags and the length.
	UINT16 GetSongCount(void) const;
	UINT16 GetCurSong(void) const;
	UINT8 SelectSong(UINT16 song);
	// the order of the NSFe 'plst' chunk (song numbers, 0-based), empty when the file has none
	const std::vector<UINT8>& GetPlaylist(void) const;

	UINT8 SetSampleRate(UINT32 sampleRate);
	double GetPlaybackSpeed(void) const;
	UINT8 SetPlaybackSpeed(double speed);
	UINT32 Tick2Sample(UINT32 ticks) const;
	UINT32 Sample2Tick(UINT32 samples) const;
	double Tick2Second(UINT32 ticks) const;

	UINT8 GetState(void) const;
	UINT32 GetCurPos(UINT8 unit) const;
	UINT32 GetCurLoop(void) const;
	UINT32 GetTotalTicks(void) const;	// get time for playing once in ticks (0 = plays forever)
	UINT32 GetLoopTicks(void) const;	// get time for one loop in ticks (always 0: no loops)

	UINT8 Start(void);
	UINT8 Stop(void);
	UINT8 Reset(void);
	UINT8 Seek(UINT8 unit, UINT32 pos);
	UINT32 Render(UINT32 smplCnt, WAVE_32BS* data);

private:
	// file
	UINT8 ParseNSF(void);
	UINT8 ParseNSFe(void);
	UINT8 ParseChunks(UINT32 pos, bool isNSFe);
	UINT8 FinishLoad(void);
	void ClearFileData(void);
	void InitRegion(void);

	// tags and length
	void RefreshSongTags(void);
	void CalcSongLength(void);
	void SetTagString(const char* tagName, const std::string& value);
	std::string GetUTF8String(const char* startPtr, const char* endPtr, bool isUTF8);
	static std::string ReadString(const UINT8* data, size_t len, size_t* usedLen);

	// devices
	size_t DeviceID2OptionID(UINT32 id) const;
	void RefreshMuting(NSF_CHIPDEV& chipDev, const PLR_MUTE_OPTS& muteOpts);
	void GenerateDeviceConfig(void);
	UINT16 GetMixVolume(UINT8 kind, UINT16 baseVol) const;
	void RefreshTSRates(void);

	// the machine
	void BuildMemory(void);
	void SetBank(UINT16 reg, UINT8 bank);
	void CopyDpcmMemory(UINT32 addr, UINT32 length);
	void InitSoundRegs(void);
	void CallRoutine(UINT16 addr);
	void StartSong(void);
	void RunCpu(UINT64 cycleLimit);
	void SyncDevices(UINT32 smpl);
	void RenderVrc7(VGM_BASEDEV* dev, UINT32 smplCnt, WAVE_32BS* out);
	UINT32 CycleToSample(UINT64 cycle) const;
	UINT64 SampleToCycle(UINT32 smpl) const;
	UINT64 PlayTime(UINT32 index) const;
	void ScheduleNextPlay(void);
	UINT8 CpuRead(UINT16 addr);
	void CpuWrite(UINT16 addr, UINT8 value);
	UINT8 ReadIO(UINT16 addr, UINT64 cycle);
	void WriteIO(UINT16 addr, UINT8 value, UINT64 cycle);
	void WriteHigh(UINT16 addr, UINT8 value, UINT64 cycle);
	static UINT8 BusRead(void* ctx, UINT16 addr);
	static void BusWrite(void* ctx, UINT16 addr, UINT8 value);

	// playback
	UINT8 SeekToTick(UINT32 tick);
	UINT32 RenderSamples(UINT32 smplCnt, WAVE_32BS* data);
	void ApplyFade(UINT32 smplCnt, WAVE_32BS* data) const;
	void DoFileEnd(void);

	CPCONV* _cpc1252;	// CP1252 -> UTF-8 codepage conversion
	DATA_LOADER* _dLoad;
	UINT32 _fileLen;
	const UINT8* _fileData;	// data pointer for quick access, equals DataLoader_GetData(_dLoad)

	// the file
	NSF_HEADER _fileHdr;
	bool _hasRate;			// the file has its own play speeds (NSF header or NSFe 'RATE')
	const UINT8* _romData;	// the program data
	UINT32 _romLen;
	std::string _name;		// the strings of the NSF header or the NSFe 'auth' chunk
	std::string _artist;
	std::string _copyright;
	std::string _ripper;
	std::string _text;
	bool _hdrStringsUTF8;	// NSF2 or NSFe: the strings are UTF-8, otherwise CP1252
	UINT8 _vrc7Variant;		// NSFe 'VRC7' chunk: 0 = VRC7, 1 = YM2413
	std::vector<UINT8> _vrc7Patches;	// NSFe 'VRC7' chunk: replacement patch set (128 or 152 bytes)
	std::vector<UINT8> _playlist;
	std::vector<INT32> _trackTime;	// NSFe 'time': length of a track in milliseconds
	std::vector<INT32> _trackFade;	// NSFe 'fade'
	std::vector<std::string> _trackLabel;
	std::vector<std::string> _trackAuthor;
	INT16 _mixe[8];			// NSFe 'mixe': millibels, _mixeSet says which are in the file
	UINT8 _mixeSet;

	// the region
	UINT8 _region;
	UINT32 _cpuClock;
	UINT32 _playPeriodUs;	// time between two calls of PLAY, in microseconds

	std::vector<DevCfg> _devCfgs;
	UINT32 _tickFreq;
	UINT16 _curSong;
	UINT32 _playTicks;	// length of the song without the fade (0 = no length, plays forever)
	UINT32 _fadeTicks;	// length of the fade-out after _playTicks
	UINT32 _totalTicks;	// _playTicks + _fadeTicks (0 = plays forever)
	std::map<std::string, std::string> _tagData;
	std::vector<const char*> _tagList;

	// tick/sample conversion rates. The speed scales the rate of the PLAY calls; a tick is a millisecond of the
	// song at speed 1. A change of the speed during the song starts a new stretch of the conversion at the
	// position then (_tickBase ticks = _smplBase samples), so the position does not jump.
	UINT32 _speedNum;		// the playback speed in thousandths
	UINT32 _tickBase;
	UINT32 _smplBase;
	UINT64 _tsMult;
	UINT64 _tsDiv;

	NSF_PLAY_OPTIONS _playOpts;
	std::vector<PLR_DEV_OPTS> _devOpts;	// by device (the order of _devCfgs)
	std::vector<NSF_CHIPDEV> _devices;
	NSF_CHIPDEV* _dev[DEVKIND_COUNT];	// the device of a kind, NULL = the song does not have it

	// the machine
	NSF_CPU6502 _cpu;
	UINT8 _mem[0x10000];	// RAM, WRAM and the 4K windows of the program
	std::vector<UINT8> _bankRom;	// the program as 4K banks (bankswitched files)
	UINT8 _banks[10];		// current bank of the windows $8000-$FFFF (0-7) and $6000/$7000 (8, 9)
	UINT8 _mulA;			// MMC5 multiplier
	UINT8 _mulB;
	UINT8 _phase;
	UINT64 _cpuCycle;		// CPU cycles since the start of the song
	UINT64 _playBase;		// cycle of the first PLAY
	UINT32 _playIndex;		// number of the next PLAY
	UINT64 _nextPlay;		// cycle of the next PLAY

	// the Render() in progress
	WAVE_32BS* _rBuf;
	UINT32 _rBase;			// sample position of _rBuf[0]
	UINT32 _rEnd;			// sample position where this Render() ends
	UINT32 _devSmpl;		// sample position the devices have rendered to

	// The YM2413 core of the VRC7 has no output filter, as the cores of the other chips have: the console's
	// filter is applied here, to the output of the VRC7 device (it carries a DC offset that varies with the notes).
	struct NSF_VRC7_FILTER;
	NSF_VRC7_FILTER* _vrc7Filter;
	std::vector<WAVE_32BS> _vrc7Buf;	// the output of the VRC7 device before the filter

	UINT32 _playSmpl;	// output sample position
	UINT32 _playTick;	// position in ticks (derived from _playSmpl, or the target of a seek)

	UINT8 _playState;
};

#endif	// __NSFPLAYER_HPP__

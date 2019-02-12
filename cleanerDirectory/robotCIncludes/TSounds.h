#ifndef TSounds__H_
#define TSounds__H_

///////////////////////////////////////////////////////////////////////////////////////////////
//
//                              Sound Types
//
//////////////////////////////////////////////////////////////////////////////////////////////

typedef enum TSounds
{
#if !defined(bHasSoundsInRomChip) || defined(_WINDOWS)
	soundBlip             = 0,
	soundBeepBeep         = 1,
	soundDownwardTones    = 2,
	soundUpwardTones      = 3,
	soundLowBuzz          = 4,
	soundFastUpwardTones  = 5,
	soundShortBlip        = 6,
	soundException        = 7,
	soundLowBuzzShort     = 8,
#endif

#if defined(bHasSoundsInRomChip) || defined(_WINDOWS)
	// VEX IQ Integrated WAVE sounds
	kFirstSoundInChipRom,
	soundSiren2						= kFirstSoundInChipRom +  0,
	soundWrongWay         = kFirstSoundInChipRom +  1,
	soundWrongWays        = kFirstSoundInChipRom +  2,   // Slower Speed
	soundGasFillup        = kFirstSoundInChipRom +  3,
	soundHeadlightsOn     = kFirstSoundInChipRom +  4,
	soundHeadlightsOff    = kFirstSoundInChipRom +  5,
	soundTollBooth        = kFirstSoundInChipRom +  6,
	soundCarAlarm2        = kFirstSoundInChipRom +  7,
	soundTada             = kFirstSoundInChipRom +  8,
	soundGarageDoorClose  = kFirstSoundInChipRom +  9,
	soundRatchet          = kFirstSoundInChipRom + 10,
	soundAirWrench        = kFirstSoundInChipRom + 11,
	soundSiren4           = kFirstSoundInChipRom + 12,   // Seems to be identical to "Siren2"
	soundRatchet4         = kFirstSoundInChipRom + 13,
	soundCarAlarm4        = kFirstSoundInChipRom + 14,
	soundPowerOff2        = kFirstSoundInChipRom + 15,

#endif

#if !(defined(NXT))
	nNumbOfSystemSounds
#endif

#if defined(bHasSoundsInRomChip)
	,
	kLastSoundInChipRomPlusOne = nNumbOfSystemSounds,
	kNumbOfChipRomSounds = kLastSoundInChipRomPlusOne - kFirstSoundInChipRom

#endif
} TSounds;


#endif

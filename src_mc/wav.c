#include "wav.h"

#define CHUNKID					0x52494646 // big-endian
#define FORMAT					0x57415645 // big-endian
#define SUBCHUNK1ID			0x666d7420 // big-endian
#define SUBCHUNK1SIZE		0x10000000 // little-endian
#define AUDIOFORMAT			0x0100		 // little-endian
#define SAMPLERATE			0x44ac0000 // little-endian
#define BITSPERSAMPLE		0x1000		 // little-endian
#define SUBCHUNK2ID			0x64617461 // big-endian

struct Header getNewHeader(uint32_t numSamples, uint8_t channels){
	
	struct Header newHeader;
	
	newHeader.chunkID = CHUNKID;
	newHeader.chunkSize = 0; //placeholder
	newHeader.format = FORMAT;
	newHeader.subchunk1ID = SUBCHUNK1ID;
	newHeader.subchunk1Size = 0; //placeholder
	newHeader.audioFormat = AUDIOFORMAT;
	newHeader.numChannels = channels;
	newHeader.sampleRate = SAMPLERATE;
	newHeader.byteRate = SAMPLERATE * channels * BITSPERSAMPLE/8;
	newHeader.blockAlign = channels * BITSPERSAMPLE/8;
	newHeader.bitsPerSample = BITSPERSAMPLE;
	newHeader.subchunk2ID = SUBCHUNK2ID;
	newHeader.subchunk2Size = numSamples * channels * BITSPERSAMPLE/8;
	
	return newHeader;
}

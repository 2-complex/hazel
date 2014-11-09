
/*
  g2c Copyright (C) 2012 2-Complex

  This software is provided 'as-is', without any express or implied
  warranty.  In no event will the authors be held liable for any damages
  arising from the use of this software.

  Permission is granted to anyone to use this software for any purpose,
  including commercial applications, and to alter it and redistribute it
  freely, subject to the following restrictions:

  1. The origin of this software must not be misrepresented; you must not
     claim that you wrote the original software. If you use this software
     in a product, an acknowledgment in the product documentation would be
     appreciated but is not required.
  2. Altered source versions must be plainly marked as such, and must not be
     misrepresented as being the original software.
  3. This notice may not be removed or altered from any source distribution.
*/

#include "wave.h"
#include "log.h"

#include <string.h>
#include <stdlib.h>

struct RiffHeader {
    char riffTag[4]; /* Contains the letters "RIFF" in ASCII form
                         (0x52494646 big-endian form).*/
    
    int chunkSize;   /*36 + SubChunk2Size, or more precisely:
                       4 + (8 + SubChunk1Size) + (8 + SubChunk2Size)
                       This is the size of the rest of the chunk 
                       following this number.  This is the size of the 
                       entire file in bytes minus 8 bytes for the
                       two fields not included in this count:
                       riffTag and chunkSize.*/
};

struct WavHeader {
    char waveTag[4];  /* Contains the letters "WAVE"
                        (0x57415645 big-endian form). */
    
    /* The "WAVE" format consists of two subchunks: "fmt " and "data":
       The "fmt " subchunk describes the sound data's format:*/
    
    char fmtTag[4];  /*Contains the letters "fmt " (0x666d7420 big-endian form).*/
    
    
    int Subchunk1Size;    /*16 for PCM.  This is the size of the
                          rest of the Subchunk which follows this number.*/
    
    short audioFormat;    /* PCM = 1 (i.e. Linear quantization)
                               Values other than 1 indicate some
                               form of compression.*/
    
    short numChannels;   /*Mono = 1, Stereo = 2, etc.*/
    
    int sampleRate;      /*8000, 44100, etc.*/
    
    int byteRate;        /* == sampleRate * numChannels * bitsPerSample/8*/
    
    short  blockAlign;   /* == numChannels * bitsPerSample/8
                               The number of bytes for one sample including
                               all channels. */
    
    short bitsPerSample;  /* 8 bits = 8, 16 bits = 16, etc. */
    
    /* The "data" subchunk contains the size of the data and the actual sound: */
    
    char dataTag[4];  /*Contains the letters "data"
                            (0x64617461 big-endian form).*/
    
    int   dataSize;  /* == NumSamples * numChannels * bitsPerSample/8
                               This is the number of bytes in the data.
                               You can also think of this as the size
                               of the read of the subchunk following this 
                               number.*/
};


void checkTag(const char* tag, const char* exemplar)
{
    if( !(tag[0] == exemplar[0] &&
          tag[1] == exemplar[1] &&
          tag[2] == exemplar[2] &&
          tag[3] == exemplar[3] ) )
    {
        g2cerror( "Wave parser expected tag '%c%c%c%c', found '%c%c%c%c'.\n",
            exemplar[0], exemplar[1],
            exemplar[2], exemplar[3],
            tag[0], tag[1],
            tag[2], tag[3] );
        exit(0);
    }
}


void Wave::initWithData(const uint8_t* data, size_t size)
{
    if( size < sizeof(RiffHeader) )
    {
        g2cerror( "Wav data too small for riff header.\n" );
        exit(0);
    }
    
    RiffHeader* riff = (RiffHeader*)(data);
    memcpy(riff, data, sizeof(RiffHeader));
    
    checkTag(riff->riffTag, "RIFF");
    
    if( size < sizeof(RiffHeader) + sizeof(WavHeader) )
    {
        g2cerror( "Wav data too small both riff header and wav header."
            "  Cannot read.\n" );
        exit(0);
    }
    
    WavHeader* wav = (WavHeader*)(data + sizeof(RiffHeader));
    
    checkTag(wav->waveTag, "WAVE");
    checkTag(wav->fmtTag, "fmt ");
    checkTag(wav->dataTag, "data");
    
    if( wav->audioFormat!=1 )
    {
        g2cerror( "Wave expected audio format 1 representing"
            " audio PCM format, %d found instead.\n", wav->audioFormat );
    }
        
    if( wav->bitsPerSample%8 != 0 || wav->bitsPerSample<=0 || wav->bitsPerSample > 4096 )
    {
        g2clog( "WARNING: Wave bits-per-sample = %d\n", wav->bitsPerSample );
    }
    
    if( (wav->sampleRate) <= 0 || (wav->sampleRate) > 250000 )
    {
        g2clog( "WARNING: Wave sample-rate = %d\n", wav->sampleRate );
    }
    
    sampleRate = wav->sampleRate;
    numChannels = wav->numChannels;
    bytesPerSample = wav->bitsPerSample / 8;
    numSamples = wav->dataSize / bytesPerSample;
    this->data = (uint8_t*)(wav+1);
    
    if( this->data + wav->dataSize > data + size )
    {
        g2clog( "Size of data encoded in wav exceeds size of data given." );
        exit(0);
    }
}



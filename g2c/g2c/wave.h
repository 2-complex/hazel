
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

#ifndef _WAVE_
#define _WAVE_

#include <stdint.h>
#include <stddef.h>

/*! A utility class for Wave (as in .wav file) sound data.  To use Wave, open
    a .wav file, and read in all the data to memory.  Then initialize the Wave
    object using initWithData().  That function will populate the public members
    with the sample-rate number of channels number of samples and bytes-per-sample
    from the file.  The member data will point to the first byte in the raw PCM
    sound data.*/
class Wave {
public:
    /*! The number of samples per second.  44100 for CD-quality sound stereo or mono.*/
    int sampleRate;

    /*! 1 for mono 2 for stereo. */
    int numChannels;

    /*! The number of samples.  Each sample is a a packet of data consisting of one
       number for each channel.  So, for CD-quality stero a sample is two numebrs,
       a total of 4 bytes.*/
    int numSamples;

    /*! 1 for 8-bit sound, 2 for 16-bit sound.*/
    int bytesPerSample;

    /*! A pointer to the raw pcm audio data.*/
    uint8_t* data;
    
    /*! Initialize the Wave file with a pointer to .wav file data.  This populates
        the member variables.*/
    void initWithData(const uint8_t* data, size_t size);
};

#endif


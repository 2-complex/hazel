
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

#ifndef _AUDIO_PLAYER_
#define _AUDIO_PLAYER_

#include <stdint.h>

namespace g2c
{

class AudioPlayer
{
public:
    virtual ~AudioPlayer();

    virtual int createContext();
    virtual void destroyContext(int index);

    virtual int createSource();
    virtual void destroySource(int index);

    virtual int createSound();
    virtual void destroySound(int index);

    virtual void makeContextCurrent(int index);

    virtual bool isSourcePlaying(int index);

    virtual void stopSource(int index);

    virtual void loadSound(int index, int sampleRate, int numSamples, int numChannels,
        int bytesPerSample, const uint8_t* data);

    virtual void playSound(int soundIndex, int sourceIndex, bool loop, double gain);
};

}

#endif


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

#include "openalplayer.h"
#include "log.h"

#include <stdlib.h>
#include <string>

using namespace std;

namespace g2c {

int OpenALPlayer::alcDeviceRefCounter = 0;
ALCdevice* OpenALPlayer::alcDevice = NULL;

int OpenALPlayer::createContext()
{
    if(!OpenALPlayer::alcDevice)
        OpenALPlayer::alcDevice = alcOpenDevice(NULL);
    
    ContextInfo info;
    
    info.device = alcDevice;
    alcDeviceRefCounter++;
    
    info.context = alcCreateContext(info.device, 0);

    if ( !info.context )
    {
        // On linux this returns an error, but the sound plays.
        // So, we check to make sure the context is null before retrieving the error.
        ALenum  error = AL_NO_ERROR;
        if((error = alGetError()) != AL_NO_ERROR)
        {
            g2cerror("OpenAL error in context initialization: 0x%x\n", error);
            exit(0);
        }
    }

    int index = 1;
    while( contextInfo.find(index)!=contextInfo.end() )
        index++;
    
    contextInfo[index] = info;
    
    return index;
}

void OpenALPlayer::makeContextCurrent(int index)
{
    map<int, ContextInfo>::iterator itr = contextInfo.find(index);
    if( itr == contextInfo.end() )
    {
        g2cerror("Attempt to make context current that doesn't exist. index=%d\n", index);
        exit(0);
    }
    
    ALCcontext* context = itr->second.context;
    
    alcMakeContextCurrent(context);
}

bool OpenALPlayer::isSourcePlaying(int index)
{
    ALint state = 0;
    alGetSourcei(index, AL_SOURCE_STATE, &state);
    return state == AL_PLAYING;
}

void OpenALPlayer::stopSource(int index)
{
    alSourceStop(index);
}

void OpenALPlayer::destroyContext(int index)
{
    map<int, ContextInfo>::iterator itr = contextInfo.find(index);
    if( itr == contextInfo.end() )
    {
        g2cerror("Attempt to destroy context that doesn't exist. index=%d\n", index);
        exit(0);
    }
    
    ALCdevice* device = itr->second.device;
    ALCcontext* context = itr->second.context;
    
    alcSuspendContext(context);
    alcMakeContextCurrent(NULL);
    alcDestroyContext(context);
    
    alcDeviceRefCounter--;
    if(alcDeviceRefCounter==0)
        alcCloseDevice(device);
    
    contextInfo.erase(index);
}

int OpenALPlayer::createSource()
{
    ALuint index = 0;
    alGenSources(1, &index);
    
    return index;
}

void OpenALPlayer::destroySource(int index)
{
    ALuint i = index;
    alDeleteSources(1, &i);
}

int OpenALPlayer::createSound()
{
    ALuint index = 0;
    alGenBuffers(1, &index);
    return index;
}

void OpenALPlayer::destroySound(int index)
{
    ALuint i = index;
    alDeleteBuffers(1, &i);
}

void OpenALPlayer::loadSound(int index, int sampleRate, int numSamples, int numChannels,
    int bytesPerSample, const uint8_t* data)
{
    int format = getOpenALFormat(numChannels, bytesPerSample);
    int size = numSamples * bytesPerSample;
    
    alBufferData(index, format, data, size, sampleRate);
}

void OpenALPlayer::playSound(int soundIndex, int sourceIndex, bool loop, double gain)
{
    alSourcei(sourceIndex, AL_BUFFER, soundIndex);
    alSourcei(sourceIndex, AL_LOOPING, loop);
    alSourcef(sourceIndex, AL_GAIN, gain);
    alSourcePlay(sourceIndex);
}

int OpenALPlayer::getOpenALFormat(int numChannels, int bytesPerSample)
{
    string ch, b;
    switch( numChannels )
    {
        case 1: ch = "MONO"; break;
        case 2: ch = "STEREO"; break;
        case 4: ch = "QUAD"; break;
        case 6: ch = "51CHN"; break;
        case 7: ch = "CHN61"; break;
        case 8: ch = "71CHN"; break;
        default:
            g2cerror( "Error, wave number of channels %d not 1,2,4,6,7 or 8.\n", numChannels );
            exit(0);
        break;
    }
    
    switch( bytesPerSample )
    {
        case 1: b = "8"; break;
        case 2: b = "16"; break;
        case 4:
            b = "_FLOAT32";
            g2clog( "WARNING: Wave bytes per sample 32."
                "  OpenAL supports 32-bit float only, attempting float.\n" );
        break;
    }
    
    return alGetEnumValue(("AL_FORMAT_"+ch+b).c_str());
}


}


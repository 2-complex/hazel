
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

#include "openslplayer.h"
#include "log.h"

#include <stdlib.h>
#include <string>

using namespace std;

namespace g2c
{

// this callback handler is called every time a buffer finishes playing
void bqPlayerCallback(SLAndroidSimpleBufferQueueItf bq, void *ptr)
{
    OpenSLPlayer::SourceInfo* sourceInfo =
        reinterpret_cast<OpenSLPlayer::SourceInfo*>(ptr);

    if( bq != sourceInfo->bqPlayerBufferQueue )
    {
        g2cerror(
            "OpenSL callback called with interface"
            " not euqal to buffer queue as expected.\n");
        exit(0);
    }

    if( sourceInfo->loop )
    {
        SLresult result;

        result = (*(sourceInfo->bqPlayerBufferQueue))->Enqueue(
            sourceInfo->bqPlayerBufferQueue,
            sourceInfo->audio->buffer,
            sourceInfo->audio->size);

        if( SL_RESULT_SUCCESS != result )
            g2clog("sound failed to loop");
    }
}


OpenSLPlayer::OpenSLPlayer()
    : contextIndex(0)
{
}

int OpenSLPlayer::createContext()
{
    SLresult result;

    ContextInfo info;

    // create engine
    result = slCreateEngine(&info.engineObject, 0, NULL, 0, NULL, NULL);
    if( SL_RESULT_SUCCESS != result )
    {
        g2cerror("OpenSL Failed to get create engine.\n");
        exit(0);
    }

    // realize the engine
    result = (*info.engineObject)->Realize(info.engineObject, SL_BOOLEAN_FALSE);
    if( SL_RESULT_SUCCESS != result )
    {
        g2cerror("OpenSL Failed to realize engine object.\n");
        exit(0);
    }

    // get the engine interface, which is needed in order to create other objects
    result = (*info.engineObject)->GetInterface(info.engineObject, SL_IID_ENGINE, &info.engine);
    if( SL_RESULT_SUCCESS != result )
    {
        g2cerror("OpenSL Failed to get interface engine.\n");
        exit(0);
    }

    // create output mix, with environmental reverb specified as a non-required interface
    const SLInterfaceID ids[0] = {};
    const SLboolean req[0] = {};
    result = (*info.engine)->CreateOutputMix(info.engine, &info.outputMixObject, 0, NULL, NULL);
    if( SL_RESULT_SUCCESS != result )
    {
        g2cerror("OpenSL Failed to create output mix.\n");
        exit(0);
    }

    // realize the output mix
    result = (*info.outputMixObject)->Realize(info.outputMixObject, SL_BOOLEAN_FALSE);

    if( SL_RESULT_SUCCESS != result )
    {
        g2cerror("Failed to initialize OpenSL.\n");
        exit(0);
    }

    int index = 1;
    while( contextInfos.find(index)!=contextInfos.end() )
        index++;

    contextInfos[index] = info;

    return index;
}

void OpenSLPlayer::destroyContext(int index)
{
    map<int, ContextInfo>::iterator itr = contextInfos.find(index);
    if( itr == contextInfos.end() )
    {
        g2cerror("Attempt to make context current that doesn't exist. index=%d\n", index);
        exit(0);
    }

    ContextInfo& info(itr->second);

    // destroy output mix object, and invalidate all associated interfaces
    if (info.outputMixObject != NULL) {
        (*info.outputMixObject)->Destroy(info.outputMixObject);
        info.outputMixObject = NULL;
        info.outputMixEnvironmentalReverb = NULL;
    }

    // destroy engine object, and invalidate all associated interfaces
    if (info.engineObject != NULL) {
        (*info.engineObject)->Destroy(info.engineObject);
        info.engineObject = NULL;
        info.engine = NULL;
    }
}

int OpenSLPlayer::createSource()
{
    map<int, ContextInfo>::iterator itr = contextInfos.find(contextIndex);
    if( itr == contextInfos.end() )
    {
        g2cerror(
            "Attempt to create source from context that doesn't exist."
            " index=%d\n", contextIndex);
        exit(0);
    }

    ContextInfo& contextInfo(itr->second);

    SourceInfo* sourceInfo = new SourceInfo;

    SLresult result;

    // configure audio source
    SLDataLocator_AndroidSimpleBufferQueue loc_bufq = {SL_DATALOCATOR_ANDROIDSIMPLEBUFFERQUEUE, 2};
    SLDataFormat_PCM format_pcm = {SL_DATAFORMAT_PCM, 1, SL_SAMPLINGRATE_44_1,
        SL_PCMSAMPLEFORMAT_FIXED_16, SL_PCMSAMPLEFORMAT_FIXED_16,
        SL_SPEAKER_FRONT_CENTER, SL_BYTEORDER_LITTLEENDIAN};
    SLDataSource audioSrc = {&loc_bufq, &format_pcm};

    // configure audio sink
    SLDataLocator_OutputMix loc_outmix = {SL_DATALOCATOR_OUTPUTMIX, contextInfo.outputMixObject};
    SLDataSink audioSnk = {&loc_outmix, NULL};

    // create audio player
    const SLInterfaceID ids[3] = {SL_IID_BUFFERQUEUE, SL_IID_EFFECTSEND, SL_IID_VOLUME};
    const SLboolean req[3] = {SL_BOOLEAN_TRUE, SL_BOOLEAN_TRUE, SL_BOOLEAN_TRUE};

    result = (*contextInfo.engine)->CreateAudioPlayer(
        contextInfo.engine, &(sourceInfo->bqPlayerObject),
        &audioSrc, &audioSnk, 3, ids, req);

    if( SL_RESULT_SUCCESS != result )
    {
        g2cerror("OpenSL Failed to initialize OpenSL audio player.\n");
        exit(0);
    }

    int index = 1;
    while( sourceInfos.find(index) != sourceInfos.end() )
        index++;

    sourceInfos[index] = sourceInfo;
    sourceInfo->realize();

    return index;
}

void OpenSLPlayer::destroySource(int index)
{
    if( sourceInfos.find(index) == sourceInfos.end() )
    {
        g2cerror("OpenSL Attempt to destroy non-existant source.");
        exit(0);
    }

    SourceInfo* info(sourceInfos[index]);

    // destroy buffer queue audio player object, and invalidate all associated interfaces
    if( info->bqPlayerObject != NULL )
    {
        (*info->bqPlayerObject)->Destroy(info->bqPlayerObject);
        info->bqPlayerObject = NULL;
        info->bqPlayerPlay = NULL;
        info->bqPlayerBufferQueue = NULL;
        info->bqPlayerEffectSend = NULL;
        info->bqPlayerMuteSolo = NULL;
        info->bqPlayerVolume = NULL;
    }

    sourceInfos.erase(index);

    delete info;
}


int OpenSLPlayer::createSound()
{
    Audio* audio = new Audio;

    int index = 1;
    while( audios.find(index)!=audios.end() )
        index++;

    audios[index] = audio;

    return index;
}

void OpenSLPlayer::destroySound(int index)
{
    map<int, Audio*>::iterator itr = audios.find(index);
    if( itr == audios.end() )
    {
        g2cerror("OpenSL Attempt to destroy audio that doesn't exist. index=%d\n", index);
        exit(0);
    }

    delete audios[index];
    audios.erase(index);
}

void OpenSLPlayer::makeContextCurrent(int index)
{
    map<int, ContextInfo>::iterator itr = contextInfos.find(index);
    if( itr == contextInfos.end() )
    {
        g2cerror("OpenSL Attempt to make context current that doesn't exist. index=%d\n", index);
        exit(0);
    }

    contextIndex = index;
}

void OpenSLPlayer::loadSound(
    int index,
    int sampleRate,
    int numSamples,
    int numChannels,
    int bytesPerSample,
    const uint8_t* data)
{
    map<int, Audio*>::iterator itr = audios.find(index);
    if( itr == audios.end() )
    {
        g2cerror("OpenSL Attempt to load into non-existant audio buffer. index=%d\n", index);
        exit(0);
    }

    Audio* audio = itr->second;

    if( audio->buffer )
    {
        delete[] audio->buffer;
        audio->buffer = NULL;
    }

    audio->size = numSamples*bytesPerSample;
    audio->buffer = new uint8_t[audio->size];
    memcpy(audio->buffer, data, audio->size);
}

void OpenSLPlayer::playSound(
    int soundIndex,
    int sourceIndex,
    bool loop,
    double gain)
{
    OpenSLPlayer::Audio* audio = audios[soundIndex];
    OpenSLPlayer::SourceInfo* sourceInfo = sourceInfos[sourceIndex];

    sourceInfo->loop = loop;
    sourceInfo->audio = audio;

    if( audio->size > 0 )
    {
        SLresult result;

        result = (*(sourceInfo->bqPlayerBufferQueue))->Clear(
            sourceInfo->bqPlayerBufferQueue);

        result = (*(sourceInfo->bqPlayerBufferQueue))->Enqueue(
            sourceInfo->bqPlayerBufferQueue, audio->buffer, audio->size);

        if( SL_RESULT_SUCCESS != result )
            g2clog("sound failed to play");
    }
}

bool OpenSLPlayer::isSourcePlaying(int index)
{
    OpenSLPlayer::SourceInfo* sourceInfo = sourceInfos[index];
    SLAndroidSimpleBufferQueueState state;

    SLresult result = (*(sourceInfo->bqPlayerBufferQueue))->GetState(
        sourceInfo->bqPlayerBufferQueue, &state);

    if( SL_RESULT_SUCCESS != result )
        g2clog("sound failed to play");

    return state.count > 0;
}

void OpenSLPlayer::stopSource(int index)
{
    OpenSLPlayer::SourceInfo* sourceInfo = sourceInfos[index];

    SLresult result;

    sourceInfo->loop = false;

    result = (*(sourceInfo->bqPlayerBufferQueue))->Clear(
        sourceInfo->bqPlayerBufferQueue);

    if( SL_RESULT_SUCCESS != result )
        g2clog("source failed to stop");
}

OpenSLPlayer::Audio::Audio()
    : buffer(NULL)
    , size(0)
{
}

OpenSLPlayer::Audio::~Audio()
{
    if( buffer )
        delete[] buffer;
}

OpenSLPlayer::SourceInfo::SourceInfo() : bqPlayerObject(NULL)
{
}

OpenSLPlayer::SourceInfo::~SourceInfo()
{
}

void OpenSLPlayer::SourceInfo::realize()
{
    SLresult result;

    // realize the player
    result = (*bqPlayerObject)->Realize(bqPlayerObject, SL_BOOLEAN_FALSE);
    if( SL_RESULT_SUCCESS != result )
    {
        g2cerror("OpenSL Failed to realize player.");
        exit(0);
    }

    // get the play interface
    result = (*bqPlayerObject)->GetInterface(bqPlayerObject, SL_IID_PLAY, &bqPlayerPlay);
    if( SL_RESULT_SUCCESS != result )
    {
        g2cerror("OpenSL Failed to get player interface.");
        exit(0);
    }

    // get the buffer queue interface
    result = (*bqPlayerObject)->GetInterface(bqPlayerObject, SL_IID_BUFFERQUEUE,
        &bqPlayerBufferQueue);
    if( SL_RESULT_SUCCESS != result )
    {
        g2cerror("OpenSL Failed to get player interface.");
        exit(0);
    }

    // register callback on the buffer queue
    result = (*bqPlayerBufferQueue)->RegisterCallback(
        bqPlayerBufferQueue, bqPlayerCallback, this);
    if( SL_RESULT_SUCCESS != result )
    {
        g2cerror("OpenSL Failed register callback.");
        exit(0);
    }

    // get the effect send interface

    result = (*bqPlayerObject)->GetInterface(bqPlayerObject, SL_IID_EFFECTSEND,
        &bqPlayerEffectSend);
    if( SL_RESULT_SUCCESS != result )
    {
        g2cerror("OpenSL Failed to get effect send interface.");
        exit(0);
    }

    // get the volume interface
    result = (*bqPlayerObject)->GetInterface(
        bqPlayerObject, SL_IID_VOLUME, &bqPlayerVolume);
    if( SL_RESULT_SUCCESS != result )
    {
        g2cerror("OpenSL Failed to get volume control interface.");
        exit(0);
    }

    // set the player's state to playing
    result = (*bqPlayerPlay)->SetPlayState(bqPlayerPlay, SL_PLAYSTATE_PLAYING);
    if( SL_RESULT_SUCCESS != result )
    {
        g2cerror("OpenSL Failed to set player state to PLAYING.");
        exit(0);
    }
}

} // end namespace


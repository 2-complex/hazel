
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

#include "sound.h"
#include "util.h"

#include "log.h"

#include <stdlib.h>

using namespace std;

namespace g2c {

Context* Context::currentContext = NULL;

Context::Context(AudioPlayer* player) : index(0), player(player)
{
    index = player->createContext();
}

Context::~Context()
{
    player->destroyContext(index);
    if( Context::currentContext == this )
        Context::currentContext = NULL;
}

void Context::makeCurrent()
{
    if( index <= 0 )
    {
        g2cerror("Attempt to make context current which was not initialized properly.\n");
        exit(0);
    }

    currentContext = this;
    player->makeContextCurrent(index);
}

Source::Source() : index(0)
{
    if( !Context::currentContext )
    {
        g2cerror("Attempt to create Source with no current context.\n");
        exit(0);
    }

    player = Context::currentContext->player;

    if( !player )
    {
        g2cerror("Attempt to make Source with no player.\n");
        exit(0);
    }

    index = player->createSource();
}

Source::~Source()
{
    if( index <= 0 )
    {
        g2cerror("Attempt to destroy Source which was not initialized properly.\n");
        exit(0);
    }

    player->destroySource(index);
}

bool Source::isPlaying() const
{
    return player->isSourcePlaying(index);
}

Sound::Sound() : index(0), source(NULL), loop(false)
{
    type = "Sound";

    if( !Context::currentContext )
    {
        g2cerror("Attempt to create Sound with no current context.\n");
        exit(0);
    }

    player = Context::currentContext->player;

    if( !player )
    {
        g2cerror("Attempt to create Sound with no player set.\n");
        exit(0);
    }

    index = player->createSound();

    if( index <= 0 )
    {
        g2cerror("Player failed to create sound.\n");
        exit(0);
    }
}

Sound::~Sound()
{
    if( index <= 0 )
    {
        g2cerror("Attempt to destroy Sound which was not initialized properly.\n");
        exit(0);
    }

    player->destroySound(index);
}

void Sound::play(double gain) const
{
    if( !source )
    {
        g2cerror("Attempt to play Sound with no source.\n");
        exit(0);
    }

    player->playSound(index, source->index, loop, gain);
}

void Sound::stop() const
{
    if( !source )
    {
        g2cerror("Attempt to stop Sound with no source.\n");
        exit(0);
    }

    player->stopSource(source->index);
}

string Sound::serializeElements(std::string indent) const
{
    string r = Serializable::serializeElements(indent);
    r += TAB + indent + "'file' : " + string("'") + file + "',\n";
    if(loop)
        r += TAB + indent + "'loop' : " + intToString(loop) + ",\n";
    return r;
}

void Sound::handleChild(const parse::Node* n)
{
    string n_name = n->data.s;
    parse::Node* value = n->data.value;
    if(n_name == "file")
        file = value->data.s;

    if(n_name == "name")
        name = value->data.s;

    if(n_name == "file")
        file = value->data.s;

    if(n_name == "loop")
        loop = value->data.i;
}

void Sound::load(int sampleRate, int numSamples, int numChannels,
    int bytesPerSample, const uint8_t* data)
{
    if( !player )
    {
        g2cerror("Attempt to load sound with no player.");
        exit(0);
    }

    player->loadSound(index, sampleRate, numSamples, numChannels,
        bytesPerSample, data);
}

void Sound::initWithWave(const Wave& wave)
{
    load(wave.sampleRate, wave.numSamples, wave.numChannels,
        wave.bytesPerSample, wave.data);
}


} // end namespace


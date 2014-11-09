
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

#ifndef _SOUND_
#define _SOUND_

#include "wave.h"
#include "serializable.h"
#include "audioplayer.h"

namespace g2c {

class Context;

/*! A Context represents a 3-dimensional space where sounds can eminate from sources.
    To use a context, construct with a pointer to an object which is a subclass of
    AudioPlayer, then call makeCurrent().  Once the context is curent, you can use 
    Source and Sound objects to make sounds.*/
class Context {
friend class Sound;
friend class Source;
public:
    explicit Context(AudioPlayer* player);
    virtual ~Context();

    /*! Call this function to use the context.  Future calls on Source and Sound objects
        will apply to this context, and they will use the AudioPlayer object with which
        the context was initialized.*/
    void makeCurrent();

private:
    int index;
    AudioPlayer* player;

    static Context* currentContext;
};


/*! A Source represents a position in space from which a sound eminates.  To use play a sound with
    a source, instantate a Source object, assign the source of a Sound object to it and call the
    sound's play() function.

    A Source can play one sound at a time.  If a sound is playing on a Source and another sound
    plays before that sound is finished.  The old sound will stop immediately and the new sound
    starts immediately. */
class Source {
friend class Sound;
public:
    Source();
    virtual ~Source();

    /*! Returns true if the sound is currently playing.*/
    bool isPlaying() const;

private:
    int index;
    AudioPlayer* player;
};

/*! A Sound represents a single sound effect resource.  To play a sound, make sure a context is current,
    initialize using initWithWave(), assign a Source object to the source member variable, then call
    the play() function.*/
class Sound : public Serializable {
public:
    Sound();
    virtual ~Sound();

    std::string file;

    /*! Set the source of the sound to a Source object and call play() to play the sound.*/
    mutable Source* source;

    /*! Set this flag to true to make the sound automatically play over and over.*/
    bool loop;

    /*! Initialize this sound with a Wave object.  A Wave object stores raw PCM sound data in an array
        in CPU memory.  A Sound represents an object ready for playing using a low-level sound engine.
        Use this function to initialize the Sound with PCM data from the Wave.*/
    void initWithWave(const Wave& wave);

    /*! Play the sound.  Make sure a Context is current, and the Sound has been initialized and assigned
        a Source.*/
    void play(double gain = 1.0) const;

    /*! Stop an already playing sound immediately.*/
    void stop() const;

    virtual std::string serializeElements(std::string indent = "") const;
    virtual void handleChild(const parse::Node* n);

    // Temprary fix for MacBank which has code in it which appeals directly to OpenAL.
    // Sheesh.
    int getIndex() {return index;}

    // Was private, not sure if it should be private.
    void load(int sampleRate, int numSamples, int numChannels,
        int bytesPerSample, const uint8_t* data);

private:
    int index;
    AudioPlayer* player;
};

} // end namespace

#endif


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


#ifndef _BANK_
#define _BANK_

#include "serializable.h"
#include "sound.h"
#include "texture.h"

#include <queue>

namespace g2c {

/*! A generic binary data container.  Keeps an privately allocated array of bytes which
    can be resized and accessed.  ~Data() frees the array if it has been allocated.*/
class Data {
public:
    Data();
    virtual ~Data();
    uint8_t* array() const;
    size_t size() const;
    void resize(size_t size);
    
private:
    size_t mSize;
    uint8_t* mArray;
};


/*! Bank is used for cross-platform development.  Bank is an abstract class with methods for
    retrieving resources such as images, sounds, and strings.  Since retrieval of such resource
    data tends to be platform specific, Bank is meant to be subclassed for a particular platform.*/
class Bank {
public:
    Bank() {}
    virtual ~Bank() {}

    /*! Populate a Data object with the binary data from the file indicated by path.*/
    virtual void initDataWithPath(Data* data, const char* path) = 0;

    /*! Retrieves a serializable object from persistent memory with the given key.  "Persistent"
        means that the data persists after the app is closed and reopened, such as preferences
        or saved games.*/
    virtual void initPersistentSerializableWithKey(Serializable* s, const char* key) = 0;

    /*! Saves a serializable object to persistent memory given a key.  "Persistent" means that
        the data persists after the app is closed and reopened, such as preferences or saved games.*/
    virtual void writePersistentSerializableWithKey(const Serializable* s, const char* key) = 0;

    /*! Populate a serializable object with using the json string in the file indicated by path.*/
    virtual void initSerializableWithPath(Serializable* s, const char* path) = 0;

    /*! Saves a serializable object to the file indicated by the path.  This does not work in some
        implementations.*/
    virtual void writeSerializableToPath(const Serializable* s, const char* path) = 0;

    /*! Populates a Sound object with the sound from the file indicated by path.  Must be a sound file
        of some sort, but .wav is required to work.*/
    virtual void initSoundWithPath(Sound* sound, const char* path);

    /*! Initializes a texture populating it with the bitmap data from the image file at path.  Must be
        an image file of some sort, but .png is required to work.*/
    virtual void initTextureWithPath(Texture2D* texture, const char* path) = 0;

    /*! Populates a Bitmap object with data from the image file indicated by path.  Must be an image file
        of some sort, but .png is required to work.*/
    virtual void initBitmapWithPath(Bitmap* bitmap, const char* path) = 0;
};


/*! AsynchronousBank is used for loading resources one-by-one with a progress indicator.  Appeals to
    member bank to load the resources.  The function step() gets one resource and the function
    percent() reports the percent of the resources loaded. */
class AsynchronousBank : public Bank {
public:
    AsynchronousBank() : bank(NULL), biggestSize(1) {}
    virtual ~AsynchronousBank() {}
    
    Bank* bank;
    
    struct LoadInstruction
    {
        LoadInstruction(Serializable* resource, const std::string& path)
            : resource(resource)
            , path(path)
        {
        }

        Serializable* resource;
        std::string path;
    };
    
    virtual void initPersistentSerializableWithKey(Serializable* s, const char* key);
    virtual void writePersistentSerializableWithKey(const Serializable* s, const char* key);

    virtual void initSerializableWithPath(Serializable* s, const char* path);
    virtual void writeSerializableToPath(const Serializable* s, const char* path);

    virtual void initSoundWithPath(Sound* sound, const char* path);
    virtual void initTextureWithPath(Texture2D* texture, const char* path);

    /*! Pop a resource from the instruction queue and load it. */
    bool step();

    /*! Get a percentage (out of 100) of the resources from the queue that have been loaded. */
    int percent() const;

private:
    mutable int biggestSize;
    std::queue<LoadInstruction> q;
};

} // end namespace


#endif



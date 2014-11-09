
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



#ifndef _MAC_BANK_
#define _MAC_BANK_

#include "bank.h"

#include <AudioToolbox/AudioToolbox.h>
#include <OpenAL/al.h>
#include <OpenAL/alc.h>

#include <QuartzCore/QuartzCore.h>

#include <queue>

namespace g2c
{

/*! An partial implementation of Bank for Mac OSX.  Contains much of the functionality for loading
    resources from files in iOS as well.  Use the subclass IOSResourceBank to load resources
    in an iOS app.  Use the subclass MacFileSystemBank to load resources from files in a local
    directory strucutre on a Mac.*/
class MacBank : public Bank
{
public:
    MacBank() {}
    virtual ~MacBank() {}
    
    std::string base_path;
    
    virtual void initDataWithPath(Data* data, const char* path);
    
    virtual void initPersistentSerializableWithKey(Serializable* s, const char* key);
    virtual void writePersistentSerializableWithKey(const Serializable* s, const char* key);
    
    virtual void initSerializableWithPath(Serializable* s, const char* path);
    virtual void writeSerializableToPath(const Serializable* s, const char* path);
    virtual void initTextureWithPath(Texture2D* texture, const char* path);
    
    virtual void initSoundWithPath(Sound* sound, const char* path);

    void* getOpenALAudioData(CFURLRef inFileURL,
                     ALsizei* outDataSize,
                     ALenum* outDataFormat,
                     ALsizei* outSampleRate) const;
    
protected:
    std::string directory;
    
    void initTextureWithCGImage(Texture2D* texture, CGImageRef image);
    void initBitmapWithCGImage(Bitmap* bitmap, CGImageRef image);
};

/*! An full implementation of Bank for loading resources from files in the file system on a Mac.*/
class MacFileSystemBank : public MacBank
{
public:
    MacFileSystemBank() {}
    virtual ~MacFileSystemBank() {}
    virtual void initBitmapWithPath(Bitmap* bitmap, const char* path);
};

} // end namespace


#endif


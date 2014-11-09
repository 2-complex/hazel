
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


#ifndef _UNIX_BANK_
#define _UNIX_BANK_

#include "bank.h"

#include <queue>

namespace g2c
{

/*! An implementation of Bank for linux/unix-based systems.  Loads uses unix libraries to load
    bitmap data from image files, raw audio from sound files etc, uses standard libaries for
    loading raw data from a file. */
class UnixBank : public Bank {
public:
    UnixBank() {}
    virtual ~UnixBank() {}
    
    std::string base_path;

    virtual void initPersistentSerializableWithKey(Serializable* s, const char* key);
    virtual void writePersistentSerializableWithKey(const Serializable* s, const char* key);
    virtual void initDataWithPath(Data* data, const char* path);
    virtual void initSerializableWithPath(Serializable* s, const char* path);
    virtual void writeSerializableToPath(const Serializable* s, const char* path);
    virtual void initTextureWithPath(Texture2D* texture, const char* path);
    virtual void initBitmapWithPath(Bitmap* bitmap, const char* path);
    virtual void initSoundWithPath(Sound* sound, const char* path);

protected:
    std::string directory;
};

} // end namespace

#endif


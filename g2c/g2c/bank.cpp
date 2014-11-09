
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


#include "bank.h"
#include "util.h"

#include <stdlib.h>

using namespace std;

namespace g2c {

Data::Data() : mArray(NULL), mSize(0) {}

Data::~Data()
{
    if( mArray )
        free(mArray);
}

void Data::resize(size_t size)
{
    if( mArray )
    {
        free(mArray);
        mArray = NULL;
    }
    
    if( size > 0 )
    {
        mArray = (uint8_t*)malloc(size);
    }
    
    mSize = size;
}

uint8_t* Data::array() const
{
    return mArray;
}

size_t Data::size() const
{
    return mSize;
}

void Bank::initSoundWithPath(Sound* sound, const char* path)
{
    if( endsWith(path, ".wav") || endsWith(path, ".wave") ||
        endsWith(path, ".WAV") || endsWith(path, ".WAVE") )
    {
    	Data data;
        Wave wave;
        
        initDataWithPath(&data, path);
        
        wave.initWithData(data.array(), data.size());
        
        sound->initWithWave(wave);
    }
}

void AsynchronousBank::initPersistentSerializableWithKey(Serializable* s, const char* key)
{
    bank->initPersistentSerializableWithKey(s, key);
}

void AsynchronousBank::writePersistentSerializableWithKey(const Serializable* s, const char* key)
{
    bank->writePersistentSerializableWithKey(s, key);
}

void AsynchronousBank::initSerializableWithPath(Serializable* s, const char* path)
{
    q.push(LoadInstruction(s, path));
}

void AsynchronousBank::writeSerializableToPath(const Serializable* s, const char* path)
{
    bank->writeSerializableToPath(s, path);
}

void AsynchronousBank::initSoundWithPath(Sound* sound, const char* path)
{
    q.push(LoadInstruction(sound, path));
}

void AsynchronousBank::initTextureWithPath(Texture2D* texture, const char* path)
{
    q.push(LoadInstruction(texture, path));
}

bool AsynchronousBank::step()
{
    if(!bank)
    {
        g2cerror( "AsynchronousBank used with no bank set.\n" );
        exit(0);
    }
    
    if( q.empty() )
        return false;
    
    LoadInstruction inst = q.front();
    
    if( inst.resource->type == "Texture" ||
        inst.resource->type == "Texture2D" ||
        inst.resource->type == "Sampler" ||
        inst.resource->type == "Sprite" ||
        inst.resource->type == "Font" )
    {
        bank->initTextureWithPath((Texture2D*)inst.resource, inst.path.c_str());
    }
    else if( inst.resource->type == "Sound" )
    {
        bank->initSoundWithPath((Sound*)inst.resource, inst.path.c_str());
    }
    else
    {   // Assume if it's anything else it's a generic serializable.
        bank->initSerializableWithPath(inst.resource, inst.path.c_str());
    }
    q.pop();
    
    return q.empty();
}

int AsynchronousBank::percent() const
{
    int size = q.size();
    if(size > biggestSize)
        biggestSize = size;
    return (int)(100 * (1.0 - (1.0 * size / biggestSize)));
}

} // end namespace



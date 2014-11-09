
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


#include "macbank.h"
#include "util.h"

#include "log.h"

using namespace std;

namespace g2c {

void MacBank::initPersistentSerializableWithKey(Serializable* s, const char* key)
{
    initSerializableWithPath(s, (string(key) + ".txt").c_str());
}

void MacBank::writePersistentSerializableWithKey(const Serializable* s, const char* key)
{
    writeSerializableToPath(s, (string(key) + ".txt").c_str());
}

void MacBank::initDataWithPath(Data* data, const char* path)
{
    string fullpath = base_path + directory + path;
    
    FILE* fp = fopen(fullpath.c_str(), "r");
    if( !fp )
    {
        g2cerror( "File not found: %s\n", fullpath.c_str() );
        exit(0);
    }
    
    fseek(fp, 0, SEEK_END);
    int size = ftell(fp);
    fseek(fp, 0, SEEK_SET);
    
    data->resize(size+1);
    data->array()[size] = 0;
    fread(data->array(), 1, size, fp);
    fclose(fp);
}

void MacBank::initSerializableWithPath(Serializable* s, const char* path)
{
    Data data;
    initDataWithPath(&data, path);

    string old_directory = directory;
    directory += directoryOfPath(path);
    s->deserialize((char*)data.array());
    directory = old_directory;
}

void MacBank::writeSerializableToPath(const Serializable* s, const char* path)
{
    FILE* fp = fopen(path, "w");
    if( !fp )
    {
        g2cerror( "serializable file failed to open for writing: %s\n", path );
        exit(0);
    }
    string data = s->serialize();
    fwrite(data.c_str(), data.size(), 1, fp);
    fclose(fp);
}

void MacBank::initTextureWithCGImage(Texture2D* texture, CGImageRef image)
{
    int width = CGImageGetWidth(image);
    int height = CGImageGetHeight(image);

    if(((width-1)&width) != 0 || ((height-1)&height) != 0)
        g2clog( "WARNING: texture dimensions not a power of two: %s.\n",
                texture->name.c_str() );

    if(image)
    {
        GLubyte* data = (GLubyte*)calloc(width * height * 4, sizeof(GLubyte));
        CGContextRef context = CGBitmapContextCreate(data, width, height, 8, 4 * width,
                                                     CGImageGetColorSpace(image),
                                                     kCGImageAlphaPremultipliedLast);
        CGContextTranslateCTM(context, 0, height);
        CGContextScaleCTM(context, 1.0, -1.0);
        CGContextDrawImage(context,
                           CGRectMake(0.0, 0.0, (CGFloat)width, (CGFloat)height), image);
        CGContextRelease(context);
        
        texture->initWithImageData(data, width, height, 32);
        free(data);
    }
}

void MacBank::initBitmapWithCGImage(Bitmap* bitmap, CGImageRef image)
{
    int width = CGImageGetWidth(image);
    int height = CGImageGetHeight(image);
    
    if(image) {
        int size = 4 * width * height;
        GLubyte* data = (GLubyte*)calloc(size, sizeof(GLubyte));
        CGContextRef context = CGBitmapContextCreate(data, width, height, 8, 4 * width,
                                                     CGImageGetColorSpace(image),
                                                     kCGImageAlphaPremultipliedLast);
        CGContextTranslateCTM(context, 0, height);
        CGContextScaleCTM(context, 1.0, -1.0);
        CGContextDrawImage(context,
                           CGRectMake(0.0, 0.0,
                                      (CGFloat)width,
                                      (CGFloat)height),
                                      image);
        CGContextRelease(context);
        
        bitmap->set(data, width, height, 32);
        free(data);
    }
}

void MacFileSystemBank::initBitmapWithPath(Bitmap* bitmap, const char* path)
{
    string newpath = directory + path;
    
#if(!TARGET_OS_IPHONE)
    CFURLRef fileURL = CFURLCreateFromFileSystemRepresentation(
        kCFAllocatorDefault,
        (const UInt8*)(newpath.c_str()),
        newpath.size(),
        false);
    
    CGImageSourceRef image_source = CGImageSourceCreateWithURL(fileURL, NULL);
    if(!image_source || CGImageSourceGetCount(image_source) <= 0)
    {
        g2cerror( "Image source failed for %s\n", path );
        
        // default 64x64 white
        int w = 64, h = 64;
        int size = 4*w*h;
        unsigned char* data = new unsigned char[size];
        memset(data, 0xff, size);
        bitmap->set(data, w, h, 32);
        delete data;
        
        return;
    }
    
    CGImageRef image = CGImageSourceCreateImageAtIndex(image_source, 0, NULL);
    assert(image);
    
    initBitmapWithCGImage(bitmap, image);
#endif
}

void MacBank::initTextureWithPath(Texture2D* texture, const char* path)
{
    Bitmap bitmap;
    initBitmapWithPath(&bitmap, path);
    texture->initWithBitmap(bitmap);
}

void MacBank::initSoundWithPath(Sound* sound, const char* path)
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
    else
    {
        string fullpath = base_path + directory + path;
        
        ALenum  error = AL_NO_ERROR;
        
        CFURLRef fileURL = CFURLCreateFromFileSystemRepresentation(
           kCFAllocatorDefault,
           (const UInt8*)fullpath.c_str(),
           fullpath.size(),
           false);
        
        ALsizei size;
        ALenum format;
        ALsizei freq;
        
        void* data = getOpenALAudioData(fileURL, &size, &format, &freq);
        
        CFRelease(fileURL);
        
        if((error = alGetError()) != AL_NO_ERROR)
        {
            g2cerror("OpenAL error = 0x%x loading audio file: %s\n", error, fullpath.c_str());
            exit(0);
        }
        
        // Attach Audio Data to OpenAL Buffer
        alBufferData(sound->getIndex(), format, data, size, freq);
        
        free(data);
    }
}

void* MacBank::getOpenALAudioData(CFURLRef inFileURL,
                                  ALsizei* outDataSize,
                                  ALenum* outDataFormat,
                                  ALsizei* outSampleRate) const
{
    OSStatus err = noErr;
    SInt64 theFileLengthInFrames = 0;
    AudioStreamBasicDescription theFileFormat;
    UInt32 thePropertySize = sizeof(theFileFormat);
    ExtAudioFileRef extRef = NULL;
    void* theData = NULL;
    AudioStreamBasicDescription theOutputFormat;
    
    // Open a file with ExtAudioFileOpen()
    err = ExtAudioFileOpenURL(inFileURL, &extRef);
    if(err)
    {
        g2cerror("getOpenALAudioData: ExtAudioFileOpenURL FAILED, Error = %d\n", (int)err);
        exit(0);
    }
    
    // Get the audio data format
    err = ExtAudioFileGetProperty(extRef,
                                  kExtAudioFileProperty_FileDataFormat,
                                  &thePropertySize,
                                  &theFileFormat);
    if(err)
    {
        g2cerror("getOpenALAudioData: ExtAudioFileGetProperty"
                 "(kExtAudioFileProperty_FileDataFormat) FAILED, Error = %d\n", (int)err);
        exit(0);
    }
    if (theFileFormat.mChannelsPerFrame > 2)
    {
        g2cerror("getOpenALAudioData - Unsupported Format, channel count = %d.\n",
                 (int)(theFileFormat.mChannelsPerFrame));
        exit(0);
    }
    
    // Set the client format to 16 bit signed integer (native-endian) data
    // Maintain the channel count and sample rate of the original source format
    theOutputFormat.mSampleRate = theFileFormat.mSampleRate;
    theOutputFormat.mChannelsPerFrame = theFileFormat.mChannelsPerFrame;
    
    theOutputFormat.mFormatID = kAudioFormatLinearPCM;
    theOutputFormat.mBytesPerPacket = 2 * theOutputFormat.mChannelsPerFrame;
    theOutputFormat.mFramesPerPacket = 1;
    theOutputFormat.mBytesPerFrame = 2 * theOutputFormat.mChannelsPerFrame;
    theOutputFormat.mBitsPerChannel = 16;
    theOutputFormat.mFormatFlags = kAudioFormatFlagsNativeEndian |
        kAudioFormatFlagIsPacked | kAudioFormatFlagIsSignedInteger;
    
    // Set the desired client (output) data format.
    err = ExtAudioFileSetProperty(extRef,
                                  kExtAudioFileProperty_ClientDataFormat,
                                  sizeof(theOutputFormat),
                                  &theOutputFormat);
    if(err)
    {
        g2cerror("getOpenALAudioData: ExtAudioFileSetProperty"
               "(kExtAudioFileProperty_ClientDataFormat) FAILED, Error = %d\n", (int)err);
        exit(0);
    }
    
    // Get the total frame count
    thePropertySize = sizeof(theFileLengthInFrames);
    err = ExtAudioFileGetProperty(extRef,
                                  kExtAudioFileProperty_FileLengthFrames,
                                  &thePropertySize,
                                  &theFileLengthInFrames);
    if(err)
    {
        g2cerror("getOpenALAudioData: ExtAudioFileGetProperty"
              "(kExtAudioFileProperty_FileLengthFrames) FAILED, Error = %d\n", (int)err);
        exit(0);
    }
    
    // Read all the data into memory
    UInt32 theFramesToRead = (UInt32)theFileLengthInFrames;
    UInt32 dataSize = theFramesToRead * theOutputFormat.mBytesPerFrame;
    theData = malloc(dataSize);
    if (theData)
    {
        AudioBufferList theDataBuffer;
        theDataBuffer.mNumberBuffers = 1;
        theDataBuffer.mBuffers[0].mDataByteSize = dataSize;
        theDataBuffer.mBuffers[0].mNumberChannels = theOutputFormat.mChannelsPerFrame;
        theDataBuffer.mBuffers[0].mData = theData;
        
        // Read the data into an AudioBufferList
        err = ExtAudioFileRead(extRef, &theFramesToRead, &theDataBuffer);
        if(err == noErr)
        {
            // success
            *outDataSize = (ALsizei)dataSize;
            *outDataFormat = (theOutputFormat.mChannelsPerFrame > 1) ?
                AL_FORMAT_STEREO16 : AL_FORMAT_MONO16;
            *outSampleRate = (ALsizei)theOutputFormat.mSampleRate;
        }
        else 
        {
            // failure
            free (theData);
            theData = NULL; // make sure to return NULL
            g2cerror("getOpenALAudioData: ExtAudioFileRead FAILED, Error = %d\n", (int)err);
            exit(0);
        }    
    }
    
    // Dispose the ExtAudioFileRef, it is no longer needed
    if (extRef) ExtAudioFileDispose(extRef);
    return theData;
}

} // end namespace



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


#include "androidbank.h"
#include "util.h"

#include <android/bitmap.h>
#include <android/asset_manager.h>
#include <android/asset_manager_jni.h>

#include <assert.h>
#include <string>

using namespace std;


namespace g2c {

AndroidBank::AndroidBank()
    : jvm(NULL)
{
}

AndroidBank::~AndroidBank()
{
}

void AndroidBank::setEnv(JNIEnv* env)
{
    if( env->GetJavaVM(&jvm) )
    {
        g2cerror("Retrieving java virtual machine from env failed.");
        exit(0);
    }
}

void AndroidBank::initPersistentSerializableWithKey(Serializable* s, const char* key)
{
    assert(false);
}

void AndroidBank::writePersistentSerializableWithKey(const Serializable* s, const char* key)
{
    assert(false);
}

void AndroidBank::initDataWithPath(Data* data, const char* path)
{
    JNIEnv* env = getEnvForCurrentThread();

    // Get the Loader class.
    jclass class_StaticLoader = env->FindClass("com.complex.g2c.StaticLoader");
    
    // Get the c object representing the java method for getting a string.
    jmethodID method_getAssetManager = env->GetStaticMethodID(
        class_StaticLoader, "getAssetManager", "()Landroid/content/res/AssetManager;");
    
    jobject localAsssetManager = env->CallStaticObjectMethod(
        class_StaticLoader, method_getAssetManager);
    
    AAssetManager* manager = AAssetManager_fromJava(env, localAsssetManager);
    
    assert(NULL != manager);
    
    AAsset* asset = AAssetManager_open(manager, path, AASSET_MODE_UNKNOWN);
    assert(NULL != asset);
    
    void const* temp = AAsset_getBuffer(asset);
    size_t size = AAsset_getLength(asset);
    
    data->resize(size+1);
    data->array()[size] = 0;
    memcpy(data->array(), temp, size);
}

void AndroidBank::initSerializableWithPath(Serializable* s, const char* path)
{
    JNIEnv* env = getEnvForCurrentThread();

    string fullpath = base_path + directory + path;
    
    string old_directory = directory;
    directory += directoryOfPath(path);
    
    // Get the Loader class.
    jclass class_StaticLoader = env->FindClass("com.complex.g2c.StaticLoader");
    
    // Get the c object representing the java method for getting a string.
    jmethodID method_getStringFromPath = env->GetStaticMethodID(
        class_StaticLoader, "getStringFromPath", "(Ljava/lang/String;)Ljava/lang/String;");
    
    // Construct input to java function as a jstring.
    jstring java_path = env->NewStringUTF(fullpath.c_str());
    
    // Call loader java method to get text from file.
    jstring java_str = (jstring) env->CallStaticObjectMethod(
        class_StaticLoader, method_getStringFromPath, java_path);
    
    // Convert to c string.
    const char* native_str = env->GetStringUTFChars(java_str, NULL);
    
    // Deserialize from c string.
    s->deserialize(native_str);
    
    // Deallocate c string.
    env->ReleaseStringUTFChars(java_str, native_str);
    
    directory = old_directory;
}

void AndroidBank::writeSerializableToPath(const Serializable* s, const char* path)
{
    assert(false);
}

void AndroidBank::initBitmapWithPath(Bitmap* bitmap, const char* path)
{
    JNIEnv* env = getEnvForCurrentThread();

    string fullpath = base_path + directory + path;
    
    AndroidBitmapInfo info;
    void* pixels;
    int ret;
    
    // Get the Loader class.
    jclass class_StaticLoader = env->FindClass("com.complex.g2c.StaticLoader");
    
    // Get the c object representing the java method for getting a bitmap.
    jmethodID method_getBitmapFromPath = env->GetStaticMethodID(
        class_StaticLoader,
        "getBitmapFromPath",
        "(Ljava/lang/String;)Landroid/graphics/Bitmap;");
    
    // Construct input to java function as a jstring.
    jstring java_path = env->NewStringUTF(fullpath.c_str());
    
    jobject java_bitmap = env->CallStaticObjectMethod(
        class_StaticLoader, method_getBitmapFromPath, java_path);
    
    if( (ret = AndroidBitmap_getInfo(env, java_bitmap, &info)) < 0 )
    {
        g2cerror("AndroidBitmap_getInfo failed. error=%d", ret);
        return;
    }
    
    if( info.format != ANDROID_BITMAP_FORMAT_RGBA_8888 )
    {
        g2cerror("Bitmap format is not ANDROID_BITMAP_FORMAT_RGBA_8888 is %d !", info.format);
        return;
    }
    
    if( (ret = AndroidBitmap_lockPixels(env, java_bitmap, &pixels)) < 0 )
    {
        g2cerror("AndroidBitmap_lockPixels() failed ! error=%d", ret);
        return;
    }
    
    bitmap->set(
        (uint8_t*)pixels,
        info.width,
        info.height,
        32);
    
    if( (ret = AndroidBitmap_unlockPixels(env, java_bitmap)) < 0 )
    {
        g2cerror("AndroidBitmap_unlockPixels() failed ! error=%d", ret);
        return;
    }
    
    bitmap->flipVertically();
}

void AndroidBank::initTextureWithPath(Texture2D* texture, const char* path)
{
    Bitmap bitmap;
    initBitmapWithPath(&bitmap, path);
    texture->initWithBitmap(bitmap);
}

JNIEnv* AndroidBank::getEnvForCurrentThread()
{
    JNIEnv* env;

    if( jvm == NULL )
    {
        g2cerror("Java virtual machine uninitialized.\n");
        exit(0);
    }

    if( jvm->GetEnv(reinterpret_cast<void**>(&env), JNI_VERSION_1_4) != JNI_OK )
    {
        g2cerror("Retrieving java env object unsuccessful.\n");
        exit(0);
    }

    if( jvm->AttachCurrentThread(&env, 0) < 0 )
    {
        g2cerror("Java virtual machine failed to attach current thread.\n");
        exit(0);
    }

    return env;
}

} // end namespace


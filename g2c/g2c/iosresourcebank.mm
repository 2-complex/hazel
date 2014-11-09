
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



#include "iosresourcebank.h"

#include <string>

using namespace std;

namespace g2c {


void IOSResourceBank::initPersistentSerializableWithKey(Serializable* s, const char* key)
{
	NSUserDefaults* defaults = [NSUserDefaults standardUserDefaults];
	NSString* sString = [defaults objectForKey: [NSString stringWithUTF8String: key]];
	if( sString )
		s->deserialize(string([sString cStringUsingEncoding:1]));
}

void IOSResourceBank::writePersistentSerializableWithKey(const Serializable* s, const char* key)
{
	NSUserDefaults* defaults = [NSUserDefaults standardUserDefaults];
	[defaults setObject: [NSString stringWithUTF8String:s->serialize().c_str()]
				forKey: [NSString stringWithUTF8String:key]];
	[defaults synchronize];
}

void IOSResourceBank::initSerializableWithPath(Serializable* s, const char* path)
{
	NSString *nsbundlepath = [[NSBundle mainBundle] bundlePath];
	string old_base_path = base_path;
	base_path = string([nsbundlepath cStringUsingEncoding:1]) + string("/");
	MacBank::initSerializableWithPath(s, path);
	base_path = old_base_path;
}

void IOSResourceBank::writeSerializableToPath(const Serializable* s, const char* path)
{
	NSString *nsbundlepath = [[NSBundle mainBundle] bundlePath];
	string revisedpath = string([nsbundlepath cStringUsingEncoding:1]) + string("/") + path;
	MacBank::writeSerializableToPath(s, revisedpath.c_str());
}

void IOSResourceBank::initSoundWithPath(Sound* sound, const char* path)
{
	NSString *nsbundlepath = [[NSBundle mainBundle] bundlePath];
	string old_base_path = base_path;
	base_path = string([nsbundlepath cStringUsingEncoding:1]) + string("/");
	MacBank::initSoundWithPath(sound, path);
	base_path = old_base_path;
}

void IOSResourceBank::initBitmapWithPath(Bitmap* bitmap, const char* path)
{
	string revisedpath = directory + path;
	CGImageRef image = [UIImage imageNamed:
		[NSString stringWithUTF8String:revisedpath.c_str()]].CGImage;
	if( !image )
	{
		g2cerror( "image %s not found\n", revisedpath.c_str() );
		exit(0);
	}
	MacBank::initBitmapWithCGImage(bitmap, image);
}

} // end namespace



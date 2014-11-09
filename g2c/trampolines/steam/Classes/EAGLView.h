
#import <UIKit/UIKit.h>

#import "ESSetup.h"

#include "sprites.h"
#include "iosresourcebank.h"
#include "app.h"

#include <map>

class Insider
{
public:
	g2c::App* app;
	g2c::World* world;
	g2c::IOSResourceBank* bank;
	g2c::Renderer* renderer;
	std::map<UITouch*, int> touchIndexMap;
};

// This class wraps the CAEAGLLayer from CoreAnimation into a convenient UIView subclass.
// The view content is basically an EAGL surface you render your OpenGL scene into.
// Note that setting the view non-opaque will only work if the EAGL surface has an alpha channel.
@interface EAGLView : UIView
{
@private
	id <ESSetup> setup;
	
	NSUInteger mWidth;
	NSUInteger mHeight;
	
	BOOL firstReshape;
	BOOL _animating;
	NSInteger _animationFrameInterval;
	
	// Use of the CADisplayLink class is the preferred method for controlling animation timing.
	// CADisplayLink will link to the main display and fire every vsync when added to a given run-loop.
	// The NSTimer class is used only as fallback when running on a pre 3.1 device where CADisplayLink
	// isn't available.
	id displayLink;
    NSTimer *animationTimer;
    
	Insider* insider;
}

@property (readonly, nonatomic, getter=isAnimating) BOOL animating;
@property (nonatomic) NSInteger animationFrameInterval;

- (void)startAnimation;
- (void)stopAnimation;

- (void)resizeWithWidth:(NSUInteger)width height:(NSUInteger)height;

@end


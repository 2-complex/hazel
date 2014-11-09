
#import "EAGLView.h"
#import "ES1Setup.h"
#import "ES2Setup.h"

#include "iosresourcebank.h"
#include "sprites.h"

#include "app.h"

class TrampolineApp : public g2c::App {
    void draw() const {
        glClearColor(0.3f, 0.4f, 0.6f, 1.f);
        glClear(GL_COLOR_BUFFER_BIT);
    }
};

using namespace std;
using namespace g2c;

@implementation EAGLView

+ (Class) layerClass
{
    return [CAEAGLLayer class];
}

- (id) initWithCoder:(NSCoder*)coder
{
    if( self = [super initWithCoder:coder] )
	{
        // Get the layer
        CAEAGLLayer *eaglLayer = (CAEAGLLayer*)self.layer;
        
        eaglLayer.opaque = YES;
        eaglLayer.drawableProperties = @{ kEAGLDrawablePropertyRetainedBacking: @(NO),
                                          kEAGLDrawablePropertyColorFormat: kEAGLColorFormatRGBA8 };
		
		_animating = YES;
		_animationFrameInterval = 1;
		
		[self initWorld];
    }
	
    return self;
}

- (void) initWorld
{
	// For OpenGLES1/OpenGLES2, use 1 or 2 here.
	int OpenGLESVersion = 2;
	
	if( OpenGLESVersion == 1 )
		setup = [[ES1Setup alloc] init];
	else
		setup = [[ES2Setup alloc] init];
	
	Renderer* spriteRenderer = NULL;
	
	if( OpenGLESVersion==1 )
		spriteRenderer = new RendererGL1;
	else
		spriteRenderer = new RendererGL2;
	
	spriteRenderer->init();
	
	Sprite::renderer = spriteRenderer;
	
	[self setAnimationFrameInterval: 1];
	
	insider = new Insider;
	firstReshape = true;
	insider->bank = new IOSResourceBank;
    
	TrampolineApp* app = new TrampolineApp();
	app->setBank(insider->bank);
	insider->app = app;
	app->init();
}

- (void) drawView:(id)sender
{
    [setup beginDraw];
	
	if( firstReshape )
	{
		int width = self.window.frame.size.width;
		int height = self.window.frame.size.height;
		
		[self resizeWithWidth:width height:height];
		
		firstReshape = NO;
	}
	
	insider->app->step(currentTime());
	insider->app->draw();
	
	[setup endDraw];
}

- (void) layoutSubviews
{
	[setup resizeFromLayer:(CAEAGLLayer*)self.layer];
    [self drawView:nil];
}

- (void) setAnimationFrameInterval:(NSInteger)frameInterval
{
	// Frame interval defines how many display frames must pass between each time the
	// display link fires. The display link will only fire 30 times a second when the
	// frame internal is two on a display that refreshes 60 times a second. The default
	// frame interval setting of one will fire 60 times a second when the display refreshes
	// at 60 times a second. A frame interval setting of less than one results in undefined
	// behavior.
	if( frameInterval >= 1 )
	{
		_animationFrameInterval = frameInterval;
		
		if( _animating )
		{
			[self stopAnimation];
			[self startAnimation];
		}
	}
}

- (void) startAnimation
{
	if( !_animating )
	{
        displayLink = [CADisplayLink displayLinkWithTarget:self selector:@selector(drawView:)];
        [displayLink setFrameInterval:_animationFrameInterval];
        [displayLink addToRunLoop:[NSRunLoop currentRunLoop] forMode:NSDefaultRunLoopMode];
        
		_animating = YES;
	}
}

- (void) stopAnimation
{
	if( _animating )
	{
        [displayLink invalidate];
        displayLink = nil;
		_animating = NO;
	}
}

- (void)resizeWithWidth:(NSUInteger)width height:(NSUInteger)height
{
	mWidth = width;
	mHeight = height;
	
	insider->app->reshape(width, height);
}

- (void)touchesBegan:(NSSet*)touches withEvent:(UIEvent*)event
{
    unsigned int index = 0;
    set<unsigned int> filled;
    for( map<UITouch*, int>::iterator itr = insider->touchIndexMap.begin(); itr!=insider->touchIndexMap.end(); itr++ )
        filled.insert(itr->second);
    
    // Iterate through all touches
    for( UITouch *touch in [touches allObjects] ) {
        
        // Get the location of the touch on the screen
        CGPoint touchPoint = [touch locationInView:self];
        // Invert the y point before passing to g2c
        Vec2 c(touchPoint.x, mHeight - touchPoint.y);
        
        while (filled.find(index) != filled.end())
            index++;
        
        if( index == 0 )
            insider->app->mouseDown(c);
        
        insider->app->touchDown(index, c);
        insider->touchIndexMap[touch] = index;
        
        filled.insert(index);
        
        insider->app->mouseDown(c);
    }
}

- (void)touchesMoved:(NSSet*)touches withEvent:(UIEvent*)event
{
    // Iterate through all touches
    for( UITouch *touch in [touches allObjects] )
    {
        // Get the location of the touch on the screen
        CGPoint touchPoint = [touch locationInView:self];
        // Invert the y point before passing to g2c
        Vec2 c(touchPoint.x, mHeight - touchPoint.y);
        
        int index = insider->touchIndexMap[touch];
        if( index == 0 )
            insider->app->mouseDragged(c);
        
        insider->app->touchDragged(index, c);
    }
}

- (void)touchesEnded:(NSSet*)touches withEvent:(UIEvent*)event
{
    // Iterate through all touches
    for( UITouch *touch in [touches allObjects] )
    {
        
        // Get the location of the touch on the screen
        CGPoint touchPoint = [touch locationInView:self];
        // Invert the y point before passing to g2c
        Vec2 c(touchPoint.x, mHeight - touchPoint.y);
        
        int index = insider->touchIndexMap[touch];
        if( index == 0 )
            insider->app->mouseUp(c);
        
        insider->app->touchUp(index, c);
        insider->touchIndexMap.erase(touch);
    }
}

- (void)insertText:(NSString *)text
{
	char c = [text UTF8String][0];
	insider->app->keyboard(c);
}

- (void)deleteBackward
{
	insider->app->keyboard(0x7f);
}

- (BOOL)canBecomeFirstResponder
{
    return YES;
}

- (void) dealloc
{	
	delete insider->app;
	
	delete insider->bank;
	delete insider->renderer;
	
	delete insider;
}

@end
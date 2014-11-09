
#import "EAGLView.h"
#import "ES1Setup.h"
#import "ES2Setup.h"

#include "iosresourcebank.h"
#include "sprites.h"
#include "hazel.h"


using namespace std;
using namespace g2c;

@implementation EAGLView

@synthesize animating;
@dynamic animationFrameInterval;


+ (Class) layerClass
{
    return [CAEAGLLayer class];
}

- (void) initApp
{
    animating = TRUE;
    displayLinkSupported = FALSE;
    animationFrameInterval = 1;
    displayLink = nil;
    animationTimer = nil;
    
    resizeOn = false;
    
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

    insider->bank = new IOSResourceBank;
    Hazel* app = new Hazel;
    
    app->setBank(insider->bank);
    
    insider->app = app;
    
    app->init();
}

- (void) cueResizeWithWidth:(int)width height:(int)height
{
    resizeOn = true;
    resizeWidth = width;
    resizeHeight = height;
}

- (void) drawView:(id)sender
{
    [setup beginDraw];
    
    if( resizeOn )
    {
        [self resizeWithWidth: resizeWidth height: resizeHeight];
        resizeOn = false;
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

- (NSInteger) animationFrameInterval
{
    return animationFrameInterval;
}

- (void) setAnimationFrameInterval:(NSInteger)frameInterval
{
    // Frame interval defines how many display frames must pass between each time the
    // display link fires. The display link will only fire 30 times a second when the
    // frame internal is two on a display that refreshes 60 times a second. The default
    // frame interval setting of one will fire 60 times a second when the display refreshes
    // at 60 times a second. A frame interval setting of less than one results in undefined
    // behavior.
    if (frameInterval >= 1)
    {
        animationFrameInterval = frameInterval;
        
        if (animating)
        {
            [self stopAnimation];
            [self startAnimation];
        }
    }
}

- (void) startAnimation
{
    if( !animating )
    {
        if( displayLinkSupported )
        {
            // CADisplayLink is API new to iPhone SDK 3.1. Compiling against earlier versions will result in a warning, but can be dismissed
            // if the system version runtime check for CADisplayLink exists in -initWithCoder:. The runtime check ensures this code will
            // not be called in system versions earlier than 3.1.
            
            displayLink = [NSClassFromString(@"CADisplayLink") displayLinkWithTarget:self selector:@selector(drawView:)];
            [displayLink setFrameInterval:animationFrameInterval];
            [displayLink addToRunLoop:[NSRunLoop currentRunLoop] forMode:NSDefaultRunLoopMode];
        }
        else
        {
            animationTimer =
            [NSTimer scheduledTimerWithTimeInterval:
             (NSTimeInterval)((1.0 / 60.0) * animationFrameInterval)
                                             target: self
                                           selector: @selector(drawView:)
                                           userInfo: nil
                                            repeats: TRUE];
        }
        
        animating = TRUE;
    }
}

- (void) stopAnimation
{
    if( animating )
    {
        if( displayLinkSupported )
        {
            [displayLink invalidate];
            displayLink = nil;
        }
        else
        {
            [animationTimer invalidate];
            animationTimer = nil;
        }
        
        animating = FALSE;
    }
}

- (void)resizeWithWidth:(int)width height:(int)height
{
    mWidth = width;
    mHeight = height;

    insider->app->reshape(width, height);
}

-(void)touchesBegan:(NSSet*)touches withEvent:(UIEvent*)event
{
    CGPoint touchPoint = [[touches anyObject] locationInView:self];
    Vec2 c(touchPoint.x, mHeight-touchPoint.y);
    
    unsigned int index = 0;
    set<unsigned int> filled;
    for( map<UITouch*, int>::iterator itr = insider->touchIndexMap.begin(); itr!=insider->touchIndexMap.end(); itr++ )
        filled.insert(itr->second);
    
    NSEnumerator *enumerator = [touches objectEnumerator];
    
    UITouch* touch;
    while ( touch = [enumerator nextObject] )
    {
        while (filled.find(index) != filled.end())
            index++;
        
        CGPoint touchPoint = [touch locationInView:self];
        Vec2 c(touchPoint.x, mHeight-touchPoint.y);
        
        if( index == 0 )
            insider->app->mouseDown(c);
        
        insider->app->touchDown(index, c);
        insider->touchIndexMap[touch] = index;
        
        filled.insert(index);
    }
    
    insider->app->mouseDown(c);
}

-(void)touchesMoved:(NSSet*)touches withEvent:(UIEvent*)event
{
    CGPoint touchPoint = [[touches anyObject] locationInView:self];
    Vec2 c(touchPoint.x, mHeight-touchPoint.y);
    
    NSEnumerator *enumerator = [touches objectEnumerator];
    
    UITouch* touch;
    while ( touch = [enumerator nextObject] )
    {
        CGPoint touchPoint = [touch locationInView:self];
        Vec2 c(touchPoint.x, mHeight-touchPoint.y);
        
        int index = insider->touchIndexMap[touch];
        
        if( index == 0 )
            insider->app->mouseDragged(c);
        
        insider->app->touchDragged(index, c);
    }
    
    insider->app->mouseDragged(c);
}

-(void)touchesEnded:(NSSet*)touches withEvent:(UIEvent*)event
{
    CGPoint touchPoint = [[touches anyObject] locationInView:self];
    Vec2 c(touchPoint.x, mHeight-touchPoint.y);
    
    NSEnumerator *enumerator = [touches objectEnumerator];
    
    UITouch* touch;
    while ( touch = [enumerator nextObject] )
    {
        CGPoint touchPoint = [touch locationInView:self];
        Vec2 c(touchPoint.x, mHeight-touchPoint.y);
        
        int index = insider->touchIndexMap[touch];
        if( index == 0 )
            insider->app->mouseUp(c);
        
        insider->app->touchUp(index, c);
        insider->touchIndexMap.erase(touch);
    }
    
    insider->app->mouseUp(c);
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

- (BOOL)hasText
{
    return YES;
}

- (BOOL)canBecomeFirstResponder
{
    return YES;
}

- (BOOL)canResignFirstResponder
{
    return YES;
}


- (void) dealloc
{
    [setup release];
    
    delete insider->app;
    
    delete insider->bank;
    delete insider->renderer;
    
    delete insider;
    
    [super dealloc];
}


@end



#import <QuartzCore/QuartzCore.h>

#import <OpenGLES/EAGL.h>
#import <OpenGLES/EAGLDrawable.h>

@protocol ESSetup <NSObject>
@required
- (BOOL) resizeFromLayer:(CAEAGLLayer *)layer;
- (void) beginDraw;
- (void) endDraw;
@end


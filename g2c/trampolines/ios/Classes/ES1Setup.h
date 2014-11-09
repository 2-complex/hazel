
#import "ESSetup.h"

#import <OpenGLES/ES1/gl.h>
#import <OpenGLES/ES1/glext.h>

@interface ES1Setup : NSObject <ESSetup>
{
@private
	EAGLContext *context;
	
	GLint backingWidth;
	GLint backingHeight;
	GLuint defaultFramebuffer;
	GLuint colorRenderbuffer;
}

- (BOOL) resizeFromLayer:(CAEAGLLayer *)layer;
- (void) beginDraw;
- (void) endDraw;

@end


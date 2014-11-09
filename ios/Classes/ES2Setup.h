
#import "ESSetup.h"

#import <OpenGLES/ES2/gl.h>
#import <OpenGLES/ES2/glext.h>

@interface ES2Setup : NSObject <ESSetup>
{
@private
	EAGLContext *context;
	
	GLint backingWidth;
	GLint backingHeight;
	GLuint defaultFramebuffer;
	GLuint colorRenderbuffer;
	GLuint depthRenderbuffer;
}

- (BOOL) resizeFromLayer:(CAEAGLLayer *)layer;
- (void) beginDraw;
- (void) endDraw;

@end


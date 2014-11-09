
#import "EAGLViewController.h"

@implementation EAGLViewController

- (BOOL) prefersStatusBarHidden
{
    return YES;
}

- (BOOL) shouldAutorotateToInterfaceOrientation:(UIInterfaceOrientation)interfaceOrientation
{
	switch(interfaceOrientation)
	{
		case UIInterfaceOrientationPortrait: return YES;
		case UIInterfaceOrientationPortraitUpsideDown: return NO;
		case UIInterfaceOrientationLandscapeLeft: return NO;
		case UIInterfaceOrientationLandscapeRight: return NO;
		
		default:
			return NO;
	}
	
	return NO;
}

- (void) willRotateToInterfaceOrientation:(UIInterfaceOrientation)toInterfaceOrientation
								duration:(NSTimeInterval)duration
{
	int width = self.view.window.frame.size.width;
	int height = self.view.window.frame.size.height;
	
	if( UIInterfaceOrientationIsLandscape((toInterfaceOrientation)) )
	{
		int temp = width;
		width = height;
		height = temp;
	}
	
	[(EAGLView*)self.view resizeWithWidth: width height: height];
}

@end


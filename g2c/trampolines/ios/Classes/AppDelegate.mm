
#import "AppDelegate.h"
#import "EAGLView.h"
#import "EAGLViewController.h"

@implementation AppDelegate

- (void) applicationDidFinishLaunching:(UIApplication *)application
{
	// Forbid the screen from sleeping.
	application.idleTimerDisabled = YES;
    
	[self.glView startAnimation];
}

- (void) applicationWillResignActive:(UIApplication *)application
{
	[self.glView stopAnimation];
}

- (void) applicationDidBecomeActive:(UIApplication *)application
{
	[self.glView startAnimation];
}

- (void)applicationWillTerminate:(UIApplication *)application
{
	[self.glView stopAnimation];
}

@end


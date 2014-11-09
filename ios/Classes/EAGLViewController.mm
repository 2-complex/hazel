
#import "EAGLViewController.h"

@implementation EAGLViewController

- (BOOL)prefersStatusBarHidden
{
    return YES;
}

- (void)loadView
{
    EAGLView *eaglView = [[EAGLView alloc] initWithFrame:CGRectMake(0, 0, 0, 0)];
    [eaglView initApp];
    self.view = eaglView;
}

- (BOOL) shouldAutorotateToInterfaceOrientation:(UIInterfaceOrientation)interfaceOrientation
{
    switch(interfaceOrientation)
    {
        case UIInterfaceOrientationPortrait: return NO;
        case UIInterfaceOrientationPortraitUpsideDown: return NO;
        case UIInterfaceOrientationLandscapeLeft: return YES;
        case UIInterfaceOrientationLandscapeRight: return YES;

        default:
            return NO;
    }
    
    return NO;
}

- (BOOL)shouldAutorotate
{
    return YES;
}

-(void)showStatusAgain
{
    [[UIApplication sharedApplication] setStatusBarHidden:YES];
}

- (void) viewDidLoad
{
	CGRect screenRect = [[UIScreen mainScreen] bounds];
    CGFloat width = screenRect.size.width;
    CGFloat height = screenRect.size.height;
    UIInterfaceOrientation interfaceOrientation = [[UIApplication sharedApplication] statusBarOrientation];

    [self resizeViewWithWidth:width height:height orientation:interfaceOrientation];

    [[UIApplication sharedApplication] setStatusBarHidden:YES];
    [self performSelector:@selector(showStatusAgain) withObject:nil afterDelay:6.0];
    [super viewDidLoad];
}

- (void) willRotateToInterfaceOrientation:(UIInterfaceOrientation)toInterfaceOrientation
                                duration:(NSTimeInterval)duration
{
	CGRect screenRect = [[UIScreen mainScreen] bounds];
    CGFloat width = screenRect.size.width;
    CGFloat height = screenRect.size.height;

    [self resizeViewWithWidth:width height:height orientation:toInterfaceOrientation];
}

- (void) resizeViewWithWidth:(int)inWidth height:(int)inHeight orientation:(UIInterfaceOrientation)orientation
{
	int width = inWidth;
	int height = inHeight;

    if( orientation == UIInterfaceOrientationLandscapeLeft ||
        orientation == UIInterfaceOrientationLandscapeRight )
    {
        int temp = width;
        width = height;
        height = temp;
    }

    [(EAGLView*)(self.view) cueResizeWithWidth: width height: height];
}

@end


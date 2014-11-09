
#include "gluttrampoline.h"
#include "worldapp.h"

void launch(App* app)
{
    GlutTrampoline trampoline;

    trampoline.initWindow("World", 800, 600);

    trampoline.app = app;
    trampoline.mainLoop();
}

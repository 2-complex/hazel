
#include "appenvironment.h"
#include "hazel.h"
#include "macbank.h"
#include "openalplayer.h"

using namespace g2c;

int main(int argc, char** args)
{
	AppEnvironment E;
	Hazel app;
	
    E.initWindow("Hazel", 600, 800);
    
    MacFileSystemBank* bank = new MacFileSystemBank;
    OpenALPlayer* player = new OpenALPlayer;
    
	app.setBank(bank);
	app.setPlayer(player);
    
    E.app = &app;
    
    E.mainLoop();
    
    delete bank;
    
    return 0;
}


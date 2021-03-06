
#include "appenvironment.h"
#include "hazel.h"
#include "macbank.h"
#include "openalplayer.h"

using namespace g2c;

int main(int argc, char** args)
{
    AppEnvironment E;
    Hazel app;

    E.initWindow("Hazel", 800, 600);

    MacFileSystemBank* bank = new MacFileSystemBank;
    OpenALPlayer* player = new OpenALPlayer;

    app.setBank(bank);
    app.setAudioPlayer(player);

    E.app = &app;

    E.mainLoop();

    delete bank;

    return 0;
}



#include "app.h"
#include "launch.h"

using namespace g2c;

class TriangleApp : public App
{
protected:
    void draw() const;
};

void TriangleApp::draw() const
{
    glColor3f(0,0,1);
    glBegin(GL_POLYGON);
    glVertex3f(0,0,0);
    glVertex3f(1,0,0);
    glVertex3f(0,1,0);
    glEnd();
}

int main()
{
    TriangleApp app;
    launch(&app);
    return 0;
}

#ifndef ZOOM_H_
#define ZOOM_H_

int const zoomRec = 200;
int mousex, mousey;

Mat zoomIn(int x, int y);
Mat zoomOut(int x, int y);
static void onMouse( int event, int x, int y, int /*flags*/, void* /*param*/ );






#endif

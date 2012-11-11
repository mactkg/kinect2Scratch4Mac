
#include "testApp.h"
#include "ofMain.h"
#include "ofAppGlutWindow.h"

//========================================================================
int main(){
    ofAppGlutWindow window;
	ofSetupOpenGL(&window, defaultWidth, defaultHeight, OF_WINDOW);			// <-------- setup the GL context
    ofSetWindowShape(400, 300);
    
	// this kicks off the running of my app
	// can be OF_WINDOW or OF_FULLSCREEN
	// pass in width and height too:
	ofRunApp( new testApp());

}

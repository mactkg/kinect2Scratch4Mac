#ifndef _TEST_APP
#define _TEST_APP

//#define USE_IR // Uncomment this to use infra red instead of RGB cam...

#include "ofxOpenNI.h"
#include "ofxScratch.h"
#include "ofxUI.h"
#include "ofMain.h"

class testApp : public ofBaseApp{

public:
	void setup();
	void update();
	void draw();
    void exit();
    
	void keyPressed  (int key);
	void keyReleased(int key);
	void mouseMoved(int x, int y );
	void mouseDragged(int x, int y, int button);
	void mousePressed(int x, int y, int button);
	void mouseReleased(int x, int y, int button);
	void windowResized(int w, int h);

	void	setupRecording(string _filename = "");
    void    setupGui();

	ofxOpenNIContext	recordContext;
	ofxDepthGenerator	recordDepth;

#ifdef USE_IR
	ofxIRGenerator		recordImage;
#else
	ofxImageGenerator	recordImage;
#endif

	ofxUserGenerator	recordUser;

#if defined (TARGET_OSX) //|| defined(TARGET_LINUX) // only working on Mac/Linux at the moment (but on Linux you need to run as sudo...)
	ofxHardwareDriver	hardware;
#endif
    
    ofxScratch          scratch;
    void                sendPoints(XnPoint3D position, int joint);
	void				drawMasks();

	int					nearThreshold, farThreshold;
	int					pointCloudRotationY;

	ofImage				allUserMasks, user1Mask, user2Mask, depthRangeMask;

	float				filterFactor;

    ofxUICanvas *gui;   	
	void guiEvent(ofxUIEventArgs &e);
    bool connectScratch, connectKinect;
};

#endif

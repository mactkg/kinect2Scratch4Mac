#ifndef _TEST_APP
#define _TEST_APP

//#define USE_IR // Uncomment this to use infra red instead of RGB cam...

#include "ofxOpenNI.h"
#include "ofxScratch.h"
#include "ofxSimpleGuiToo.h"
#include "ofMain.h"

class testApp : public ofBaseApp{

public:
	void setup();
	void update();
	void draw();

	void keyPressed  (int key);
	void keyReleased(int key);
	void mouseMoved(int x, int y );
	void mouseDragged(int x, int y, int button);
	void mousePressed(int x, int y, int button);
	void mouseReleased(int x, int y, int button);
	void windowResized(int w, int h);

	void	setupKinect();

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
    void                updateKinect();

	int					nearThreshold, farThreshold;
	int					pointCloudRotationY;

	ofImage				allUserMasks, user1Mask, user2Mask, depthRangeMask;

	float				filterFactor;
    
    bool                 isKinect, isScratch, goKinect, goScratch, newVal;
};

#endif

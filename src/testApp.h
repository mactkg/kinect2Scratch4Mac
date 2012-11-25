#ifndef _TEST_APP
#define _TEST_APP

#include "ofxOpenNI.h"
#include "ofxScratch.h"
#include "ofxGui.h"
#include "ofMain.h"

const int defaultWidth = 800;
const int defaultHeight = 600;

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

	void setupKinect(bool & dummy);
    void setupScratch(bool & dummy);
    void updateKinect();
    void exit();

    //Kinect
	ofxOpenNI kinect;
    bool isKinect;
    
    //Scratch
    ofxScratch scratch;
    void sendPoints(ofPoint position, int joint, int n);
    bool isScratch;
    
    //GUI
    ofxPanel gui;
    ofxToggle connectScratch, connectKinect, oldValues;
    ofxIntSlider tilt_angle;
    bool isGui;
    float scale;
};

#endif

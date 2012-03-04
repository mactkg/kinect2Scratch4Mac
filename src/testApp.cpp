#include "testApp.h"

string jointNames[] = { "head", "neck", "l_shoulder", "l_elbow", "l_hand", 
                        "r_shoulder", "r_elbow", "r_hand", "torso", "l_hip",
                        "l_knee", "l_foot", "r_hip", "r_knee", "r_foot" };

//--------------------------------------------------------------
void testApp::setup() {
    
	nearThreshold = 500;
	farThreshold  = 1000;
    connectKinect = false;
    connectScratch = false;
    //setupGui();
    setupRecording();
	ofBackground(0, 0, 0);
        
}

void testApp::setupRecording(string _filename) {
#if defined (TARGET_OSX) //|| defined(TARGET_LINUX) // only working on Mac/Linux at the moment (but on Linux you need to run as sudo...)
	hardware.setup();				// libusb direct control of motor, LED and accelerometers
	hardware.setLedOption(LED_OFF); // turn off the led just for yacks (or for live installation/performances ;-)
#endif

	recordContext.setup();	// all nodes created by code -> NOT using the xml config file at all
	recordDepth.setup(&recordContext);

	recordUser.setup(&recordContext);
	recordUser.setSmoothing(0.1f);				// built in openni skeleton smoothing...
	recordUser.setUseMaskPixels(true);
	recordUser.setUseCloudPoints(false);
	recordUser.setMaxNumberOfUsers(1);					// use this to set dynamic max number of users (NB: that a hard upper limit is defined by MAX_NUMBER_USERS in ofxUserGenerator)

    recordContext.toggleRegisterViewport();
	recordContext.toggleMirror();
    connectKinect = true;
}

void testApp::setupGui(){
    gui = new ofxUICanvas(0,0,ofGetWidth(),ofGetHeight());
    gui->addWidgetDown(new ofxUILabelButton(connectKinect, "CONNECT TO KINECT", OFX_UI_FONT_MEDIUM));
    gui->addWidgetDown(new ofxUILabelButton(connectScratch, "CONNECT TO SCRATCH", OFX_UI_FONT_MEDIUM));
    ofAddListener(gui->newGUIEvent,this,&testApp::guiEvent);
}

//--------------------------------------------------------------
void testApp::update(){
    if(connectKinect){
    #ifdef TARGET_OSX // only working on Mac at the moment
        hardware.update();
    #endif

        // update all nodes
        recordContext.update();
        recordDepth.update();

        // demo getting depth pixels directly from depth gen
        depthRangeMask.setFromPixels(recordDepth.getDepthPixels(nearThreshold, farThreshold),
                                     recordDepth.getWidth(), recordDepth.getHeight(), OF_IMAGE_GRAYSCALE);

        // update tracking/recording nodes
        recordUser.update();

        // demo getting pixels from user gen
        
        allUserMasks.setFromPixels(recordUser.getUserPixels(), recordUser.getWidth(), recordUser.getHeight(), OF_IMAGE_GRAYSCALE);
        user1Mask.setFromPixels(recordUser.getUserPixels(1), recordUser.getWidth(), recordUser.getHeight(), OF_IMAGE_GRAYSCALE);
    }
    
    if(connectScratch) {
        ofxTrackedUser* user = recordUser.getTrackedUser(1);
        sendPoints(user->neck.position[0], 0);
        sendPoints(user->neck.position[1], 1);
        sendPoints(user->left_shoulder.position[1], 2);
        sendPoints(user->left_upper_arm.position[1], 3);
        sendPoints(user->left_lower_arm.position[1], 4);
        sendPoints(user->right_shoulder.position[1], 5);
        sendPoints(user->right_upper_arm.position[1], 6);
        sendPoints(user->right_lower_arm.position[1], 7);
        sendPoints(user->left_upper_torso.position[1], 8);
        sendPoints(user->left_lower_torso.position[1], 9);
        sendPoints(user->left_upper_leg.position[1], 10);
        sendPoints(user->left_lower_leg.position[1], 11);
        sendPoints(user->right_lower_torso.position[1], 12);
        sendPoints(user->right_upper_leg.position[1], 13);
        sendPoints(user->right_lower_leg.position[1], 14);
        scratch.update();
    }
}

//--------------------------------------------------------------
void testApp::draw(){

	ofSetColor(255, 255, 255);

    if(connectKinect){
        glPushMatrix();
        glScalef(0.50, 0.50, 0.50);
        
        recordDepth.draw(0,0,640,480);

        recordUser.draw();

        drawMasks();
        
        glPopMatrix();

        ofSetColor(255, 255, 0);

        string statusHardware;

    #ifdef TARGET_OSX // only working on Mac at the moment
        ofPoint statusAccelerometers = hardware.getAccelerometers();
        stringstream	statusHardwareStream;

        statusHardwareStream
        << " TILT: " << hardware.getTiltAngle();
        statusHardware = statusHardwareStream.str();
    #endif

        stringstream msg;

        msg
        << "FPS   : " << ofToString(ofGetFrameRate()) << "  " << statusHardware << endl;

        ofDrawBitmapString(msg.str(), 20, 500);
    }

}

void testApp::sendPoints(XnPoint3D position, int joint){
    int points[3];
        
    points[0] = -2 * (0.5 - (double)position.X / recordDepth.getWidth()) *240;
    points[1] = 2 * (0.5 - (double)position.Y / recordDepth.getHeight()) *180;
    points[2] = 2 * (0.5 - (double)position.Z / recordDepth.getMaxDepth()) *180;
    
    scratch.sensorUpdate(jointNames[joint] + "_x", ofToString(points[0]));
    scratch.sensorUpdate(jointNames[joint] + "_y", ofToString(points[1]));
    scratch.sensorUpdate(jointNames[joint] + "_z", ofToString(points[2]));
}

void testApp::drawMasks() {
	glPushMatrix();
	glEnable(GL_BLEND);
	glBlendFunc(GL_ONE_MINUS_DST_COLOR, GL_ZERO);
	allUserMasks.draw(640, 0, 640, 480);
	glDisable(GL_BLEND);
    glPopMatrix();

}

void testApp::exit()
{
    gui->saveSettings("GUI/guiSettings.xml");     
    delete gui; 
}

void testApp::guiEvent(ofxUIEventArgs &e)
{
    if(e.widget->getName() == "CONNECT TO SCRATCH"){
        ofxUIButton *button = (ofxUIButton *) e.widget;
        connectScratch = false;
        scratch.setup();
    }
}

//--------------------------------------------------------------
void testApp::keyPressed(int key){

	float smooth;

	switch (key) {
#ifdef TARGET_OSX // only working on Mac at the moment
		case 357: // up key
            if(hardware.tilt_angle <= 30){
                hardware.setTiltAngle(hardware.tilt_angle++);
                break;
            }
		case 359: // down key
            if (hardware.tilt_angle >= -30) {
                hardware.setTiltAngle(hardware.tilt_angle--);
                break;
            }
#endif
    }
}


//--------------------------------------------------------------
void testApp::keyReleased(int key){

}

//--------------------------------------------------------------
void testApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void testApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void testApp::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void testApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void testApp::windowResized(int w, int h){

}


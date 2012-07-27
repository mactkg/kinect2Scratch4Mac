#include "testApp.h"

string oldJointNames[] = { "head", "neck", "l_shoulder", "l_elbow", "l_hand", 
                        "r_shoulder", "r_elbow", "r_hand", "torso", "l_hip",
                        "l_knee", "l_foot", "r_hip", "r_knee", "r_foot" };
string newJointNames[] = { "Head", "ShoulderCenter", "ShoulderLeft", "ElbowLeft", "HandLeft",
                        "ShoulderRight", "ElbowRight", "HandRight", "Spine", "HipLeft",
                        "KneeLeft", "FootLeft", "HipRight", "KneeRight", "FootRight"};

//--------------------------------------------------------------
void testApp::setup() {
    
	nearThreshold = 500;
	farThreshold  = 1000;

	ofBackground(100, 100, 100);
    isKinect = false;
    isScratch = false;
    goKinect = false;
    goScratch = false;
    newVal = false;
    
    gui.addToggle("Kinect::Connect", goKinect);
    gui.addSlider("Kinect::Angle", hardware.tilt_angle, -30, 30);
    gui.addToggle("Scratch::Connect", goScratch);
    gui.addToggle("Send vals \nas K2S(1.5~)", newVal);

    gui.setup();
    gui.show();
    
}

void testApp::setupKinect() {

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

    isKinect = true;
}


//--------------------------------------------------------------
void testApp::update(){

    if (isKinect) {
        updateKinect();
    } else if(goKinect) {
        setupKinect();
    }
    
    if (isScratch) {
        scratch.update();
    } else if(goScratch) {
        scratch.setup();
        isScratch = true;
    }
    
}

//--------------------------------------------------------------
void testApp::draw(){
    
    gui.draw();
    ofRect(300, 112, 480, 360);
    
    if(isKinect){

        ofSetColor(255, 255, 255);

        glPushMatrix();
        glScalef(0.75, 0.75, 0.75);

        drawMasks();
        
        recordDepth.draw(400, 150, 640, 480);
        
        glTranslatef(400, 150, 0);
        
        recordUser.draw();
        
        glTranslatef(0, 0, 0);
        
        glPopMatrix();

        ofSetColor(255, 255, 0);

        string statusHardware = "";
    
    #ifdef TARGET_OSX // only working on Mac at the moment
        ofPoint statusAccelerometers = hardware.getAccelerometers();
        stringstream	statusHardwareStream;

        statusHardwareStream
        << "ACCELEROMETERS:"
        << " TILT: " << hardware.getTiltAngle() << "/" << hardware.tilt_angle
        << " x - " << statusAccelerometers.x
        << " y - " << statusAccelerometers.y
        << " z - " << statusAccelerometers.z;

        statusHardware = statusHardwareStream.str();
    #endif

        stringstream msg;

        msg
        << "FPS   : " << ofToString(ofGetFrameRate()) << "  " << statusHardware << endl;
            
        ofDrawBitmapString(msg.str(), 20, 500);
    }
}

//-----------------------------------------------------//
//    ----------------------------------------------   //
//    \                                            /   //
//     \                                          /    //
//      \________________________________________/     //
//                      /________\                     //
//-----------------------------------------------------//

void testApp::updateKinect(){
#ifdef TARGET_OSX // only working on Mac at the moment
	hardware.update();
    hardware.setTiltAngle(hardware.tilt_angle);
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
}

//--------------------------------------------------------------

void testApp::sendPoints(XnPoint3D position, int joint){
    if (isScratch) {
        int points[3];
            
        points[0] = -2 * (0.5 - (double)position.X / recordDepth.getWidth()) *240;
        points[1] = 2 * (0.5 - (double)position.Y / recordDepth.getHeight()) *180;
        points[2] = 2 * (0.5 - (double)position.Z / recordDepth.getMaxDepth()) *180;
        
        if (newVal) {
            scratch.sensorUpdate(newJointNames[joint] + "_x", ofToString(points[0]));
            scratch.sensorUpdate(newJointNames[joint] + "_y", ofToString(points[1]));
            scratch.sensorUpdate(newJointNames[joint] + "_z", ofToString(points[2]));
        } else {
            scratch.sensorUpdate(oldJointNames[joint] + "_x", ofToString(points[0]));
            scratch.sensorUpdate(oldJointNames[joint] + "_y", ofToString(points[1]));
            scratch.sensorUpdate(oldJointNames[joint] + "_z", ofToString(points[2]));
        }
    }
}

//--------------------------------------------------------------

void testApp::drawMasks() {
	glPushMatrix();
	glEnable(GL_BLEND);
	glBlendFunc(GL_ONE_MINUS_DST_COLOR, GL_ZERO);
	allUserMasks.draw(400, 150, 640, 480);
	glDisable(GL_BLEND);
    glPopMatrix();

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


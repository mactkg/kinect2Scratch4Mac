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

	ofBackground(100);
    isKinect = false;
    isScratch = false;
    isGui = true;
    scale = 1.0;
    
    ofSetWindowTitle("Kinect2Scratch4Mac - v005b1");
    
    gui.setup("Toggle gui panel:g");
    gui.add(connectKinect.setup("1:Kinect::Connect", false));
    gui.add(connectScratch.setup("2:Scratch::Connect", false));
    gui.add(tilt_angle.setup("Kinect::Motor", 0, -30, 30));
    gui.add(oldValues.setup("Old style value name", true));
    
    for (int i = 0; i < gui.getNumControls(); i++) {
        ofxBaseGui* part = gui.getControl(i);
        //write codes here
    }
    
    connectKinect.addListener(this, &testApp::setupKinect);
    connectScratch.addListener(this, &testApp::setupScratch);
}

//--------------------------------------------------------------
void testApp::update(){

    if (isKinect) {
        updateKinect();
    }    
    if (isScratch) {
        scratch.update();
    }
    
}

//--------------------------------------------------------------
void testApp::draw(){
    //draw the lower right corner
    ofSetColor(220);
    ofRect(ofGetWidth()-10, ofGetHeight()-10, 10, 10);
    
    int rx = 50;
    int ry = 50;

    glPushMatrix();
    if(scale < 1.0) {
        glScalef(scale, scale, 1.0f);
    }
    
    ofRect(rx, ry, 640, 480);
    
    ofSetColor(255);
    
    if(isKinect){

        ofSetColor(255, 255, 255);

        drawMasks(rx, ry);
        
        recordDepth.draw(rx, ry, 640, 480);
        
        glTranslatef(rx, ry, 0);
        recordUser.draw();
        glTranslatef(0, 0, 0);
        
        ofSetColor(255, 255, 0);
        
    }
    glScalef(1.0f, 1.0f, 1.0f);
    glPopMatrix();

    if(isGui){
        ofPushMatrix();
        gui.draw();
        ofPopMatrix();
    }
}

//--------------------------------------------------------------
void testApp::keyPressed(int key){

	switch (key) {
#ifdef TARGET_OSX
		case OF_KEY_UP:
            if(hardware.tilt_angle <= 30){
                hardware.setTiltAngle(tilt_angle++);
                break;
            }
		case OF_KEY_DOWN:
            if (hardware.tilt_angle >= -30) {
                hardware.setTiltAngle(tilt_angle--);
                break;
            }
#endif
        case 'g':
            isGui ? isGui = false : isGui = true;
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
    if(1.0*w/defaultWidth > 1.0*h/defaultHeight) {
        scale = 1.0*h/defaultHeight;
    } else {
        scale = 1.0*w/defaultWidth;
    }
}

//-----------------------------------------------------//
//    ----------------------------------------------   //
//    \                                            /   //
//     \                                          /    //
//      \________________________________________/     //
//                      /________\                     //
//-----------------------------------------------------//

void testApp::setupKinect(bool & dummy) {
    
#if defined (TARGET_OSX) //|| defined(TARGET_LINUX) // only working on Mac/Linux at the moment (but on Linux you need to run as sudo...)
	hardware.setup();				
	hardware.setLedOption(LED_OFF); 
#endif
    
    recordContext.setup();
    recordDepth.setup(&recordContext);
    
    recordUser.setup(&recordContext);
    recordUser.setSmoothing(0.1f);
    recordUser.setUseMaskPixels(true);
    recordUser.setUseCloudPoints(false);
    recordUser.setMaxNumberOfUsers(1);
    
    recordContext.toggleRegisterViewport();
    recordContext.toggleMirror();
    
    isKinect = true;
}

//--------------------------------------------------------------
void testApp::updateKinect(){
#ifdef TARGET_OSX // only working on Mac at the moment!!!
	hardware.update();
    hardware.setTiltAngle(tilt_angle);
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
    
    // send joints data to scratch
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
        
        if (oldValues) {
            scratch.sensorUpdate(oldJointNames[joint] + "_x", ofToString(points[0]));
            scratch.sensorUpdate(oldJointNames[joint] + "_y", ofToString(points[1]));
            scratch.sensorUpdate(oldJointNames[joint] + "_z", ofToString(points[2]));
        } else {
            scratch.sensorUpdate(newJointNames[joint] + "_x", ofToString(points[0]));
            scratch.sensorUpdate(newJointNames[joint] + "_y", ofToString(points[1]));
            scratch.sensorUpdate(newJointNames[joint] + "_z", ofToString(points[2]));
        }
    }
}

//--------------------------------------------------------------
void testApp::drawMasks(int x, int y) {
	glPushMatrix();
	glEnable(GL_BLEND);
	glBlendFunc(GL_ONE_MINUS_DST_COLOR, GL_ZERO);
	allUserMasks.draw(x, y, 640, 480);
	glDisable(GL_BLEND);
    glPopMatrix();
    
}

//--------------------------------------------------------------
void testApp::setupScratch(bool & dummy) {
    scratch.setup();
    isScratch = true;
}

//--------------------------------------------------------------
void testApp::exit(){
    if(isKinect){
#if defined (TARGET_OSX)
        hardware.shutDown();
#endif
        recordContext.shutdown();
    }
}
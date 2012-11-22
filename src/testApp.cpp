#include "testApp.h"

string oldJointNames[] = {"neck", "torso", "head", "l_shoulder", "l_elbow",
                        "l_hand", "r_shoulder", "r_elbow", "r_hand", "l_hip",
                        "l_knee", "l_foot", "r_hip", "r_knee", "r_foot" };
string newJointNames[] = {"Spine", "ShoulderCenter", "Head", "ShoulderLeft", "ElbowLeft",
                        "HandLeft", "ShoulderRight", "ElbowRight", "HandRight", "HipLeft",
                        "KneeLeft", "FootLeft", "HipRight", "KneeRight", "FootRight"};

//--------------------------------------------------------------
void testApp::setup() {
	nearThreshold = 500;
	farThreshold  = 1000;

	ofBackground(100);
    isKinect = false;
    isScratch = false;
    isGui = true;
    newVal = false;
    scale = 1.0;
    
    ofSetWindowTitle("Kinect2Scratch4Mac - v005b1");
    
    gui.setup("toggle gui panel:g");
    gui.add(connectKinect.setup("1:Kinect::Connect", false));
    gui.add(connectScratch.setup("2:Scratch::Connect", false));
    //gui.add(tilt_angle.setup("Kinect::Motor", 0, -30, 30));  yet
    
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
        kinect.update();
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
        
        kinect.drawDepth(rx, ry, 640, 480);
        kinect.drawSkeletons(rx, ry, 640, 480);
        
        //glTranslatef(rx, ry, 0);
        //Recorder.draw();
        //glTranslatef(0, 0, 0);
        
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
    // kinect.setup();
    kinect.setupFromXML(ofToDataPath("openni/config/modules.xml"));
    kinect.setRegister(true);
    kinect.setMirror(true);
    kinect.addDepthGenerator();
    kinect.addImageGenerator();
    kinect.addUserGenerator();
    kinect.setMaxNumUsers(2);
    kinect.start();
    
    
    
    isKinect = true;
}

//--------------------------------------------------------------
void testApp::updateKinect(){ //->sendLimbs?
    
    // send joints data to scratch
    for (int i = 0; i < kinect.getNumTrackedUsers(); i++) {
        ofxOpenNIUser &user = kinect.getTrackedUser(i);
        ofLogNotice() << user.getDebugInfo() << endl;
        for (int j = 0; j < user.getNumJoints(); j++) {
            ofxOpenNIJoint &joint = user.getJoint(Joint(j));
            ofLogNotice() << joint.getName() << endl;
            sendPoints(joint.getProjectivePosition(), j);
        }
    }
}

//--------------------------------------------------------------
void testApp::sendPoints(ofPoint position, int joint){
    if (isScratch) {
        int points[3];
        xn::DepthGenerator depthGen = kinect.getDepthGenerator();
        
        points[0] = -2 * (0.5 - (double)position.x / kinect.getWidth()) *240;
        points[1] = 2 * (0.5 - (double)position.y / kinect.getHeight()) *180;
        points[2] = 2 * (0.5 - (double)position.z / depthGen.GetDeviceMaxDepth()) *180;
        
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
        kinect.stop();
    }
}
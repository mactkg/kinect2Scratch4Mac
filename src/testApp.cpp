#include "testApp.h"

string oldJointNames[] = {"neck", "torso", "head", "l_shoulder", "l_elbow",
                        "l_hand", "r_shoulder", "r_elbow", "r_hand", "l_hip",
                        "l_knee", "l_foot", "r_hip", "r_knee", "r_foot" };
string newJointNames[] = {"Spine", "ShoulderCenter", "Head", "ShoulderLeft", "ElbowLeft",
                        "HandLeft", "ShoulderRight", "ElbowRight", "HandRight", "HipLeft",
                        "KneeLeft", "FootLeft", "HipRight", "KneeRight", "FootRight"};

//--------------------------------------------------------------
void testApp::setup() {
    //init
	ofBackground(100);
    isKinect = false;
    isScratch = false;
    isGui = true;
    scale = 1.0;
    
    ofSetWindowTitle("Kinect2Scratch4Mac - v005b3");
    
    //GUI
    gui.setup("Toggle gui panel:g");
    gui.add(connectKinect.setup("1:Kinect::Connect", false));
    gui.add(connectScratch.setup("2:Scratch::Connect", false));
    gui.add(tilt_angle.setup("Kinect::Motor", 0, -30, 30));
    gui.add(oldValues.setup("Old style value name", false));
    
    for (int i = 0; i < gui.getNumControls(); i++) {
        ofxBaseGui* part = gui.getControl(i);
        //write codes for setting here
    }
    
    //Event listener
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
        
        kinect.drawDepth(rx, ry, 640, 480);
        kinect.drawSkeletons(rx, ry, 640, 480);
        
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
        case OF_KEY_UP:
            tilt_angle++;
            break;
        case OF_KEY_DOWN:
            tilt_angle--;
            break;
        case 'g':
            isGui ? isGui = false : isGui = true;
            break;
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
    
    dev_kinect.setup();
    
    isKinect = true;
}

//--------------------------------------------------------------
void testApp::updateKinect(){
    //grab data
    kinect.update();
    dev_kinect.update();
    
    // send joints data to scratch
    for (int i = 0; i < kinect.getNumTrackedUsers(); i++) {
        ofxOpenNIUser &user = kinect.getTrackedUser(i);
        ofLogNotice() << user.getDebugInfo() << endl; //debugging
        for (int j = 0; j < user.getNumJoints(); j++) {
            ofxOpenNIJoint &joint = user.getJoint(Joint(j));
            ofLogNotice() << joint.getName() << "," << joint.getProjectivePosition().x; ; //debugging
            sendPoints(joint.getProjectivePosition(), j, i);
        }
    }
    
    dev_kinect.setTiltAngle(tilt_angle);
}

//--------------------------------------------------------------
void testApp::sendPoints(ofPoint position, int joint, int n){
    if (isScratch) {
        //numbering
        n++;
        string number = "";
        if(n != 1)
            number = ofToString(n);
        
        
        int points[3];
        xn::DepthGenerator depthGen = kinect.getDepthGenerator();
        
        //converting points for Scratch
        points[0] = -2 * (0.5 - (double)position.x / kinect.getWidth()) *240;
        points[1] = 2 * (0.5 - (double)position.y / kinect.getHeight()) *180;
        points[2] = 2 * (0.5 - (double)position.z / depthGen.GetDeviceMaxDepth()) *180;
        
        //Send values!
        if (oldValues) {
            scratch.sensorUpdate(oldJointNames[joint] + "_x" + number, ofToString(points[0]));
            scratch.sensorUpdate(oldJointNames[joint] + "_y" + number, ofToString(points[1]));
            scratch.sensorUpdate(oldJointNames[joint] + "_z" + number, ofToString(points[2]));
        } else {
            scratch.sensorUpdate(newJointNames[joint] + "_x" + number, ofToString(points[0]));
            scratch.sensorUpdate(newJointNames[joint] + "_y" + number, ofToString(points[1]));
            scratch.sensorUpdate(newJointNames[joint] + "_z" + number, ofToString(points[2]));
        }
    }
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
        dev_kinect.shutDown();
    }
}
 /*
 *  ofxScratch.cpp
 *  
 *
 *  Created by mactkg on 10/2/11.
 *  Copyright 2011 Tokyo Tech High School of Science and Technoligy. All rights reserved.
 *
 */

#include "ofxScratch.h"
#include "ofUtils.h"

void ofxScratch::setup() {
    std::cout << "---ofxScratch setup----" << endl;
	msgTxS = "sensor-update";
    msgTxB = "broadcast";
	weConnected = udpClient.Connect("127.0.0.1", 42001);
	if(weConnected) std::cout << "success connecting to scratch!" << endl;
	if(!weConnected) std::cout << "failed connecting to scratch...X(" << endl;
	connectTime = 0;
	deltaTime = 0;
}

void ofxScratch::update() {
    if(weConnected) {
        //sensor-send
        if (msgTxS != "sensor-update") {
            if (!sendMessage(msgTxS)) {
                std::cout << "connection is lost" << endl;
                weConnected = false;
            }
        }
        //broadcast-send
        if (msgTxB != "broadcast") {
            if (!sendMessage(msgTxB)) {
                std::cout << "connection is lost" << endl;
                weConnected = false;
            }
        }
        msgTxS = "sensor-update";
        msgTxB = "broadcast";
        
        msgRx = "";
        char pBuf[100];
        string str;
        int iSize;
        udpClient.Receive(pBuf, iSize);
        str.append(pBuf, iSize);
        if( str.length() > 0 ) {
            msgRx = str;
        }
    } else {
        deltaTime = ofGetElapsedTimeMillis() - connectTime;
        if( deltaTime > 5000) {
            weConnected = udpClient.Connect("127.0.0.1", 42001);
            connectTime = ofGetElapsedTimeMillis();
            std::cout << "lost connection, reconnecting... " << connectTime << endl;
        }
		if (weConnected) std::cout << "success reconnect!" << endl;
    }
    
}

void ofxScratch::sensorUpdate(string sensor, string val) {
	msgTxS += " \"" + sensor + "\" " + val;
}

void ofxScratch::broadcastUpdate(string val) {
	msgTxB += " \"" + val + "\"";
}

bool ofxScratch::sendMessage(string message){
	unsigned char sizeBytes[4];
	int len = message.size();

	sizeBytes[0] =(unsigned char)( len >> 24 );
	sizeBytes[1] =(unsigned char)( (len << 8) >> 24 );
	sizeBytes[2] =(unsigned char)( (len << 16) >> 24 );
	sizeBytes[3] =(unsigned char)( (len << 24) >> 24 );
    
    if(weConnected) {
        if(udpClient.Send((char*)sizeBytes, 4)) {
           
        } else {
           return false;
        }
        if(udpClient.Send(message.c_str(), message.size())) {
           return true;
        } else {
           return false;  
        }
    } else {
        return false;
    }
}

int ofxScratch::getConnectTime() {
  return connectTime;
}

int ofxScratch::getDeltaTime() {
  return deltaTime;
}

bool ofxScratch::getWeConnected() {
  return weConnected;
}


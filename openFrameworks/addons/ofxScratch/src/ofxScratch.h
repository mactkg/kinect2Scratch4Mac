/*
 *  ofxScratch.h
 *  
 *
 *  Created by mactkg on 10/2/11.
 *  Copyright 2011 Tokyo Tech High School of Science and Technoligy. All rights reserved.
 *
 */

#ifndef _OFXSCRATCH_H_
#define _OFXSCRATCH_H_

#include "ofxNetwork.h"

class ofxScratch {
public:
	void setup();
	void update();
    bool sendMessage(string message);
	void sensorUpdate(string sensor, string val);
	void broadcastUpdate(string val);
	int getConnectTime();
    int getDeltaTime();
    bool getWeConnected();

private:
    
	ofxUDPManager udpClient;
	string msgTxS, msgTxB, msgRx;

	float counter;
	int connectTime;
	int deltaTime;

	bool weConnected;
};
#endif

/** ofxKinect2ProjectorCalibration **/
/** work in progress, not even beta! **/
/** Kj1, www.hangaar.net **/

#pragma once

#include "ofMain.h"

class Chessboard {
private:	
	float projectorWidth;
	float projectorHeight;
	float chessboardSizeX;
	float chessboardSizeY;
	float scale; 
	ofColor color;

public:
	Chessboard() {
		projectorWidth = 1280;
		projectorHeight = 800;
		chessboardSizeX = 8;
		chessboardSizeY = 6;
		scale = 0.75; 
		color = ofColor(255);
	}

	void draw() {
		ofPushStyle();
		ofSetLineWidth(0.0f);
		ofSetColor(color);	
		ofFill();	
		ofRect(0,0,projectorWidth,projectorHeight);
		float xstep = projectorWidth / chessboardSizeX * scale;
		float ystep = projectorHeight / chessboardSizeY * scale;	
		float xInset = (projectorWidth - (projectorWidth * scale)) / 2.0f;	
		float yInset = (projectorHeight - (projectorHeight * scale)) / 2.0f;	
		for (int i=0; i<chessboardSizeX; ++i) {
			for (int j=0; j<chessboardSizeY; ++j) {			
				if (i%2 == j%2)
					ofSetColor(0);
				else
					ofSetColor(color);
				ofRect(xInset + xstep * i, yInset + ystep * j, xstep, ystep);
			}
		}	
		ofPopStyle();
	}
	
	void setPatternBlocks(int x, int y) {
		chessboardSizeX = x+1;
		chessboardSizeY = y+1;
	}
	
	void setProjectorResolution(int x, int y){
		projectorWidth = x;
		projectorHeight = y;
	}
	
	void setPatternSize(float x){
		scale = x;
	}
	
	void setBgColor(int x) {
		color = ofColor(x);
	}

	vector<ofPoint> getInternalPoints() {
		vector<ofPoint> out;	
		ofPoint step = ofPoint(2.0f / float(chessboardSizeX), 2.0f / float(chessboardSizeY));	
		ofPoint inset = ofPoint(-1.0f, 1.0f);	
		ofPoint corner, xyPix;
		for (int j=1; j<chessboardSizeY; ++j)
			for (int i=1; i<chessboardSizeX; ++i) {
				corner = inset + step * ofVec2f(i, -j);
				corner *= scale;
				xyPix.x = (corner.x + 1.0f) / 2.0f * projectorWidth;
				xyPix.y = (1.0f - corner.y) / 2.0f * projectorHeight; 	
				out.push_back(xyPix);
			}
		return out;
	}
};

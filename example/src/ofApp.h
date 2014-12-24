#pragma once

#include "ofMain.h"

//openCV for video processing
#include "ofxCv.h"
#include "ofxOpenCv.h"

//kinect backend
#include "ofxKinectCommonBridge.h"
#include "KinectProjectorCalibration.h"
#include "KinectProjectorOutput.h"

//UI
#include "ofxSecondWindow.h"
#include "ofxUI.h"



 enum Mode {
	 PREVIEW,
	 CALIBRATION,
	 TEST
 };


class ofApp : public ofBaseApp{

	public:
		void setup();
		void update();
		void draw();

		void keyPressed(int key);

private:
	
		//settings
		void						loadSettings();
		int							projectorWidth;
		int							projectorHeight;
		
		//mode 
		Mode						mode;
		
		//kinect & the wrapper
		ofxKinectCommonBridge		kinect;
		KinectProjectorCalibration	kinectProjectorCalibration;
		KinectProjectorOutput		kinectProjectorOutput;
		bool						kinectReady;

		//calibration
		ofxCvContourFinder			contourFinder;
		void						drawBorder(ofColor color, string info);
		bool						reposition;
		long						repositionTime;
		bool						calibrationLoaded;

		//output

		//gui
		void						setupGui();
	    ofxUICanvas					*gui;
		void						guiEvent(ofxUIEventArgs &e);   
		void						guiUpdateLabels();  
		
		
		string						SAVENAME;
		ofxCvGrayscaleImage			gray;
		ofxSecondWindow				secondWindow;
		int							offsetX, offsetY;
		ofPixels					dstColorPixels;
		ofxCvColorImage				calibeatedColor;
};

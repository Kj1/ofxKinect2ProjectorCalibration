/** ofxKinect2ProjectorCalibration **/
/** work in progress, not even beta! **/
/** Kj1, www.hangaar.net **/

#pragma once

#include "ofxCv.h"
#include "ofMain.h"
#include "ofxKinectCommonBridge.h"

using namespace ofxCv;
using namespace cv;
	
class KinectProjectorOutput {
	public:
		//setup
		KinectProjectorOutput();			
		void setup(ofxKinectCommonBridge* _kinect, int _projectorResolutionX, int _projectorResolutionY);
		
		//project functions
		vector<ofPolyline> projectFromDepthXY(const vector<ofPolyline> p);
		ofPolyline projectFromDepthXY(const ofPolyline p);
		
		vector<ofPoint>	projectFromWorldVector(vector<ofPoint> o) const;
		ofPoint			projectFromWorld(ofPoint o) const;
		ofPoint			projectFromDepthXY(const ofPoint o) const;
		vector<ofPoint>	projectFromDepthXYVector(vector<ofPoint> o) const;
		vector<ofPoint>	projectFromDepthXYVectorMirrored(vector<ofPoint> o) const;
		ofVec2f			project(const Point3f o) const;
		vector<ofVec2f> project(vector<Point3f> src, int i) const;
		vector<ofPoint> project(vector<ofPoint> src, int i) const;
		void			loadCalibratedView(); 
		void			unloadCalibratedView();

		//load
		bool	load(string path, bool absolute = false);
		bool	isCalibrationReady();

		//getters & setters		
		float	getReprojectionError() const;

		float getDepth(ofPoint x);

		//settings
		int					projectorResolutionX;
		int					projectorResolutionY;

		int offsetX;
		int offsetY;

	protected:
		//other
		ofxKinectCommonBridge* kinect;
		bool				isReady;

		//calibrationResults
		float				reprojError;
		Mat					cameraMatrix, distCoeffs;
		vector<Mat>			boardRotations, boardTranslations;	
		Intrinsics			intrinsics;
		ofMatrix4x4			projectionMatrix;		
		ofMatrix4x4			modelMatrix;		
};
	

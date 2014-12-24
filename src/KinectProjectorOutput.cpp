/** ofxKinect2ProjectorCalibration **/
/** work in progress, not even beta! **/
/** Kj1, www.hangaar.net **/

#include "KinectProjectorOutput.h"

using namespace ofxCv ;
using namespace cv;

KinectProjectorOutput::KinectProjectorOutput() {	
	projectorResolutionX = 1280;
	projectorResolutionY = 800;
	reprojError = -1;
	isReady = false;
	offsetX = 0;
	offsetY = 0;
}


void	KinectProjectorOutput::setup(ofxKinectCommonBridge* _kinect, int _projectorResolutionX, int _projectorResolutionY) {
	kinect = _kinect;
	projectorResolutionX = _projectorResolutionX;
	projectorResolutionY = _projectorResolutionY;
}

bool KinectProjectorOutput::isCalibrationReady() {
	return isReady;
}

vector<ofPolyline> KinectProjectorOutput::projectFromDepthXY(const vector<ofPolyline> p) {
	if (p.size() <= 0) return p;
	vector <cv::Point3f>  points;
	vector <int> sizes;

	//add to array
	for (int i = 0; i < p.size(); i++) {
		int added = 0;
		float avgDepth = 0;
		for (int j = 0; j < p[i].size(); j++) {
			ofPoint pt = kinect->mapDepthPointToWorldPoint(p[i][j]);
			points.push_back(toCv(pt));
			added++;
		}
		sizes.push_back(added);
	}

	//calibrate
	Mat mt = boardTranslations[0];
	Mat mr = boardRotations[0];	
	vector<Point2f> projected;		
	projectPoints(points, mr, mt, cameraMatrix, distCoeffs, projected);	

	//Generate output array
	vector <ofPolyline> out;
	int iterator = 0;
	for (int i = 0; i < sizes.size(); i++) {
		ofPolyline line;
		for (int j = 0; j < sizes[i]; j++) {
			ofPoint p = ofVec2f(offsetX,offsetY) + toOf(projected[iterator++]);
			p.x = projectorResolutionX - p.x;
			line.addVertex(p);
		}
		line.close();
		out.push_back(line);
	}


	return out;

}


ofPolyline KinectProjectorOutput::projectFromDepthXY(const ofPolyline p) {
	if (p.size() <= 0) return p;
	vector <cv::Point3f>  points;


	float avgDepth = 0;
	for (int j = 0; j < p.size(); j++) {
		ofPoint pt = kinect->mapDepthPointToWorldPoint(p[j]);
		points.push_back(toCv(pt));
	}

	//calibrate
	Mat mt = boardTranslations[0];
	Mat mr = boardRotations[0];	
	vector<Point2f> projected;		
	projectPoints(points, mr, mt, cameraMatrix, distCoeffs, projected);	

	//Generate output array
	ofPolyline out;
	int iterator = 0;
	for (int j = 0; j < projected.size(); j++) {
		ofPoint p = ofVec2f(offsetX,offsetY) + toOf(projected[j]);
		p.x = projectorResolutionX - p.x;
		out.addVertex(p);
	}
	out.close();

	return out;

}


ofPoint KinectProjectorOutput::projectFromDepthXY(const ofPoint o) const {
	if (!isReady) return ofPoint(0,0);
	ofPoint ptWorld = kinect->mapDepthPointToWorldPoint(o);
	vector<cv::Point3f> vo;
	vo.push_back(cv::Point3f(ptWorld.x, ptWorld.y, ptWorld.z));	
	Mat mt = boardTranslations[0];
	Mat mr = boardRotations[0];	
	vector<Point2f> projected;		
	projectPoints(Mat(vo), mr, mt, cameraMatrix, distCoeffs, projected);	
	ofPoint p = ofVec2f(offsetX,offsetY) + toOf(projected[0]);
	p.x = projectorResolutionX - p.x;
	return p;
}


vector<ofPoint>	KinectProjectorOutput::projectFromWorldVector(vector<ofPoint> o) const {
	vector<ofPoint> out;
	if (o.size() < 1) return out;
	vector<cv::Point3f> tmp;
	if (!isReady) return out;
	for (int i = 0; i < o.size(); i++){
		tmp.push_back(cv::Point3f(o[i].x, o[i].y, o[i].z));	
	}
	Mat mt = boardTranslations[0];
	Mat mr = boardRotations[0];	
	vector<Point2f> projected;			
	projectPoints(tmp, mr, mt, cameraMatrix, distCoeffs, projected);	
	for (int i = 0; i < projected.size(); i++){
		out.push_back(ofVec2f(offsetX,offsetY) + ofPoint(projectorResolutionX-projected[i].x, projected[i].y));	
	}
	return out;
}

ofPoint	KinectProjectorOutput::projectFromWorld(ofPoint o) const {
	vector<ofPoint> out;
	vector<cv::Point3f> tmp;
	if (!isReady) return ofPoint(0,0);
	tmp.push_back(cv::Point3f(o.x, o.y, o.z));	
	Mat mt = boardTranslations[0];
	Mat mr = boardRotations[0];	
	vector<Point2f> projected;			
	projectPoints(tmp, mr, mt, cameraMatrix, distCoeffs, projected);	
	for (int i = 0; i < projected.size(); i++){
		out.push_back(ofVec2f(offsetX,offsetY) + ofPoint(projectorResolutionX-projected[i].x, projected[i].y));	
	}
	return out[0];
}

vector<ofPoint>	KinectProjectorOutput::projectFromDepthXYVector(vector<ofPoint> o) const {
	vector<ofPoint> out;
	if (o.size() < 1) return out;
	vector<cv::Point3f> tmp;
	vector<ofPoint> unMirrored;
	if (!isReady) return out;
	float minY = 99;
	for (int i = 0; i < o.size(); i++){
		ofPoint ptWorld = kinect->mapDepthPointToWorldPoint(o[i]);
		if (ptWorld.y < minY) minY = ptWorld.y;
		unMirrored.push_back(ptWorld);
	}
	for (int i = 0; i < o.size(); i++) {
		ofPoint ptWorld = unMirrored[i]; 
		//float y = ptWorld.y - minY;  //y zou nu positief moeten zijn vanaf 0 
		//y *= -1; //spiegel
		//y *= 0.5; //halvreing
		//y += minY; 

		tmp.push_back(cv::Point3f(ptWorld.x,ptWorld.y, ptWorld.z));	
	}
	Mat mt = boardTranslations[0];
	Mat mr = boardRotations[0];	
	vector<Point2f> projected;			
	projectPoints(tmp, mr, mt, cameraMatrix, distCoeffs, projected);	
	for (int i = 0; i < projected.size(); i++){
		out.push_back(ofVec2f(offsetX,offsetY) + ofPoint(projectorResolutionX-projected[i].x, projected[i].y));	
	}
	return out;
}

//mirrored
vector<ofPoint>	KinectProjectorOutput::projectFromDepthXYVectorMirrored(vector<ofPoint> o) const {
	vector<ofPoint> out;
	if (o.size() < 1) return out;
	vector<cv::Point3f> tmp;
	vector<ofPoint> unMirrored;
	if (!isReady) return out;
	float minY = 99;
	ofPolyline polLine;

	for (int i = 0; i < o.size(); i++){
		ofPoint ptWorld = kinect->mapDepthPointToWorldPoint(o[i]);
		if (ptWorld.y < minY) minY = ptWorld.y;
		unMirrored.push_back(ptWorld);
		polLine.addVertex(ptWorld);
	}
	polLine.close();

	int degree = (ofGetFrameNum() % 3600) / 10;
	ofPoint rotationAxisXYZ (0,0,1);
	ofPoint pivot = polLine.getCentroid2D();
	pivot.z = 0;
	pivot.y = minY;

	for (int i = 0; i < o.size(); i++) {
		ofPoint ptWorld = unMirrored[i]; 
		
		//ofPoint res = ptWorld.rotate(degree,pivot, rotationAxisXYZ);
		
		float y = ptWorld.y - minY;  //y zou nu positief moeten zijn vanaf 0 
		y *= -1; //spiegel
		y *= 0.5; //halvreing
		y += minY; 
		

		tmp.push_back(cv::Point3f(ptWorld.x,y, ptWorld.z));	
	}
	Mat mt = boardTranslations[0];
	Mat mr = boardRotations[0];	
	vector<Point2f> projected;			
	projectPoints(tmp, mr, mt, cameraMatrix, distCoeffs, projected);	
	for (int i = 0; i < projected.size(); i++){
		out.push_back(ofVec2f(offsetX,offsetY) + ofPoint(projectorResolutionX-projected[i].x, projected[i].y));	
	}
	return out;
}



ofVec2f KinectProjectorOutput::project(const Point3f o) const {
	if (!isReady) return ofVec2f(0,0);
	vector<cv::Point3f> vo(1, o);	
	Mat mt = boardTranslations[0];
	Mat mr = boardRotations[0];	
	vector<Point2f> projected;		
	projectPoints(Mat(vo), mr, mt, cameraMatrix, distCoeffs, projected);	
	ofPoint p = ofVec2f(offsetX,offsetY) + toOf(projected[0]);
	p.x = projectorResolutionX - p.x;
	return p;
}

vector<ofVec2f> KinectProjectorOutput::project(vector<Point3f> wrldSrc, int i) const {
	if (!isReady ||wrldSrc.size() <= 1)  { vector<ofVec2f> v; v.push_back(ofVec2f(0,0)); return v; }
	Mat mt = boardTranslations[0];
	Mat mr = boardRotations[0];	
	vector<Point2f> projected;		
	projectPoints(wrldSrc, mr, mt, cameraMatrix, distCoeffs, projected);	
	vector<ofVec2f> projectedOF;
	for (int i = 0; i < projected.size(); i++) {
		ofVec2f p = ofVec2f(offsetX,offsetY) + toOf(projected[i]);
		p.x = projectorResolutionX - p.x;
		projectedOF.push_back(p);
	}
	return projectedOF;
}

vector<ofPoint> KinectProjectorOutput::project(vector<ofPoint> wrldSrc, int i) const {
	if (!isReady)  { vector<ofPoint> v; v.push_back(ofPoint(0,0)); return v; }
	Mat mt = boardTranslations[0];
	Mat mr = boardRotations[0];	
	vector<Point2f> projected;		
	vector<Point3f> src = toCv(wrldSrc);		
	projectPoints(src, mr, mt, cameraMatrix, distCoeffs, projected);	
	vector<ofPoint> projectedOF;
	for (int i = 0; i < projected.size(); i++) {
		ofPoint p = ofVec2f(offsetX,offsetY) + toOf(projected[i]);
		p.x = projectorResolutionX - p.x;
		projectedOF.push_back(p);
	}
	return projectedOF;
}
float KinectProjectorOutput::getDepth(ofPoint x) {
	if (x.x < 0  || x.x > 512 || x.y < 0 || x.y > 424) return 0;
		ofPoint ptWorld = kinect->mapDepthPointToWorldPoint(x);
		return ptWorld.z;
}

void	KinectProjectorOutput::loadCalibratedView(){
	if (!isReady) return;
	glPushMatrix();
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glMatrixMode(GL_MODELVIEW);

	intrinsics.loadProjectionMatrix(0.001, 2000);
	applyMatrix(modelMatrix);
}

void	KinectProjectorOutput::unloadCalibratedView(){
	if (!isReady) return;
	
	glPopMatrix();
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
}

float KinectProjectorOutput::getReprojectionError() const {
	if (!isReady) return 0.0;
	return reprojError;
}

bool KinectProjectorOutput::load(string path, bool absolute) {
	FileStorage fs(ofToDataPath(path, absolute), FileStorage::READ);
	if (fs.isOpened()) {
		cv::Mat	rvec, tvec;
		fs["intrisics"] >> cameraMatrix;
		fs["projResX"] >> projectorResolutionX;
		fs["projResY"] >> projectorResolutionY;
		fs["rotation"] >> rvec;
		fs["translation"] >> tvec;
		fs["reprojectionError"] >>  reprojError;
		fs["offsetX"] >> offsetX;
		fs["offsetY"] >> offsetY;
		intrinsics.setup(cameraMatrix, Size2i(projectorResolutionX, projectorResolutionY));
		modelMatrix = makeMatrix(rvec, tvec);    
		boardRotations.push_back(rvec);
		boardTranslations.push_back(tvec);
		distCoeffs = Mat::zeros(8, 1, CV_64F);

		isReady = true;
		return true;
	}
	return false;
}
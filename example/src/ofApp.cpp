#include "ofApp.h"

using namespace ofxCv ;
using namespace cv;


void ofApp::setup() {	
	SAVENAME = "calibration.yml";

	kinect.initSensor();
	kinect.initColorStream();
	kinect.initDepthStream();
	kinect.initBodyIndexStream();
	//kinect.initSkeletonStream();
	//kinect.initIRStream();
	kinect.initCalibratedStream();
	kinect.initWorldStream();
	kinect.setDepthClipping(500,8000);
	kinect.start();

	
	ofSetLogLevel(OF_LOG_NOTICE);  
	offsetX = 0;
	offsetY = 0;

	
	loadSettings();
	ofSetFrameRate(30);
	ofSetWindowTitle("KinectV2 Projector Calibration");	
	mode				 = PREVIEW;
	
	kinectReady = true;
	reposition = false;
	repositionTime = ofGetElapsedTimeMillis();
	
	kinectProjectorCalibration.setup(&kinect,projectorWidth,projectorHeight, SAVENAME);
	kinectProjectorOutput.setup(&kinect,projectorWidth,projectorHeight);

	//gui
	setupGui();

	//setup our second window, make sure you immediatly select the correct position
	secondWindow.setup("blah",ofGetScreenWidth(),0,projectorWidth,projectorHeight,true);
}


void ofApp::update() {	
	if (!kinectReady) {
		return;
	}
	long now = ofGetElapsedTimeMillis();
	kinect.update();
	cout << "update: " << (ofGetElapsedTimeMillis() - now) << endl;
	
	if (mode == PREVIEW) {
		secondWindow.begin();
			ofClear(0);
			kinectProjectorCalibration.drawChessboard();
			drawBorder(ofColor(0), "PREVIEW");
		secondWindow.end();
	}
	if (mode == CALIBRATION) {
		//do a very-fast check if chessboard is found
		string info = "[Captures: " + ofToString(kinectProjectorCalibration.getDatabaseSize()) + "; repr: " + ofToString(kinectProjectorCalibration.getReprojectionError(), 2) + "]";

		//first , check if we need to process the frame.  Only if we do
		//not need to reposition
		float timeSinceStable = -1;		
		float timeToCapture =  0;  //ofClamp((5000.0 - timeSinceStable)/5000.0, 0, 1);
		bool foundNoChessboard = false ; //(timeSinceStable == -1);
		bool isStableBoard     = false; //timeToCapture > 0.95;
		
		bool doneCalibration   = kinectProjectorCalibration.getDatabaseSize() > 10;
		
		
		if (reposition && (ofGetElapsedTimeMillis() - repositionTime) > 2500) {
			//fresh capture
			reposition = false;
		} else if (reposition) {
			kinectProjectorCalibration.resetTimer();
		} else {
			//fastcheck
			timeSinceStable = kinectProjectorCalibration.doFastCheck();
			timeToCapture =  ofMap((2500.0 - timeSinceStable)/2500.0, 0, 1, 1, 0, true);
			if (timeSinceStable == -1) timeToCapture = 0;
			foundNoChessboard = (timeSinceStable == -1);
			isStableBoard     = timeToCapture > 0.95;
		}

		
		if (isStableBoard) {			
			bool b = kinectProjectorCalibration.addCurrentFrame();	
			if (b) {
				reposition = true;
				repositionTime = ofGetElapsedTimeMillis();
			}
		}


		secondWindow.begin();
			ofClear(0);
			kinectProjectorCalibration.drawChessboard();
			
			//if we need to reposition
			if (reposition) {
				drawBorder(ofColor(0,0,255), "REPOSITION. " + info);
			}	
			
			else if (foundNoChessboard && !doneCalibration) {
				drawBorder(ofColor(20,0,0), "SEARCHING CHESSBOARD. " + info);
			}
			else if (foundNoChessboard && doneCalibration) {
				drawBorder(ofColor(0,0,0), "DONE - SEARCHING CHESSBOARD. " + info);
			}			
			else if (!foundNoChessboard) {
				drawBorder(ofColor(20,ofMap(timeToCapture,0,1,0,70,true),0), "FOUND CHESSBOARD - HOLD STILL. " + info);
			}
			else if (isStableBoard) {
				drawBorder(ofColor(0,70,0), "CAPTURING CHESSBOARD. " + info);
			}		

		secondWindow.end();
	}
	
	if (mode == TEST) {
		
		if (!calibrationLoaded) {
			secondWindow.begin();
				ofClear(0);
				ofSetColor(255);
				ofDrawBitmapString("Unable to load calibration file. Did you calibrate?",0,20);
			secondWindow.end(); 
		} else {
			string info = "[Captures: " + ofToString(kinectProjectorCalibration.getDatabaseSize()) + "; repr: " + ofToString(kinectProjectorCalibration.getReprojectionError(), 2) + "]";
		
			gray.setFromPixels(kinect.getBodyIndexPixelsRef());
			gray.invert();
			contourFinder.findContours(gray, 1000, 480*400, 1, false, false);
	
			secondWindow.begin();
				ofClear(0);
				drawBorder(ofColor(50,0,0), "TEST MODE. " + info);
				ofSetColor(255);
				ofSetLineWidth(3);
				//contourFinder.draw();
				vector<ofPolyline> vects;
				for (int i = 0; i < contourFinder.nBlobs; i++) {
					vector<ofPoint> points;
					for (int j = 0; j < contourFinder.blobs[i].nPts ; j++) {
						points.push_back(contourFinder.blobs[i].pts[j]);
					}
					//we project from our depth xy to projector space
					vector<ofPoint> projected = kinectProjectorOutput.projectFromDepthXYVector(points);
					
					ofPolyline p;
					for (int j = 0; j < projected.size() ; j++) {
						if (projected[j].x > -1000 && projected[j].x < 2280 && 
							projected[j].y > -1000 && projected[j].y < 1800)
							p.addVertex(projected[j]);
					}
					p.close();

					//p.draw();			
					ofSetColor(255,255,255);
					p = p.getSmoothed(3);
					p.simplify();
					p = p.getSmoothed(3);
					ofSetLineWidth(5);
					p.draw();		
					ofSetLineWidth(3);
					
					ofSetColor(255);
				}		
			secondWindow.end();
		}
	}
	guiUpdateLabels();
}


void ofApp::draw() {	
	calibeatedColor.setFromPixels(kinect.getCalibratedColorPixelsRef());

	ofSetWindowTitle("FR: " + ofToString(ofGetFrameRate()));
	ofClear(0);
	ofSetColor(255);
	if (!kinectReady) {
		ofDrawBitmapString("Kinect not ready. Open kinect streamer prior to the configuration", 350,50);
		return;
	}
	ofTranslate(400,20);
	if (mode == PREVIEW) {
		//ofScale(.75,0.75);
		calibeatedColor.draw(0,0,512,424);
		ofDrawBitmapString("Kinect RGB Preview", 0,-5);
		
		kinect.getDepth().draw(0,530,512,424);
		ofDrawBitmapString("Kinect Depth Preview", 0,515);

		
		kinect.getBodyIndex().draw(650,0,512,424);
		ofDrawBitmapString("Kinect Label Preview", 650,-5);

		//ofScale(1.0/.75,1.0/0.75);
		if (ofGetMousePressed()) {
			int x = ofGetMouseX() - 400;
			int y = ofGetMouseY() - 20;
			ofVec3f p = kinect.mapDepthPointToWorldPoint(ofPoint(x,y));
			ofSetColor(255);
			ofDrawBitmapStringHighlight(ofToString(x) + ", " + ofToString (y) + " --> " + ofToString(p.x,2) + ", " + ofToString(p.y,2) + ", " + ofToString(p.z,2), x,y);
			ofSetColor(255);
			cout << ofToString(x) + ", " + ofToString (y) + " --> " + ofToString(p.x,2) + ", " + ofToString(p.y,2) + ", " + ofToString(p.z,2) << endl;
		}
	}

	if (mode == CALIBRATION) {
		ofScale(.5,0.5);
		
		calibeatedColor.draw(0,0,512,424);
		ofDrawBitmapString("Kinect Input",0,20);

		vector<ofVec2f> pts = kinectProjectorCalibration.getFastCheckResults();
		for (int i = 0; i < pts.size(); i++) {
			ofSetColor(0,255,0);
			ofFill();
			ofCircle(pts[i].x, pts[i].y, 5);
			ofNoFill();
		}
		ofSetColor(255);

		//draw our calibration gui
		kinectProjectorCalibration.drawChessboardDebug(640+20,0,512,424);
		ofDrawBitmapStringHighlight("Chessboard (2nd screen)",640+20,0);

		kinectProjectorCalibration.drawProcessedInputDebug(0,480+20, 512,424);
		ofDrawBitmapStringHighlight("Processed Input",0,480+20);

		kinectProjectorCalibration.drawReprojectedPointsDebug(640+20,480+40,512,424);
		ofDrawBitmapStringHighlight("Reprojected points. Green pts should overlap red.",640+20,480+40);
		ofScale(2,2);
	}

	if (mode == TEST) {
		//ofScale(.75,0.75);
		ofSetColor(255);
		if (!calibrationLoaded) {
			ofDrawBitmapString("Unable to load calibration file. Did you calibrate?",0,20);
		} else {
			ofDrawBitmapString("Calibration loaded.",0,20);
			ofDrawBitmapString("To test, stand between 1 and 4 meters from the kinect.",0,40);
			ofDrawBitmapString("There should be a line projected around you at any given distance.",0,60);
						
			calibeatedColor.draw(0,200,512,424);
			ofDrawBitmapString("Kinect RGB Preview", 0,90);
			
		//gray.draw(650,0,512,424);
		ofTranslate(650,0);
		contourFinder.draw();
		ofTranslate(-650,0);
		ofDrawBitmapString("Kinect Label Preview", 650,-5);

		
		ofDrawBitmapString("Kinect Label Preview", 650,-5);
			ofDrawBitmapString("MANUAL OFFSET CORRECTION", 0,120);
			ofDrawBitmapString("Press up/down/left/right or Z/S/Q/D", 0,140);
			ofDrawBitmapString("for manually correcting an offset", 0,160);
			ofDrawBitmapString("Current: X:" + ofToString(offsetX) + ", Y:" + ofToString(-offsetY), 0,180);
			
			int xP = ofGetMouseX() - 400;
			int yP = ofGetMouseY()- 20 - 200;

			if (xP > 0 && xP < 512 && yP >0 && yP < 424) {
				ofPoint sp = kinect.mapDepthPointToWorldPoint(ofPoint(xP, yP));
				ofDrawBitmapString("X: " + ofToString(sp.x,2) + "\nY: " + ofToString(sp.y,2) + "\nZ: " + ofToString(sp.z,2), xP  ,yP+20+200);
			}

		}
		//ofScale(1.0/.75,1.0/0.75);
	}

	ofTranslate(-400,-20);
	gui->draw();
}


void ofApp::drawBorder(ofColor b, string info) {
	ofSetColor(b);
	ofFill();
	ofRect(0,0,projectorWidth, 50);
	ofRect(0,projectorHeight-50,projectorWidth, 50);
	ofRect(0,0,50, projectorHeight);
	ofRect(projectorWidth-50,0,50, projectorHeight);
	int offset = info.length() * 8  + 20;
	ofDrawBitmapStringHighlight(info, 20,15, ofColor(0), ofColor(255));
	ofDrawBitmapStringHighlight(info, projectorWidth-offset,15, ofColor(0), ofColor(255));
	ofDrawBitmapStringHighlight(info, 20,projectorHeight-20, ofColor(0), ofColor(255));
	ofDrawBitmapStringHighlight(info, projectorWidth-offset,projectorHeight-20, ofColor(0), ofColor(255));
	ofSetColor(255);
	ofNoFill();
}
//--------------------------------------------------------------
void ofApp::keyPressed(int key){
	
	if (mode == TEST) {
		bool changed = false;
		if (key == OF_KEY_DOWN || key == 'S' || key == 's') {
			offsetY++;
			changed =  true;
		} else if (key == OF_KEY_UP || key == 'Z' || key == 'z') {
			offsetY--;
			changed =  true;
		} else if (key == OF_KEY_RIGHT || key == 'd' || key == 'D') {
			offsetX++;
			changed =  true;
		} else if (key == OF_KEY_LEFT || key == 'Q' || key == 'q') {
			offsetX--;
			changed =  true;
		}
		if (changed) {
			kinectProjectorCalibration.setOffsets(offsetX, offsetY);
			kinectProjectorCalibration.calibrate();
			calibrationLoaded = kinectProjectorOutput.load(SAVENAME);
			
		}
		
	}
}

void ofApp::loadSettings() {
	ofxXmlSettings		xmlReader;
	bool save = false;
	if(! xmlReader.loadFile("settings.xml") ){		
		ofLogError("Settings file settings.xml not found. Loading default values.");
		save = true;
	}
	projectorWidth	= xmlReader.getValue("Projector:Width", 1024);
	projectorHeight	= xmlReader.getValue("Projector:Height", 768);
	
	if (save)	
		xmlReader.saveFile();
}



void ofApp::setupGui() {

	float dim = 20; 
	float xInit = 2*OFX_UI_GLOBAL_WIDGET_SPACING; 
    float length = 350-xInit; 
	gui = new ofxUICanvas(0, 0, length+xInit, 1080); 
	gui->setTheme(OFX_UI_THEME_BLUEBLUE);		
	gui->setColorFill(ofColor(200));
	gui->setColorOutline(ofColor(200));
	gui->setColorPadded(ofColor(200));
	gui->setColorPaddedOutline(ofColor(255));
	gui->setColorOutlineHighlight(ofColor(255));
	gui->setColorFillHighlight(ofColor(255));	
	
	gui->addWidgetDown(new ofxUILabel("Calibration configuration instructions", OFX_UI_FONT_LARGE)); 
    gui->addSpacer(length-xInit, 2);
	gui->addWidgetDown(new ofxUILabel("sdf", "1) Configure projector in data/settings.xml", OFX_UI_FONT_SMALL));
	gui->addWidgetDown(new ofxUILabel("sdf", "2) Activate calibration", OFX_UI_FONT_SMALL));
	gui->addWidgetDown(new ofxUILabel("sdf", "3) Hold flat board so it contains chessboard", OFX_UI_FONT_SMALL));
	gui->addWidgetDown(new ofxUILabel("sdf", "   and can be seen by the kinect", OFX_UI_FONT_SMALL));
	gui->addWidgetDown(new ofxUILabel("sdf", "   Adjust size & brightness slider if needed", OFX_UI_FONT_SMALL));
	gui->addWidgetDown(new ofxUILabel("sdf", "   Optimal distance between 2 and 4 meters.", OFX_UI_FONT_SMALL));
	gui->addWidgetDown(new ofxUILabel("sdf", "4) Make +-15 captures, then clean the highest errors", OFX_UI_FONT_SMALL));
		
	gui->addWidgetDown(new ofxUILabel(" ", OFX_UI_FONT_LARGE)); 
	gui->addWidgetDown(new ofxUILabel("Configuration mode", OFX_UI_FONT_LARGE)); 
    gui->addSpacer(length-xInit, 2);
	//gui->addWidgetDown(new ofxUIToggle("Projection Window fullscreen",false, dim, dim));
	vector<string> modes;
	modes.push_back("Preview Kinect");
	modes.push_back("Calibrate");
	modes.push_back("Test");
	gui->addWidgetDown(new ofxUIRadio("Mode", modes, OFX_UI_ORIENTATION_VERTICAL,dim, dim, OFX_UI_FONT_SMALL));

	gui->addWidgetDown(new ofxUILabel(" ", OFX_UI_FONT_LARGE)); 
	gui->addWidgetDown(new ofxUILabel("Chessboard settings", OFX_UI_FONT_LARGE)); 
    gui->addSpacer(length-xInit, 2);
	gui->addWidgetDown(new ofxUIBiLabelSlider(length,0.0,100,&kinectProjectorCalibration.chessboardSize,"boardsize","Small","Large"));
	gui->addWidgetDown(new ofxUIBiLabelSlider(length,0.0,255.0,&kinectProjectorCalibration.chessboardColor,"boardColor","dark","light"));
	gui->addWidgetDown(new ofxUIToggle("CV_CALIB_CB_ADAPTIVE_THRESH",&kinectProjectorCalibration.b_CV_CALIB_CB_ADAPTIVE_THRESH, dim, dim));
	gui->addWidgetDown(new ofxUIToggle("CV_CALIB_CB_NORMALIZE_IMAGE",&kinectProjectorCalibration.b_CV_CALIB_CB_NORMALIZE_IMAGE, dim, dim));
	
	gui->addWidgetDown(new ofxUILabel(" ", OFX_UI_FONT_LARGE)); 
	gui->addWidgetDown(new ofxUILabel("Calibration settings", OFX_UI_FONT_LARGE)); 
    gui->addSpacer(length-xInit, 2);
	gui->addWidgetDown(new ofxUIToggle("CV_CALIB_FIX_PRINCIPAL_POINT",&kinectProjectorCalibration.b_CV_CALIB_FIX_PRINCIPAL_POINT, dim, dim));
	gui->addWidgetDown(new ofxUIToggle("CV_CALIB_FIX_ASPECT_RATIO",&kinectProjectorCalibration.b_CV_CALIB_FIX_ASPECT_RATIO, dim, dim));
	gui->addWidgetDown(new ofxUIToggle("CV_CALIB_ZERO_TANGENT_DIST",&kinectProjectorCalibration.b_CV_CALIB_ZERO_TANGENT_DIST, dim, dim));
	gui->addWidgetDown(new ofxUIToggle("CV_CALIB_FIX_K1",&kinectProjectorCalibration.b_CV_CALIB_FIX_K1, dim, dim));
	gui->addWidgetDown(new ofxUIToggle("CV_CALIB_FIX_K2",&kinectProjectorCalibration.b_CV_CALIB_FIX_K2, dim, dim));
	gui->addWidgetDown(new ofxUIToggle("CV_CALIB_FIX_K3",&kinectProjectorCalibration.b_CV_CALIB_FIX_K3, dim, dim));
	gui->addWidgetDown(new ofxUIToggle("CV_CALIB_RATIONAL_MODEL",&kinectProjectorCalibration.b_CV_CALIB_RATIONAL_MODEL, dim, dim));
 
	gui->addWidgetDown(new ofxUILabel(" ", OFX_UI_FONT_LARGE)); 
	gui->addWidgetDown(new ofxUILabel("Calibration", OFX_UI_FONT_LARGE));
    gui->addSpacer(length-xInit, 2);
	gui->addWidgetDown(new ofxUIButton("Clean dataset (remove > 2 reprojection error)", false, dim, dim));
	gui->addWidgetDown(new ofxUIButton("Clean dataset (remove > 5 reprojection error)", false, dim, dim));
	gui->addWidgetDown(new ofxUIButton("Clean dataset (remove all)", false, dim, dim));

	
	gui->addWidgetDown(new ofxUILabel(" ", OFX_UI_FONT_LARGE)); 
	gui->addWidgetDown(new ofxUILabel("Status", OFX_UI_FONT_LARGE)); 
    gui->addSpacer(length-xInit, 2);
	gui->addWidgetDown(new ofxUILabel("errorLabel",    "Avg Reprojection error: 0.0", OFX_UI_FONT_SMALL));	
	gui->addWidgetDown(new ofxUILabel("capturesLabel", "Number of captures:     0", OFX_UI_FONT_SMALL));
	gui->addWidgetDown(new ofxUIFPS(OFX_UI_FONT_SMALL));



	ofAddListener(gui->newGUIEvent,this,&ofApp::guiEvent);

}

void ofApp::guiUpdateLabels() {
	ofxUILabel* l;
	
	l = (ofxUILabel*) gui->getWidget("errorLabel");
	l->setLabel("Avg Reprojection error: " + ofToString(kinectProjectorCalibration.getReprojectionError(), 2));
	
	l = (ofxUILabel*) gui->getWidget("capturesLabel");
	l->setLabel("Number of captures:     " + ofToString(kinectProjectorCalibration.getDatabaseSize()));
}

void ofApp::guiEvent(ofxUIEventArgs &e)
{
	string name = e.widget->getName(); 
	int kind = e.widget->getKind(); 
	//if (name == "Projection Window fullscreen") {
		//secondWindow.toggleFullScreen();   
	//} 
	//else
	if (name == "Preview Kinect") {
		mode = PREVIEW;
	}
	else if (name == "Calibrate") {
		mode = CALIBRATION;
			kinectProjectorCalibration.resetTimer();
	}
	else if (name == "Test") {
		mode = TEST;
		calibrationLoaded = kinectProjectorOutput.load(SAVENAME);
		offsetX = kinectProjectorOutput.offsetX;
		offsetY = kinectProjectorOutput.offsetY;
	}	
	else if (name == "Clean dataset (remove > 2 reprojection error)") { 
		ofxUIButton* b = (ofxUIButton*)e.widget;
		if(b->getValue()) kinectProjectorCalibration.clean(2);	
	} 
	else if (name == "Clean dataset (remove > 5 reprojection error)") { 
		ofxUIButton* b = (ofxUIButton*)e.widget;
		if(b->getValue()) kinectProjectorCalibration.clean(5);	
	} 
	else if (name == "Clean dataset (remove all)") { 
		ofxUIButton* b = (ofxUIButton*)e.widget;
		if(b->getValue()) kinectProjectorCalibration.clearAll();
	
	} 
}

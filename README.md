ofxProjectorKinectV2Calibration
===============================

Openframeworks 084 addon for calibrating a kinect V2 and a projector, currently work in progress
info: http://forum.openframeworks.cc/t/projector-kinect-calibration/12712
WINDOWS ONLY!

Calibration procedure
---------------------
See this excellent video; the addon really implents Elliots video literally.
https://www.youtube.com/watch?feature=player_embedded&v=llQM-OGsETQ


Used addons
-----------
- ofxCv 			- https://github.com/kylemcdonald/ofxCv
- ofxKinectV2 		- https://github.com/Kj1/ofxKinectV2
- ofxSecondWindow 	- https://github.com/genekogan/ofxSecondWindow
- ofxUi 			- https://github.com/rezaali/ofxUI

Example videos
--------------
https://vimeo.com/75415111 (with kinectV1 but concept & gui is exactly the same)
https://vimeo.com/100504580


How to use the calibrated output
--------------------------------
See example.  The important stuff:

//[setup] load calibration file (once)
kinectProjectorOutput.load(saveFile);

//[draw] get contour of the person in the kinect label/person image
vector<ofPoint> points;
for (int j = 0; j < contourFinder.blobs[i].nPts ; j++) {
	points.push_back(contourFinder.blobs[i].pts[j]);
}
//Project points from depth XY coordinates to projector XY coordinates.
//internally, depth XY location is transformed to world (x,y,z) then to projector XY.
vector<ofPoint> projected = kinectProjectorOutput.projectFromDepthXYVector(points);
			
					
Chessboard finding parameters
----------------------------- 
- CV_CALIB_CB_ADAPTIVE_THRESH Use adaptive thresholding to convert the image to black and white, rather than a fixed threshold level (computed from the average image brightness).
- CV_CALIB_CB_NORMALIZE_IMAGE Normalize the image gamma with equalizeHist() before applying fixed or adaptive thresholding.
- CALIB_CB_FAST_CHECK Run a fast check on the image that looks for chessboard corners, and shortcut the call if none is found. This can drastically speed up the call in the degenerate condition when no chessboard is observed.


Calibration parameters
----------------------
- CV_CALIB_FIX_PRINCIPAL_POINT The principal point is not changed during the global optimization.
- CV_CALIB_FIX_ASPECT_RATIO The functions considers only fy as a free parameter. The ratio fx/fy stays the same as in the input cameraMatrix . When CV_CALIB_USE_INTRINSIC_GUESS is not set, the actual input values of fx and fy are ignored, only their ratio is computed and used further.
- CV_CALIB_ZERO_TANGENT_DIST Tangential distortion coefficients are set to zeros and stay zero.
- CV_CALIB_FIX_Kx http://docs.opencv.org/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html#calibratecamera
- CV_CALIB_RATIONAL_MODEL No idea
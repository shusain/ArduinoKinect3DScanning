#include "ofApp.h"
#include <fstream>
#include <cmath>
#include <vector>
#include <png++/png.hpp>
#include <SerialStream.h>


void ofApp::runSequence(){
//    moveTo(30);
//    sleep(2);
//    moveTo(60);
//    sleep(2);
//    moveTo(90);
//    sleep(2);
//    moveTo(120);
//    sleep(2);

}

//--------------------------------------------------------------
void ofApp::setup() {

	ofSetLogLevel(OF_LOG_VERBOSE);

	// enable depth->video image calibration
	kinect.setRegistration(true);

	kinect.init();
	//kinect.init(true); // shows infrared instead of RGB video image
	//kinect.init(false, false); // disable video image (faster fps)

	kinect.open();		// opens first available kinect
	//kinect.open(1);	// open a kinect by id, starting with 0 (sorted by serial # lexicographically))
	//kinect.open("A00362A08602047A");	// open a kinect using it's unique serial #

	// print the intrinsic IR sensor values
	if(kinect.isConnected()) {
		ofLogNotice() << "sensor-emitter dist: " << kinect.getSensorEmitterDistance() << "cm";
		ofLogNotice() << "sensor-camera dist:  " << kinect.getSensorCameraDistance() << "cm";
		ofLogNotice() << "zero plane pixel size: " << kinect.getZeroPlanePixelSize() << "mm";
		ofLogNotice() << "zero plane dist: " << kinect.getZeroPlaneDistance() << "mm";
	}

#ifdef USE_TWO_KINECTS
	kinect2.init();
	kinect2.open();
#endif

	colorImg.allocate(kinect.width, kinect.height);
	grayImage.allocate(kinect.width, kinect.height);
	grayThreshNear.allocate(kinect.width, kinect.height);
	grayThreshFar.allocate(kinect.width, kinect.height);

	nearThreshold = 230;
	farThreshold = 70;
	bThreshWithOpenCV = false;

	ofSetFrameRate(60);

    fileCount = 0;
	// zero the tilt on startup
	angle = -19;
	kinect.setCameraTiltAngle(angle);

	rotationAngle = 30;
	moveTo(rotationAngle);

	// start from the front
	bDrawPointCloud = false;

	bSaveImage = false;

    runSequence();
}


// Sends a message to the serial port to move the servo position to a particular angle
void ofApp::moveTo(int angle){
    using namespace LibSerial ;
    SerialStream my_serial_stream ;
    my_serial_stream.Open("/dev/ttyACM1" );
    my_serial_stream.SetBaudRate(SerialStreamBuf::BAUD_9600);
    my_serial_stream.SetCharSize(SerialStreamBuf::CHAR_SIZE_8);
    my_serial_stream.SetParity(SerialStreamBuf::PARITY_NONE);
    my_serial_stream.SetNumOfStopBits(1);
    my_serial_stream.SetFlowControl(SerialStreamBuf::FLOW_CONTROL_HARD);
    my_serial_stream << angle;
    cout << angle;
    my_serial_stream.Close();
    sleep(1);
}

//--------------------------------------------------------------
void ofApp::savePointCloud(ofMesh mesh){
    myfile.open ("example"+std::to_string(fileCount++)+".obj");

    std::vector<ofVec3f>::iterator it;
    std::vector<ofFloatColor>::iterator itColors;
    std::vector<ofVec3f> vertices =  mesh.getVertices();
    std::vector<ofFloatColor> meshColors = mesh.getColors();
    for(it = vertices.begin(),itColors = meshColors.begin(); it<vertices.end()&&itColors<meshColors.end();it++,itColors++){
        myfile << "v " << *it << " " << *itColors << endl;
    }
    myfile.close();

}

ofMesh ofApp::getMesh(){
	int w = 640;
	int h = 480;
	ofMesh mesh;
	mesh.setMode(OF_PRIMITIVE_POINTS);
	int step = 1;

    for(int y = 0; y < h; y += step) {
		for(int x = 200; x < w-200; x += step) {
			if(kinect.getDistanceAt(x, y) > 0) {
				ofVec3f coordinate = kinect.getWorldCoordinateAt(x, y);
				//cout << coordinate.z << endl;
				//ofDrawBitmapString(coordinate.z, 20, 610);
                    //cout << coordinate.y << endl;
                if(coordinate.z<1000 && coordinate.y<160){
                    ofColor col = kinect.getColorAt(x,y);
                    if(colorDistance(col, ofColor(197, 252, 232))<.05 ||
                       colorDistance(col, ofColor(180, 254, 196))<.05 ||
                       colorDistance(col, ofColor(158, 254, 116))<.10 ||
                       colorDistance(col, ofColor(167, 254, 162))<.05)
                        continue;

                    mesh.addColor(col);
                    mesh.addVertex(coordinate);
                }
			}
		}
	}

	glPointSize(3);
	ofPushMatrix();
	// the projected points are 'upside down' and 'backwards'
	ofScale(1, -1, -1);
	ofTranslate(0, 0, -1000); // center the points a bit
	ofEnableDepthTest();
	mesh.drawVertices();
	ofDisableDepthTest();
	ofPopMatrix();

	return mesh;

}

void ofApp::addToFile(ofstream &fileToFill, ofMesh mesh){
    std::vector<ofVec3f>::iterator it;
    std::vector<ofFloatColor>::iterator itColors;
    std::vector<ofVec3f> vertices =  mesh.getVertices();
    std::vector<ofFloatColor> meshColors = mesh.getColors();
    for(it = vertices.begin(),itColors = meshColors.begin(); it<vertices.end()&&itColors<meshColors.end();it++,itColors++){
        myfile << "v " << *it << " " << *itColors << endl;
    }
}

void ofApp::buildFullScan(){
    myfile.open ("fullscan"+std::to_string(fileCount++)+".obj");
    //Move the servo into position
    ofMesh currentMesh;


    for(int i = 30; i<420; i+=15){
        moveTo(i);
        sleep(3); // Give the servo a couple of seconds to move
        kinect.update();
        currentMesh = getMesh();
        rotatePoints(currentMesh, i);
        addToFile(myfile,currentMesh);

    }


    myfile.close();

}

//--------------------------------------------------------------
void ofApp::update() {
	ofBackground(100, 100, 100);

	kinect.update();

	// there is a new frame and we are connected
	if(kinect.isFrameNew()) {

		// load grayscale depth image from the kinect source
		grayImage.setFromPixels(kinect.getDepthPixels());

		// we do two thresholds - one for the far plane and one for the near plane
		// we then do a cvAnd to get the pixels which are a union of the two thresholds
		if(bThreshWithOpenCV) {
			grayThreshNear = grayImage;
			grayThreshFar = grayImage;
			grayThreshNear.threshold(nearThreshold, true);
			grayThreshFar.threshold(farThreshold);
			cvAnd(grayThreshNear.getCvImage(), grayThreshFar.getCvImage(), grayImage.getCvImage(), NULL);
		} else {

			// or we do it ourselves - show people how they can work with the pixels
			ofPixels & pix = grayImage.getPixels();
			int numPixels = pix.size();
			for(int i = 0; i < numPixels; i++) {
				if(pix[i] < nearThreshold && pix[i] > farThreshold) {
					pix[i] = 255;
				} else {
					pix[i] = 0;
				}
			}
		}

		// update the cv images
		grayImage.flagImageChanged();

		// find contours which are between the size of 20 pixels and 1/3 the w*h pixels.
		// also, find holes is set to true so we will get interior contours as well....
		contourFinder.findContours(grayImage, 10, (kinect.width*kinect.height)/2, 20, false);
	}

#ifdef USE_TWO_KINECTS
	kinect2.update();
#endif
}

//--------------------------------------------------------------
void ofApp::draw() {

	ofSetColor(255, 255, 255);

	if(bDrawPointCloud) {
		easyCam.begin();
		drawPointCloud();
		easyCam.end();
	} else {
		// draw from the live kinect
		kinect.drawDepth(10, 10, 400, 300);
		kinect.draw(420, 10, 400, 300);

		grayImage.draw(10, 320, 400, 300);
		contourFinder.draw(10, 320, 400, 300);

#ifdef USE_TWO_KINECTS
		kinect2.draw(420, 320, 400, 300);
#endif
	}

	// draw instructions
	ofSetColor(255, 255, 255);
	stringstream reportStream;

    if(kinect.hasAccelControl()) {
        reportStream << "accel is: " << ofToString(kinect.getMksAccel().x, 2) << " / "
        << ofToString(kinect.getMksAccel().y, 2) << " / "
        << ofToString(kinect.getMksAccel().z, 2) << endl;
    } else {
        reportStream << "Note: this is a newer Xbox Kinect or Kinect For Windows device," << endl
		<< "motor / led / accel controls are not currently supported" << endl << endl;
    }

	reportStream << "press p to switch between images and point cloud, rotate the point cloud with the mouse" << endl
	<< "using opencv threshold = " << bThreshWithOpenCV <<" (press spacebar)" << endl
	<< "set near threshold " << nearThreshold << " (press: + -)" << endl
	<< "set far threshold " << farThreshold << " (press: < >) num blobs found " << contourFinder.nBlobs
	<< ", fps: " << ofGetFrameRate() << endl
	<< ", foundColorPoint: " << foundColorPoint << endl
	<< "press c to close the connection and o to open it again, connection is: " << kinect.isConnected() << endl;

    if(kinect.hasCamTiltControl()) {
    	reportStream << "press UP and DOWN to change the tilt angle: " << angle << " degrees" << endl
        << "press 1-5 & 0 to change the led mode" << endl;
    }

	ofDrawBitmapString(reportStream.str(), 20, 652);

}

float ofApp::colorDistance(ofColor a, ofColor b){
    return sqrt(pow(a.r-b.r,2)+pow(a.g-b.g,2)+pow(a.b-b.b,2))/441.67295593;
}

void ofApp::rotatePoints(ofMesh &mesh, float angle){
    float radAngle = angle * M_PI/180;
    ofVec3f centroid = ofVec3f(10, 71, 562);
    ofMatrix4x4 matrix;
    matrix.makeRotationMatrix(angle, ofVec3f(0.1,1,0.5).normalize());

    for(int i = 0; i < mesh.getNumVertices(); i++){
        ofVec3f curVertex = mesh.getVertex(i);

        curVertex -= centroid;

        curVertex = curVertex * matrix;

        curVertex += centroid;

        mesh.setVertex(i,curVertex);
    }
}

void ofApp::drawPointCloud() {
	int w = 640;
	int h = 480;
	ofMesh mesh;
	mesh.setMode(OF_PRIMITIVE_POINTS);
	int step = 1;
    png::image<png::rgb_pixel> genImage;

    if(bSaveImage){
         genImage = png::image<png::rgb_pixel>(w,h);
    }
	for(int y = 0; y < h-50; y += step) {
		for(int x = 200; x < w-200; x += step) {
			if(kinect.getDistanceAt(x, y) > 0) {
				ofVec3f coordinate = kinect.getWorldCoordinateAt(x, y);
                if(coordinate.z<1000 && coordinate.y<160){
                    ofColor col = kinect.getColorAt(x,y);
                    if(colorDistance(col, ofColor(197, 252, 232))<.05 ||
                       colorDistance(col, ofColor(180, 254, 196))<.05 ||
                       colorDistance(col, ofColor(158, 254, 116))<.10 ||
                       colorDistance(col, ofColor(167, 254, 162))<.05)
                        continue;

                    mesh.addColor(col);
                    mesh.addVertex(coordinate);

                    if(colorDistance(col, ofColor(252, 90,179))<.15)
                      foundColorPoint = coordinate;

                    if(bSaveImage){
                         genImage[y][x] = png::rgb_pixel(col.r, col.g, col.b);
                    }
                }
			}
		}
	}

    if(bSaveImage){
        bSaveImage = false;
        genImage.write("testFile.png");

    }

    if(bTakeSnapshot){
        //savePointCloud(mesh);
        buildFullScan();
        bTakeSnapshot = false;
    }
	glPointSize(3);
	ofPushMatrix();
	// the projected points are 'upside down' and 'backwards'
	ofScale(1, -1, -1);
	ofTranslate(0, 0, -1000); // center the points a bit
	ofEnableDepthTest();
	mesh.drawVertices();
	ofDisableDepthTest();
	ofPopMatrix();
}

//--------------------------------------------------------------
void ofApp::exit() {
	//kinect.setCameraTiltAngle(0); // zero the tilt on exit
	kinect.close();

#ifdef USE_TWO_KINECTS
	kinect2.close();
#endif
}

//--------------------------------------------------------------
void ofApp::keyPressed (int key) {
	switch (key) {
		case ' ':
			bThreshWithOpenCV = !bThreshWithOpenCV;
			break;

		case 'b':
			bTakeSnapshot = !bTakeSnapshot;
			break;
		case 's':
			bSaveImage = !bSaveImage;
			break;
		case'p':
			bDrawPointCloud = !bDrawPointCloud;
			break;

		case '>':
		case '.':
			farThreshold ++;
			if (farThreshold > 255) farThreshold = 255;
			break;

		case '<':
		case ',':
			farThreshold --;
			if (farThreshold < 0) farThreshold = 0;
			break;

		case '+':
		case '=':
			nearThreshold ++;
			if (nearThreshold > 255) nearThreshold = 255;
			break;

		case '-':
			nearThreshold --;
			if (nearThreshold < 0) nearThreshold = 0;
			break;

		case 'w':
			kinect.enableDepthNearValueWhite(!kinect.isDepthNearValueWhite());
			break;

		case 'o':
			kinect.setCameraTiltAngle(angle); // go back to prev tilt
			kinect.open();
			break;

		case 'c':
			kinect.setCameraTiltAngle(0); // zero the tilt
			kinect.close();
			break;

		case '1':
			kinect.setLed(ofxKinect::LED_GREEN);
			break;

		case '2':
			kinect.setLed(ofxKinect::LED_YELLOW);
			break;

		case '3':
			kinect.setLed(ofxKinect::LED_RED);
			break;

		case '4':
			kinect.setLed(ofxKinect::LED_BLINK_GREEN);
			break;

		case '5':
			kinect.setLed(ofxKinect::LED_BLINK_YELLOW_RED);
			break;

		case '0':
			kinect.setLed(ofxKinect::LED_OFF);
			break;

		case OF_KEY_UP:
			angle++;
			if(angle>30) angle=30;
			kinect.setCameraTiltAngle(angle);
			break;

		case OF_KEY_DOWN:
			angle--;
			if(angle<-30) angle=-30;
			kinect.setCameraTiltAngle(angle);
			break;
	}
}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button)
{}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button)
{}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button)
{}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h)
{}

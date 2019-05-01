#pragma once

#include "ofMain.h"
#include "ofRenderApp.h"
#include "ofxOpenVR.h"


class ofApp : public ofBaseApp {

public:
	bool isFullscreen = false;
	bool bShowHelp = true;

	shared_ptr<ofAppBaseWindow> window1;
	shared_ptr<ofAppBaseWindow> window2;

	shared_ptr<RenderApp> app1;
	shared_ptr<RenderApp> app2;

	////

	// vector to store all values
	vector <ofVec3f> points;
	vector <ofVec3f> sizes;

	ofxOpenVR openVR;

	bool bUseShader;
	ofShader shader;

	float polylineResolution;

	vector<ofPolyline> leftControllerPolylines;
	vector<ofPolyline> rightControllerPolylines;
	bool bIsLeftTriggerPressed;
	bool bIsRightTriggerPressed;
	ofVec3f leftControllerPosition;
	ofVec3f rightControllerPosition;
	ofVec3f lastLeftControllerPosition;
	ofVec3f lastRightControllerPosition;

	bool bShowHelp;
	std::ostringstream _strHelp;

	ofVbo vbo;
	ofShader shaderP;
	ofEasyCam camera;
	ofTexture texture;

	void setup() {
		isFullscreen = 0;

		// load the texure
		ofDisableArbTex();
		ofLoadImage(texture, "dot.png");

		ofSetFrameRate(90);  // for VR
		ofSetVerticalSync(false); // for VR
		// We need to pass the method we want ofxOpenVR to call when rending the scene
		openVR.setup(std::bind(&ofApp::render, this, std::placeholders::_1));
		openVR.setDrawControllers(true);
		ofAddListener(openVR.ofxOpenVRControllerEvent, this, &ofApp::controllerEvent);

		shaderP.load("shaders/shader"); 
	}

	void exit() {
		ofRemoveListener(openVR.ofxOpenVRControllerEvent, this, &ofApp::controllerEvent);
		openVR.exit();
	}

	void update() {
		if (isFullscreen){
			ofHideCursor();
		} else {
			ofShowCursor();
		}
		openVR.update();
		if (bIsLeftTriggerPressed) {
			if (openVR.isControllerConnected(vr::TrackedControllerRole_LeftHand)) {
				// Getting the translation component of the controller pose matrix
				leftControllerPosition = openVR.getControllerPose(vr::TrackedControllerRole_LeftHand)[3];

				if (lastLeftControllerPosition.distance(leftControllerPosition) >= polylineResolution) {
					leftControllerPolylines[leftControllerPolylines.size() - 1].lineTo(leftControllerPosition);
					lastLeftControllerPosition.set(leftControllerPosition);
				}
			}
		}

		if (bIsRightTriggerPressed) {
			if (openVR.isControllerConnected(vr::TrackedControllerRole_RightHand)) {
				// Getting the translation component of the controller pose matrix
				rightControllerPosition = openVR.getControllerPose(vr::TrackedControllerRole_RightHand)[3];

				if (lastRightControllerPosition.distance(rightControllerPosition) >= polylineResolution) {
					rightControllerPolylines[rightControllerPolylines.size() - 1].lineTo(rightControllerPosition);
					lastRightControllerPosition = rightControllerPosition;
				}
			}
		}
	}

	void draw() {
		ofSetupScreen();  // sets up default perspective matrix

		openVR.render();
		openVR.renderDistortion();
		openVR.drawDebugInfo(10.0f, 500.0f);

		// Help
		if (bShowHelp) {
			_strHelp.str("");
			_strHelp.clear();
			_strHelp << "HELP (press h to toggle): " << endl;
			_strHelp << "Press the Trigger of a controller to draw a line with that specific controller." << endl;
			_strHelp << "Press the Touchpad to star a new line." << endl;
			_strHelp << "Press the Grip button to clear all the lines drawn with that specific controller." << endl;
			_strHelp << "Drawing resolution " << polylineResolution << " (press: +/-)." << endl;
			_strHelp << "Drawing default 3D models " << openVR.getRenderModelForTrackedDevices() << " (press: m)." << endl;
			ofDrawBitmapStringHighlight(_strHelp.str(), ofPoint(10.0f, 20.0f), ofColor(ofColor::black, 100.0f));
		}
	}

	void  render(vr::Hmd_Eye nEye) {
		
		ofMatrix4x4 currentViewProjectionMatrix = openVR.getCurrentViewProjectionMatrix(nEye);

		shader.begin();
		shader.setUniformMatrix4f("matrix", currentViewProjectionMatrix, 1);
		ofSetColor(ofColor::white);
		for (auto pl : leftControllerPolylines) {
			pl.draw();
		}

		for (auto pl : rightControllerPolylines) {
			pl.draw();
		}
		shader.end();

		vbo.draw(GL_POINTS, 0, (int)points.size());
	}

	void controllerEvent(ofxOpenVRControllerEventArgs& args) {
		//cout << "ofApp::controllerEvent > role: " << ofToString(args.controllerRole) << " - event type: " << ofToString(args.eventType) << " - button type: " << ofToString(args.buttonType) << " - x: " << args.analogInput_xAxis << " - y: " << args.analogInput_yAxis << endl;
		// Left
		if (args.controllerRole == ControllerRole::Left) {
			// Trigger
			if (args.buttonType == ButtonType::ButtonTrigger) {
				if (args.eventType == EventType::ButtonPress) {
					bIsLeftTriggerPressed = true;

					if (leftControllerPolylines.size() == 0) {
						leftControllerPolylines.push_back(ofPolyline());
						lastLeftControllerPosition.set(ofVec3f());
					}
				}
				else if (args.eventType == EventType::ButtonUnpress) {
					bIsLeftTriggerPressed = false;
				}
			}
			// ButtonTouchpad
			else if (args.buttonType == ButtonType::ButtonTouchpad) {
				if (args.eventType == EventType::ButtonPress) {
					leftControllerPolylines.push_back(ofPolyline());
					lastLeftControllerPosition.set(ofVec3f());
				}
			}
			// Grip
			else if (args.buttonType == ButtonType::ButtonGrip) {
				if (args.eventType == EventType::ButtonPress) {
					for (auto pl : leftControllerPolylines) {
						pl.clear();
					}

					leftControllerPolylines.clear();
				}
			}
		}

		// Right
		else if (args.controllerRole == ControllerRole::Right) {
			// Trigger
			if (args.buttonType == ButtonType::ButtonTrigger) {
				if (args.eventType == EventType::ButtonPress) {
					bIsRightTriggerPressed = true;

					if (rightControllerPolylines.size() == 0) {
						rightControllerPolylines.push_back(ofPolyline());
						lastRightControllerPosition.set(ofVec3f());
					}
				}
				else if (args.eventType == EventType::ButtonUnpress) {
					bIsRightTriggerPressed = false;
				}
			}
			// ButtonTouchpad
			else if (args.buttonType == ButtonType::ButtonTouchpad) {
				if (args.eventType == EventType::ButtonPress) {
					rightControllerPolylines.push_back(ofPolyline());
					lastRightControllerPosition.set(ofVec3f());
				}
			}
			// Grip
			else if (args.buttonType == ButtonType::ButtonGrip) {
				if (args.eventType == EventType::ButtonPress) {
					for (auto pl : rightControllerPolylines) {
						pl.clear();
					}

					rightControllerPolylines.clear();
				}
			}
		}
	}

	//--------------------------------------------------------------
	void addPoint(float x, float y, float z) {
		ofVec3f p(x, y, z);
		points.push_back(p); 

		// we are passing the size in as a normal x position
		float size = ofRandom(50, 100);
		sizes.push_back(ofVec3f(size));
	}

	void toggleFullScreen() { fullScreen(!isFullscreen); }

	void fullScreen(bool fs=true) {
		if (fs == isFullscreen) return;
		isFullscreen = fs;
		if(!isFullscreen) {
			ofSetWindowShape(300,300);
			ofSetFullscreen(false);
			//ofSetWindowPosition(100, 100);
		} else {
			ofSetFullscreen(true);
		}
	}

	void keyPressed(int key) {
		if(key == 'f'){
			toggleFullScreen();
		}

		switch (key) {
		case 'f':
			toggleFullScreen();
			break;
		case '+':
		case '=':
			polylineResolution += .0001f;
			break;

		case '-':
		case '_':
			polylineResolution -= .0001f;
			if (polylineResolution < 0) {
				polylineResolution = 0;
			}
			break;

		case 'h':
			bShowHelp = !bShowHelp;
			break;

		case 'm':
			openVR.setRenderModelForTrackedDevices(!openVR.getRenderModelForTrackedDevices());
			break;
		case 'a': {
			float theta1 = ofRandom(0, TWO_PI);
			float theta2 = ofRandom(0, TWO_PI);
			ofVec3f p;
			p.x = cos(theta1) * cos(theta2);
			p.y = sin(theta1);
			p.z = cos(theta1) * sin(theta2);
			p *= 800;
			addPoint(p.x, p.y, p.z);
			int total = (int)points.size();
			vbo.setVertexData(&points[0], total, GL_STATIC_DRAW);
			vbo.setNormalData(&sizes[0], total, GL_STATIC_DRAW);
		}
	}


		default:
			break;
		}
		

	}
	// void keyReleased(int key);
	// void mouseMoved(int x, int y );
	// void mouseDragged(int x, int y, int button);
	// void mousePressed(int x, int y, int button);
	// void mouseReleased(int x, int y, int button);
	// void mouseEntered(int x, int y);
	// void mouseExited(int x, int y);
	// void windowResized(int w, int h);
	// void dragEvent(ofDragInfo dragInfo);
	// void gotMessage(ofMessage msg);
	

};

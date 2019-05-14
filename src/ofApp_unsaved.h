#pragma once

#include "ofMain.h"
#include "ofxOpenVR.h"
#include "of3dGraphics.h"

#include "al/al_kinect2.h"

class ofApp : public ofBaseApp {

public:
	bool isFullscreen = false;
	bool bShowHelp = true;

	CloudDeviceManager cloudDeviceManager;

	// vector to store all values
	vector <ofVec3f> points;
	vector <ofVec3f> sizes;

	ofxOpenVR openVR;

	bool bUseShader;

	float polylineResolution = .004f;

	vector<ofPolyline> leftControllerPolylines;
	vector<ofPolyline> rightControllerPolylines;
	bool bIsLeftTriggerPressed = false;
	bool bIsRightTriggerPressed = false;
	ofVec3f leftControllerPosition;
	ofVec3f rightControllerPosition;
	ofVec3f lastLeftControllerPosition;
	ofVec3f lastRightControllerPosition;

	std::ostringstream _strHelp;

	ofVbo vbo;
	ofShader shaderP;
	ofTexture texture;

	ofTexture kinectTexture[2];
	ofShader shader;

	CloudFrame Cloud;

	bool pointsCreated = false;

	void setup() {
		isFullscreen = 0;

		if(1){
			// randomly add a point on a sphere
			//Number of particles set to a low number so that the point cloud can run
			int   num = 10;
			float radius = 1;
			for (int i = 0; i < num; i++) {

				float theta1 = ofRandom(0, TWO_PI);
				float theta2 = ofRandom(0, TWO_PI);

				ofVec3f p;
				p.x = cos(theta1) * cos(theta2);
				p.y = sin(theta1);
				p.z = cos(theta1) * sin(theta2);
				p *= radius;

				addPoint(p.x, p.y, p.z);
			}
		}

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

		lastLeftControllerPosition.set(ofVec3f());
		lastRightControllerPosition.set(ofVec3f());

		kinectTexture[0].allocate(cColorWidth, cColorHeight, GL_RGBA);
		kinectTexture[1].allocate(cColorWidth, cColorHeight, GL_RGBA);

		cloudDeviceManager.reset();
		cloudDeviceManager.open_all();

		// upload the data to the vbo
		int total = (int)points.size();
		vbo.setVertexData(&points[0], total, GL_STATIC_DRAW);
		vbo.setNormalData(&sizes[0], total, GL_STATIC_DRAW);
		shader.load("shaders_gl3/point");

	}

	void addPoint(float x, float y, float z) {
		ofVec3f p(x, y, z);
		points.push_back(p);

		// we are passing the size in as a normal x position
		float size = ofRandom(50, 100);
		sizes.push_back(ofVec3f(size));
	}

	void exit() {
		ofRemoveListener(openVR.ofxOpenVRControllerEvent, this, &ofApp::controllerEvent);
		openVR.exit();
	}

	void update() {

		std::stringstream strm;
		strm << "fps: " << ofGetFrameRate();
		ofSetWindowTitle(strm.str());

		for (int i=0; i<2; i++) {
			CloudDevice& kinect = cloudDeviceManager.devices[i];
			if (kinect.capturing) {
				// get most recent frames:
				const CloudFrame& cloud = kinect.cloudFrame();
				const ColourFrame& colour = kinect.colourFrame();

				if (!pointsCreated) {
					int   num = cDepthWidth * cDepthHeight;
					float radius = 1;
					for (int i = 0; i < num; i++) {

						ofVec3f p;
						p.x = cloud.xyz[i].x;
						p.y = cloud.xyz[i].y;
						p.z = cloud.xyz[i].z;

						addPoint(p.x, p.y, p.z);
						pointsCreated = true;
					} //TODO:: Figure out why the above code isn't displaying the point cloud. Also, find out why this eats up so much framerate!!
				}


				//kinectTexture[i].loadData((int8_t *)colour.color, cColorWidth, cColorHeight, GL_RGB);
			}
		}

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
		
		if (0) {
			for (int i = 0; i < 2; i++) {
				CloudDevice& kinect = cloudDeviceManager.devices[i];
				if (kinect.capturing) {
					// get most recent frames:
					const CloudFrame& cloud = kinect.cloudFrame();
					const ColourFrame& colour = kinect.colourFrame();
					kinectTexture[i].loadData((int8_t *)colour.color, cColorWidth, cColorHeight, GL_RGB);
				}
			}
		}
		//printf("Left Controller: %f %f %f \n Right Controller: %f %f %f \n", leftControllerPosition.x, leftControllerPosition.y, leftControllerPosition.z, rightControllerPosition.x, rightControllerPosition.y, rightControllerPosition.z);
	}

	void draw() {
		ofSetupScreen();  // sets up default perspective matrix
		ofBackground(0);

		openVR.render();
		openVR.renderDistortion();

		kinectTexture[0].draw(ofGetWidth() / 2, 0, ofGetWidth()/2, ofGetHeight()/2);
		kinectTexture[1].draw(ofGetWidth() / 2, ofGetHeight() / 2, ofGetWidth() / 2, ofGetHeight() / 2);

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

	void draw_scene(ofMatrix4x4 viewMatrix) {
		glDepthMask(GL_FALSE);
		// this makes everything look glowy :)
		ofEnableBlendMode(OF_BLENDMODE_ADD);
		ofEnablePointSprites();

		shader.begin();
		shader.setUniform1f("size", 2.f);
		shader.setUniformMatrix4f("modelViewProjectionMatrix", viewMatrix);

		glPointSize(40.f);

		texture.bind();
		vbo.draw(GL_POINTS, 0, (int)points.size());
		texture.unbind();

		shader.end();

		ofDisablePointSprites();
		ofDisableBlendMode();
		glDepthMask(GL_TRUE);

	}

	void render(vr::Hmd_Eye nEye) {
		
		ofMatrix4x4 currentViewProjectionMatrix = openVR.getCurrentViewProjectionMatrix(nEye);

		draw_scene(currentViewProjectionMatrix);

		
		

		if (0) {
			shaderP.begin();
			shaderP.setUniformMatrix4f("matrix", currentViewProjectionMatrix, 1);
			ofSetColor(ofColor::white);
			for (auto pl : leftControllerPolylines) {
				pl.draw();
			}

			for (auto pl : rightControllerPolylines) {
				pl.draw();
				//addPoint(0.1, 0.2, 0.1);
			}
			//vbo.draw(GL_POINTS, 0, (int)points.size());
			shaderP.end();
		}
	}

	void controllerEvent(ofxOpenVRControllerEventArgs& args) {
	//	cout << "ofApp::controllerEvent > role: " << ofToString(args.controllerRole) << " - event type: " << ofToString(args.eventType) << " - button type: " << ofToString(args.buttonType) << " - x: " << args.analogInput_xAxis << " - y: " << args.analogInput_yAxis << endl;
		
		//Left
		if (args.controllerRole == ControllerRole::Left) {
			// Trigger
			if (args.buttonType == ButtonType::ButtonTrigger) {
				if (args.eventType == EventType::ButtonPress) {
					bIsLeftTriggerPressed = true;
					addPoint(leftControllerPosition.x * 500.f, leftControllerPosition.y * 500.f, leftControllerPosition.z * 500.f);
					printf("Left Controller x: %f  y: %f  z: %f \n", leftControllerPosition.x, leftControllerPosition.y, leftControllerPosition.z);

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
					addPoint(rightControllerPosition.x * 800.f, rightControllerPosition.y * 800.f, rightControllerPosition.z * 800.f);
					printf("Right Controller x: %f  y: %f  z: %f \n", rightControllerPosition.x, rightControllerPosition.y, rightControllerPosition.z);

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

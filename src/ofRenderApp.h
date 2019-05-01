#pragma once

#include "ofMain.h"

class RenderApp : public ofBaseApp {

public:
	bool isFullscreen;

	shared_ptr<ofAppBaseWindow> window0;
	shared_ptr<ofAppBaseWindow> window1;

	ofShader shader;

	void setup() {
		isFullscreen = 0;
		ofSetFrameRate(60);

		shader.load("shaders_gl3/noise.vert", "shaders_gl3/noise.frag");
	}

	void setupWindow1() {
		ofSetBackgroundColor(0);
	}

	void update() {
		if (isFullscreen){
			ofHideCursor();
		} else {
			ofShowCursor();
		}
	}

	void draw() {
		ofSetupScreen();  // sets up default perspective matrix

		shader.begin();
		//we want to pass in some varrying values to animate our type / color 
		shader.setUniform1f("timeValX", ofGetElapsedTimef() * 0.1 );
		shader.setUniform1f("timeValY", -ofGetElapsedTimef() * 0.18 );
		
		//we also pass in the mouse position 
		//we have to transform the coords to what the shader is expecting which is 0,0 in the center and y axis flipped. 
		shader.setUniform2f("mouse", mouseX - ofGetWidth()/2, ofGetHeight()/2-mouseY );

		ofDrawRectangle(0, 0, 300, 300);

		shader.end();
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

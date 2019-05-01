#pragma once

#include "ofMain.h"

class RenderApp : public ofBaseApp {

public:
	bool isFullscreen;

	shared_ptr<ofAppBaseWindow> window0;
	shared_ptr<ofAppBaseWindow> window1;

	void setup() {
		isFullscreen = 0;
		ofSetFrameRate(60);
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

#pragma once

#include "ofMain.h"
#include "ofApp.h"

class RenderApp : public ofBaseApp {

public:
	bool isFullscreen;


	shared_ptr<ofApp> app;

	ofShader shader;
	ofTexture texture;

	ofVbo vbo;
	ofEasyCam camera;
	float camDist = 2;

	RenderApp(shared_ptr<ofApp> app) : app(app) {}

	void setup() {
		isFullscreen = 0;
		//ofSetFrameRate(60);
		//ofSetFrameRate(90);  // for VR
		ofSetVerticalSync(false); // for VR


		//ofSetBackgroundColor(0);
		ofBackgroundHex(0x000000);


		// load the texure
		ofDisableArbTex();
		ofLoadImage(texture, "dot.png");

		camera.setPosition(glm::vec3(0., 0., 3.));
		camera.setTarget(glm::vec3(0.f));
		camera.setNearClip(0.01);
		camera.setFarClip(10.f);
		//camera.setDistance(camDist);
		
		// upload the data to the vbo
		int total = (int)app->points.size();
		vbo.setVertexData(&app->points[0], total, GL_STATIC_DRAW);
		vbo.setNormalData(&app->sizes[0], total, GL_STATIC_DRAW);



		//shader.load("shaders_gl3/noise.vert", "shaders_gl3/noise.frag");
		shader.load("shaders_gl3/point");
	}

	void update() {


		// upload the data to the vbo
		int total = (int)app->points.size();
		vbo.setVertexData(&app->points[0], total, GL_STATIC_DRAW);
		vbo.setNormalData(&app->sizes[0], total, GL_STATIC_DRAW);

		std::stringstream strm;
		strm << "fps: " << ofGetFrameRate();
		ofSetWindowTitle(strm.str());

		if (isFullscreen){
			ofHideCursor();
		} else {
			ofShowCursor();
		}
	}

	void draw_scene(glm::mat4 viewMatrix, glm::mat4 projMatrix) {
		// TODO: is this in Simulation/shared?
		double now_s = ofGetElapsedTimeMillis() * 0.001;
		glm::mat4 viewProjectionMatrix = projMatrix * viewMatrix;



		glDepthMask(GL_FALSE);
		// this makes everything look glowy :)
		ofEnableBlendMode(OF_BLENDMODE_ADD);

		/*
		shaderIso.begin();
		shaderIso.setUniformMatrix4f("ciViewMatrix", viewMatrix);
		shaderIso.setUniformMatrix4f("ciProjectionMatrix", projMatrix);
		shaderIso.setUniform1f("uNow", now_s);
		// TODO:
		//shaderIso.setUniform1i("uGradient", 0);
		// bind mGooTex
		{ // wireframe
			shaderIso.setUniform1f("uAlpha", 0.15f);
			glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
			vboIso.draw();
		}
		shaderIso.setUniform1f("uAlpha", 0.2f);
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		vboIso.draw();
		shaderIso.end();
*/
		ofEnablePointSprites();

		shader.begin();
		shader.setUniform1f("size", 2.f);
		shader.setUniformMatrix4f("modelViewProjectionMatrix", viewProjectionMatrix);

		glPointSize(40.f);

		texture.bind();
		vbo.draw(GL_POINTS, 0, (int)app->points.size());
		texture.unbind();

		shader.end();

		ofDisablePointSprites();
		ofDisableBlendMode();
		glDepthMask(GL_TRUE);

	}

	void draw() {
		if (0) {
			ofSetupScreen();
			camera.begin();
			draw_scene(camera.getModelViewMatrix(), camera.getProjectionMatrix());
			camera.end();
		}

	if (0) {
		ofSetupScreen();
		//glDepthMask(GL_FALSE);

		ofSetColor(255, 100, 90);

		// this makes everything look glowy :)
		ofEnableBlendMode(OF_BLENDMODE_ADD);
		ofEnablePointSprites();

		// bind the shader and camera
		// everything inside this function
		// will be effected by the shader/camera
		camera.begin();
		shader.begin();
		shader.setUniform1f("size", 2.f);
		shader.setUniformMatrix4f("modelViewProjectionMatrix", camera.getModelViewProjectionMatrix());

		// bind the texture so that when all the points 
		// are drawn they are replace with our dot image
		texture.bind();
		vbo.draw(GL_POINTS, 0, (int)app->points.size());
		texture.unbind();

		camera.end();
		shader.end();

		ofDisablePointSprites();
		ofDisableBlendMode();
		//glDepthMask(GL_TRUE);
	}

	if (0) {
		// check to see if the points are 
	// sizing to the right size
		ofEnableAlphaBlending();
		camera.begin();
		for (unsigned int i = 0; i < app->points.size(); i++) {
			ofSetColor(255, 80);
			ofVec3f mid = app->points[i];
			mid.normalize();
			mid *= 300;
			ofDrawLine(app->points[i], mid);
		}
		camera.end();

		glDepthMask(GL_TRUE);

		ofSetColor(255, 100);
		ofDrawRectangle(0, 0, 250, 90);
		ofSetColor(0);
		string info = "FPS " + ofToString(ofGetFrameRate(), 0) + "\n";
		info += "Total Points " + ofToString((int)app->points.size()) + "\n";
		info += "Press 'a' to add more\n";
		info += "Press 'c' to remove all";

		ofDrawBitmapString(info, 20, 20);
	}

//		ofSetupScreen();  // sets up default perspective matrix

		// shader.begin();
		// //we want to pass in some varrying values to animate our type / color 
		// shader.setUniform1f("timeValX", ofGetElapsedTimef() * 0.1 );
		// shader.setUniform1f("timeValY", -ofGetElapsedTimef() * 0.18 );
		
		// //we also pass in the mouse position 
		// //we have to transform the coords to what the shader is expecting which is 0,0 in the center and y axis flipped. 
		// shader.setUniform2f("mouse", mouseX - ofGetWidth()/2, ofGetHeight()/2-mouseY );

		// ofDrawRectangle(0, 0, 300, 300);

		// shader.end();
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
		switch (key) {
		case 'f':
			toggleFullScreen();
			break;
		case 'a': {
#if 0
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
#endif
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

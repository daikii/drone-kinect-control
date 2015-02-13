#pragma once

#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxKinect.h"
#include "ofxOsc.h"

#define HOST "localhost"
#define PORT 8000

class ofApp : public ofBaseApp{
    
    public:
        void setup();
        void update();
        void draw();
        void mousePressed(int x, int y, int button);
        void keyPressed(int key);
    
        ofxKinect kinect;
    
        ofxCvColorImage colorImg;
        ofxCvColorImage hsbImg;
    
        ofxCvGrayscaleImage hueImg;
        ofxCvGrayscaleImage satImg;
        ofxCvGrayscaleImage briImg;
        ofxCvGrayscaleImage filtImg;
    
        ofxCvContourFinder contours;
    
        int kinectW;
        int kinectH;
    
        int trackSat;
    
        ofVec2f destPt;
        ofVec2f trackPt;
        ofVec2f dist;
    
        ofVec2f initTrPt;
        ofVec2f initHvPt;
    
        int thrust;
        int roll;
    
        bool trackSet = false;
        bool destSet = false;
        bool launchSet = false;
        bool quitSet = false;
    
        bool hasPassed = false;
    
        bool timerSet = false;
        int initTimer;
    
    
    private:
        ofxOscSender sender;
};
#pragma once

#include "ofMain.h"
#include "ofxARDrone.h"
#include "ofxARDroneOscBridge.h"
#include "ofxOpenCv.h"
#include "ofxKinect.h"
#include "ofxOsc.h"

class ofApp : public ofBaseApp{
    
    public:
        void setup();
        void update();
        void draw();
        void keyPressed(int key);
        void keyReleased(int key);
        void mouseDragged(int x, int y, int button);
        void mousePressed(int x, int y, int button);
    
        ofxARDrone::Drone drone;
        ofxARDrone::OscBridge droneOsc;
    
        bool keys[65535];
    
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
        ofVec2f trackPtPrev;
    
        ofVec2f error;
        ofVec2f errorAcc;
    
        // zone lengths for making brakes
        int zoneW = 30;
        int zoneH = 50;
    
        // end zone
        int zoneFin = 10;
    
        // PI correction amount for roll
        float pCorrR;
        float iCorrR;
    
        // PI coefficients for roll (0.0002)
        float KpR = 0.0002f;
        float KiR = 0.00006f;
    
        // PI correction amount for lift
        float pCorrL;
        float iCorrL;
    
        // PI coefficients for lift
        float KpL = 0.0002f;
        float KiL = 0.00001f;
    
        // sampling rate
        int sampling = 200;
        bool timerSet = false;
        int initTimer;
    
        int thrust;
        int roll;
    
        bool trackSet = false;
        bool destSet = false;
    
        // hover mode
        bool hoverSet = false;
};
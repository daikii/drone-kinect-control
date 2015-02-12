#include "ofApp.h"

//--------------------------------------------------------------

void ofApp::setup(){
    
    // background
    ofBackground(0,0,0);
    
    // connect to server
    sender.setup(HOST, PORT);
    
    // enable depth->video image calibration
    kinect.setRegistration(true);
    
    // init showing RGB
    kinect.init();
    
    // opens first available kinect
    kinect.open();
    
    kinectW = kinect.width;
    kinectH = kinect.height;
    
    // reserve memory for cv images
    colorImg.allocate(kinectW, kinectH);
    hsbImg.allocate(kinectW, kinectH);
    hueImg.allocate(kinectW, kinectH);
    satImg.allocate(kinectW, kinectH);
    briImg.allocate(kinectW, kinectH);
    filtImg.allocate(kinectW, kinectH);
}

//--------------------------------------------------------------

void ofApp::update(){
    
    //---------------------------------
    // KINECT - COLOR TRACKING
    //---------------------------------
    
    kinect.update();
    
    if(kinect.isFrameNew()){
        // copy kinect pixels to rgb image
        colorImg.setFromPixels(kinect.getPixels(), kinectW, kinectH);
        
        // duplicate rgb
        hsbImg = colorImg;
        
        // convert to hsb
        hsbImg.convertRgbToHsv();
        
        // store the three channels as grayscale images
        hsbImg.convertToGrayscalePlanarImages(hueImg, satImg, briImg);
        
        // filter image based on the hue/sat value were looking for
        for(int i = 0; i < kinectW * kinectH; i++) {
            filtImg.getPixels()[i] = ofInRange(satImg.getPixels()[i], trackSat - 5, trackSat + 5) ? 255 : 0;
        }
        filtImg.flagImageChanged();
        
        // run the contour finder on the filtered image to find blobs with a certain hue/sat
        contours.findContours(filtImg, 50, kinectW * kinectH / 2, 1, false);
        
        // store centroid of the tracked object
        for(int i = 0; i < contours.nBlobs; i++){
            trackPt.set(contours.blobs[i].centroid.x, contours.blobs[i].centroid.y);
        }
    }
    
    //---------------------------------
    // OSC - POSITION UPDATE
    //---------------------------------
    
    if(destSet && trackSet && launchSet){
        // OSC prep
        ofxOscMessage m;
    
        // OSC address select
        m.setAddress("/drone/position");
    
        // compute distance x,y from tracked pt to destination
        dist = initHvPt - trackPt;
        ofLog() << dist;
        
        if(isQuit){
            m.addIntArg(9);
        }
        
        // add first argument - roll
        if(dist[0] > 50){
            m.addIntArg(1);
        }else if(dist[0] < -50){
            m.addIntArg(2);
        }else{
            m.addIntArg(3);
        }
    
        // add second argument - thrust
        if(dist[1] > 160){
            m.addIntArg(1);
        }else if(dist[1] > 50){
            m.addIntArg(2);
        }else if(dist[1] < -160){
            m.addIntArg(3);
        }else if(dist[1] < -50){
            m.addIntArg(4);
        }else{
            m.addIntArg(5);
        }
        
        // send via OSC
        sender.sendMessage(m);
    }
}

//--------------------------------------------------------------

void ofApp::draw(){

    ofSetColor(255, 255, 255);
    
    // draw RGB image
    colorImg.draw(0, 0);
    
    // draw destination point
    if(destSet){
        ofSetColor(255, 0, 0);
        ofFill();
        ofCircle(destPt, 10);
    }
    
    // mark blob once hue/sat is chosen
    if(trackSet){
        // blob color
        ofSetColor(0, 255, 0);
        ofSetLineWidth(10);
        ofNoFill();
        
        // draw circle for found blobs
        for(int i = 0; i < contours.nBlobs; i++){
            ofCircle(contours.blobs[i].centroid.x, contours.blobs[i].centroid.y,
                     contours.blobs[i].boundingRect.width / 2);
        }
    }
    
    // mark the initial hover point. unmark once succeeded.
    if(launchSet && !hasPassed){
        ofSetColor(0, 0, 255);
        ofFill();
        ofCircle(initHvPt, 10);
    }
}

//--------------------------------------------------------------

void ofApp::mousePressed(int x, int y, int button){
    
    // get hue/sat value on mouse position
    trackSat = satImg.getPixels()[y * kinectW + x];
    ofLog() << "tracked object sat: " << trackSat;
    
    trackSet = true;
}

//--------------------------------------------------------------

void ofApp::keyPressed(int key){
    
    // set destination point at mouse location
    if(key == 'w'){
        destPt.set(mouseX, mouseY);
        destSet = true;
    }
    
    // trigger drone flight
    if(destSet && trackSet && key == 's'){
        // set initial hover point
        initHvPt = trackPt + ofVec2f(0, -180);
        launchSet = true;
    }
    
    if(key == 'q'){
        isQuit = true;
    }
}

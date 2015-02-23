/* -------------------------------------------------------------
 *
 * AR Drone quadcopter controller using Kinect
 *
 * thanks to ofxARDrone by Memo Akten
 *
 * daikii
 *
 * -------------------------------------------------------------
 */

#include "ofApp.h"

//--------------------------------------------------------------

void ofApp::setup(){
    
    // background
    ofBackground(0,0,0);
    
    ofSetVerticalSync(true);
    //ofSetFrameRate(60);
    //ofSetLogLevel(OF_LOG_VERBOSE);
    
    // clear all keys
    memset(keys, 0, sizeof(*keys));
    
    //---------------------------------
    // DRONE SETTINGS
    //---------------------------------
    
    // connect to the drone (yes, it's that easy)
    drone.connect();
    
    // setup osc so we can send & receive commands from another app to this one, to forward onto the drone (OPTIONAL)
    droneOsc.setup(&drone, 8000, 9000);
    
    //---------------------------------
    // KINECT + OPENCV SETTINGS
    //---------------------------------
    
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
            trackPtPrev.set(trackPt);
            trackPt.set(contours.blobs[i].centroid.x, contours.blobs[i].centroid.y);
        }
    }
    
    //---------------------------------
    // PI CONTROL POSITION UPDATE
    //---------------------------------
    
    if(!hoverSet){
        
        if(trackSet && destSet){
            // refresh when new dest selected
            if(hasNewPt){
                for(int i = 0; i < 2; i++){
                    hasReached[i] = false;
                }
                errorAcc.set(0, 0);
                hasNewPt = false;
            }
            
            // P: compute error distance x,y from tracked pt to destination
            error = (destPt - trackPt);
            
            // check closeness to destination
            if(abs(error[0]) < zoneFin){
                hasReached[0] = true;
            }
            if(abs(error[1]) < zoneFin){
                hasReached[1] = true;
            }
            
            // integrator for roll
            if(hasReached[0]){
                errorAcc[0] += error[0];
                // min and max check
                if(errorAcc[0] < intMin){
                    errorAcc[0] = intMin;
                }
                if(errorAcc[0] > intMax){
                    errorAcc[0] = intMax;
                }
                //ofLog() << "Integrator: " << errorAcc[0];
            }
        
            // PI: compute correction amount for roll
            pCorrR = KpR * error[0];
            iCorrR = KiR * errorAcc[0];
            
            // update roll
            drone.controller.rollAmount = pCorrR;
            
            // integrator for lift
            if(hasReached[1]){
                errorAcc[1] += error[1];
                // min and max check
                if(errorAcc[1] < intMin){
                    errorAcc[1] = intMin;
                }
                if(errorAcc[1] > intMax){
                    errorAcc[1] = intMax;
                }
                ofLog() << "Integrator: " << errorAcc[1];
            }
            
            // PI: compute correction amount for lift
            pCorrL = KpL * error[1];
            iCorrL = KiL * errorAcc[1];
            
            // update lift
            drone.controller.liftSpeed = -1 * (pCorrL + iCorrL);
            
            ofLog() << "error: " << error[1] << ", roll: " << drone.controller.rollAmount;
        
            // backup manual control
            manualCtrl();
        }
        
        // update the drone(process and send queued commands to drone,
        // receive commands from drone and update state)
        drone.update();
    }
    
    // hover mode, reset
    else{
        error.set(0, 0);
        ofLog() << "!!!!!!!HOVER MODE!!!!!!!!";
    }
    
    // receive and send relevant osc commands to any other apps (OPTIONAL)
    droneOsc.update();
}

//--------------------------------------------------------------

void ofApp::closenessCheck(){
    
    
}

//--------------------------------------------------------------

void ofApp::singleAxisCtrl(){
    
    // PI: compute error distance x,y from tracked pt to destination
    error = (destPt - trackPt);
    errorAcc += error;
    
    // roll direction check
    if(error[0] > 0){
        dirX = true;
    }else{
        dirX = false;
    }
    
    // save point-to-point distance
    if(hasNewPt){
        dist = error;
        hasNewPt = false;
        // scale zone range depending on the pt-pt distance
        zoneW = ofMap(abs(dist[0]), 0, 300, zoneWmin, zoneWmax);
        ofLog() << "NEW PT: " << dist[0] << " " << zoneW;
    }
    
    // enter hover mode once very close to destination
    if(abs(error[0]) < zoneFin){
        hoverSet = true;
    }
    
    // zone entrance check
    // apply brake toward opposite direction
    if(abs(error[0]) < zoneW){
        float temp = ofMap(abs(error[0]), zoneW, zoneFin, 0, brakeMax);
        error[0] = zoneW * temp;
        if(dirX){
            error[0] *= -1;
        }
        ofLog() << "ZONE: " << temp;
    }
    
    // PI: compute correction amount for roll
    pCorrR = KpR * error[0];
    iCorrR = KiR * errorAcc[0];
    
    // update roll
    drone.controller.rollAmount = pCorrR;
}

//--------------------------------------------------------------

void ofApp::manualCtrl(){
    
    //---------------------------------
    // BACKUP MANUAL CONTROL
    //---------------------------------
    
    float s = 0.02;
    
    //if(keys[OF_KEY_UP]) drone.controller.liftSpeed += s;
    //else if(keys[OF_KEY_DOWN]) drone.controller.liftSpeed -= s;
    
    //if(keys['a']) drone.controller.rollAmount -= s;
    //else if(keys['d']) drone.controller.rollAmount += s;
    
    if(keys['w']) drone.controller.pitchAmount -= s;
    else if(keys['s']) drone.controller.pitchAmount += s;
    
    if(keys[OF_KEY_LEFT]) drone.controller.spinSpeed -= s;
    else if(keys[OF_KEY_RIGHT]) drone.controller.spinSpeed += s;
}

//--------------------------------------------------------------

void ofApp::draw(){
    
    ofSetColor(255, 255, 255);
    
    // draw RGB image
    colorImg.draw(0, 0);
    
    // battery indication. <20% unstable.
    ofxARDrone::State &state = drone.state;
    string spec = "";
    spec = "battery level: "+ ofToString(state.getBatteryPercentage())+"%\n";
    ofDrawBitmapString(spec, 50, ofGetHeight() - 20);
    
    // fps on top-left
    ofDrawBitmapString(ofToString((int) ofGetFrameRate()), 50, ofGetHeight() - 50);
    
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
}

//--------------------------------------------------------------

void ofApp::keyPressed(int key){
    
    switch(key){
        case '1': drone.controller.exitBootstrap(); break;
        case '2': drone.controller.sendAck(); break;
        case '3': drone.dataReceiver.sendDummyPacket(); break;
        case '0': drone.controller.resetCommunicationWatchdog(); break;
            
        // take off and go to the initial hover point
        case 't':
            drone.controller.takeOff(!drone.state.isTakingOff(), 3000);
            destPt.set(ofGetWidth() / 2 + 100, ofGetHeight() / 2);
            destSet = true;
            break;
            
        case 'l': drone.controller.land(!drone.state.isLanding(), 3000); break;
        case 'c': drone.controller.calibrateHorizontal(!drone.state.isCalibratingHorizontal(), 3000); break;
        case 'm': drone.controller.calibrateMagnetometer(!drone.state.isCalibratingMagnetometer(), 3000); break;
            
        // hover mode (stops drone.update())
        case 'h': hoverSet ^= true; break;
            
        // force quit
        case 'E': drone.controller.emergency(0);break;
        case 'e': drone.controller.emergency(1); break;
            
        case 'f': ofToggleFullscreen(); break;
            
        // set destination point at mouse location
        case 'v':
            //destPt.set(mouseX, mouseY);
            destPt.set(mouseX, mouseY);
            hoverSet = false;
            hasNewPt = true;
            break;
    }
    
    // set key press bool
    keys[key] = true;
}

//--------------------------------------------------------------

void ofApp::keyReleased(int key){
    
    // reset when released
    switch(key) {
        case OF_KEY_UP:
        case OF_KEY_DOWN:
            drone.controller.liftSpeed = 0;
            break;
            
        case OF_KEY_LEFT:
        case OF_KEY_RIGHT:
            drone.controller.spinSpeed = 0;
            break;
            
        case 'w':
        case 's':
            drone.controller.pitchAmount = 0;
            break;
            
        case 'a':
        case 'd':
            drone.controller.rollAmount = 0;
            break;
    }
    
    keys[key] = false;
}

//--------------------------------------------------------------

void ofApp::mousePressed(int x, int y, int button){
    
    // get hue/sat value on mouse position
    trackSat = satImg.getPixels()[y * kinectW + x];
    ofLog() << "tracked object sat: " << trackSat;
    
    trackSet = true;
}

//--------------------------------------------------------------

void ofApp::mouseDragged(int x, int y, int button){
    
    mousePressed(x, y, button);
}
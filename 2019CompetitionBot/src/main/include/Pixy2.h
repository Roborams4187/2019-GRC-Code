/*//---------------------------------
Header file for sending requests to Pixy2 via I2C.
*///---------------------------------

#include <iostream>
#include <string>

#include "frc/I2C.h"
namespace martin {


struct PixyVector { //Structure object for storing values about vector
    int m_x0; //Point 0 x coordinate
    int m_y0; //Point 0 y coordinate
    int m_x1; //Point 1 x coordinate
    int m_y1; //Point 1 y coordinate
};

PixyVector getVector(frc::I2C *pixy) { //Returns PixyVector object containing vector info 
     PixyVector vector_;
     uint8_t pixyRequest[6] = {174,193,48,2,0,1};
     uint8_t pixyReceive[15];
     uint8_t* refPixyReq = &pixyRequest[0];
     uint8_t* refPixyRec = &pixyReceive[0];
     pixy->Transaction(refPixyReq,6,refPixyRec,12);
     vector_.m_x0 = pixyReceive[8];
     vector_.m_y0 = pixyReceive[9];
     vector_.m_x1 = pixyReceive[10];
     vector_.m_y1 = pixyReceive[11];
    return vector_;
}

void setBrightness(frc::I2C *pixy, uint8_t brightness) { //Sets pixy brightness to disconcern lines from background
    uint8_t pixyRequest[5] = {174,193,16,1,brightness}; //Max brightness is 255
    uint8_t pixyReceive[10];
    uint8_t* refPixyReq = &pixyRequest[0];
    uint8_t* refPixyRec = &pixyReceive[0];
    pixy->Transaction(refPixyReq,5,refPixyRec,10);
}

void setLamp(frc::I2C *pixy, uint8_t upper, uint8_t lower) { //Set LED lamp on pixy. int 1 is on, 0 is off.
    uint8_t pixyRequest[6] = {174,193,22,2,upper,lower};
    uint8_t pixyReceive[10];
    uint8_t* refPixyReq = &pixyRequest[0];
    uint8_t* refPixyRec = &pixyReceive[0];
    pixy->Transaction(refPixyReq,6,refPixyRec,10);
}

void setLED(frc::I2C *pixy, uint8_t r, uint8_t g, uint8_t b) { //Pixy has onboard LED that can be changed. Takes RGB 0-255
    uint8_t pixyRequest[7] = {174,193,20,3,r,g,b};
    uint8_t pixyReceive[10];
    uint8_t* refPixyReq = &pixyRequest[0];
    uint8_t* refPixyRec = &pixyReceive[0];
    pixy->Transaction(refPixyReq,7,refPixyRec,10);
}



}
#include "DSO-238.h"

// TFT display constants
#define PORTRAIT 		0
#define LANDSCAPE 		1

#define TFT_WIDTH		320
#define TFT_HEIGHT		240
#define GRID_WIDTH		300
#define GRID_HEIGHT		210

#define GRID_COLOR		0x4208
#define ADC_MAX_VAL		4096
#define ADC_2_GRID		800



// grid variables
constexpr uint8_t hOffset = (TFT_WIDTH - GRID_WIDTH) / 2;
constexpr uint8_t vOffset = (TFT_HEIGHT - GRID_HEIGHT) / 2;
constexpr uint8_t dHeight = GRID_HEIGHT / 8;


const char* cplNames[] = { "GND", "AC ", "DC " };
const char* rngNames[] = { "5V  ", "2V  ", "1V  ", "0.5V", "0.2V", "0.1V", "50mV", "20mV", "10mV" };
const char* tbNames[] = { "500 s/div ", "200 s/div ", "100 s/div ", "50 s/div  ", "20 s/div  ", "10 s/div  ", "5 s/div   ", "2 s/div   ", "1 s/div   ", "0.5 s/div ", "0.2 s/div ", "0.1 s/div ", "50 ms/div ", "20 ms/div ", "10 ms/div ", "5 ms/div  ", "2 ms/div  ", "1 ms/div  ", "0.5 ms/div", "0.2 ms/div", "0.1 ms/div", "50 us/div ", "20 us/div ", "10 us/div ", "5 us/div  ", "2 us/div  ", "1 us/div  " };

Adafruit_TFTLCD_8bit_STM32 tft;

// ------------------------
void initDisplay(){
// ------------------------
    tft.reset();
    tft.begin(0x9341);
    tft.setRotation(LANDSCAPE);
    tft.fillScreen(ILI9341_BLACK);
    banner();

    //delay(4000);

    // and paint o-scope
    clearWaves();
}


// ------------------------
void clearWaves(){
// ------------------------
    // clear screen
    tft.fillScreen(ILI9341_BLACK);
    // and paint o-scope
    drawGrid();
    drawAllLabels();
}



// Draw main grid
void drawGrid(){

    constexpr uint8_t hPacing = GRID_WIDTH / 12;
    constexpr uint8_t vPacing = GRID_HEIGHT / 8;

    for(int i = 1; i < 12; i++)
        tft.drawFastVLine(i * hPacing + hOffset, vOffset, GRID_HEIGHT, GRID_COLOR);

    for(int i = 1; i < 8; i++)
        tft.drawFastHLine(hOffset, i * vPacing + vOffset, GRID_WIDTH, GRID_COLOR);

    for(int i = 1; i < 5 * 8; i++)
        tft.drawFastHLine(hOffset + GRID_WIDTH / 2 - 3, i * vPacing / 5 + vOffset, 7, GRID_COLOR);

    for(int i = 1; i < 5 * 12; i++)
        tft.drawFastVLine(i * hPacing / 5 + hOffset, vOffset + GRID_HEIGHT / 2 - 4, 7, GRID_COLOR);

    tft.drawRect(hOffset, vOffset, GRID_WIDTH, GRID_HEIGHT, ILI9341_WHITE);
}

void drawHoldLabel(bool hold){
    // paint run/hold information
    // -----------------
    //clear rect
    tft.setCursor(hOffset + 2, 4);

    if(hold){
        tft.setTextColor(ILI9341_WHITE, ILI9341_RED);
        tft.print(" HOLD ");
    }
    else{
        tft.setTextColor(ILI9341_GREEN, ILI9341_BLACK);
        tft.print("RUN");
    }
}

void drawXposBar(uint16_t xPos){
    // draw x-window at top, range = 200px
    // -----------------
    int sampleSizePx = 160;
    float lOffset = (TFT_WIDTH - sampleSizePx) / 2;
    tft.drawFastVLine(lOffset, 3, vOffset - 6, ILI9341_GREEN);
    tft.drawFastVLine(lOffset + sampleSizePx, 3, vOffset - 6, ILI9341_GREEN);
    tft.drawFastHLine(lOffset, vOffset / 2, sampleSizePx, ILI9341_GREEN);

    // where does xCursor lie in this range
    float windowSize = GRID_WIDTH * sampleSizePx / NUM_SAMPLES;
    float xCursorPx = xPos * sampleSizePx / NUM_SAMPLES + lOffset;
    if(currentFocus == L_hPos)
        tft.drawRect(xCursorPx, 4, windowSize, vOffset - 8, ILI9341_WHITE);
    else
        tft.fillRect(xCursorPx, 4, windowSize, vOffset - 8, ILI9341_GREEN);
}

void drawActiveWaves(){
    // print active wave indicators
    // -----------------
    tft.drawRect(247, 0, 72, vOffset, ILI9341_BLACK);
    
    tft.setCursor(250, 4);
    if(GET_CHANNEL(ANALOG1)){
        tft.setTextColor(AN_SIGNAL1, ILI9341_BLACK);
        tft.print("A1 ");
    }
    else
        tft.print("   ");

  //tft.setCursor(267, 4);
    if(GET_CHANNEL(ANALOG2)){
        tft.setTextColor(AN_SIGNAL2, ILI9341_BLACK);
        tft.print("A2 ");
    }
    else
        tft.print("   ");

  //tft.setCursor(284, 4);
    if(GET_CHANNEL(DIGITAL1)){
        tft.setTextColor(DG_SIGNAL1, ILI9341_BLACK);
        tft.print("D1 ");
    }

  //tft.setCursor(301, 4);
    if(GET_CHANNEL(DIGITAL2)){
        tft.setTextColor(DG_SIGNAL2, ILI9341_BLACK);
        tft.print("D2 ");
    }
    else
        tft.print("   ");

    if(currentFocus == L_waves)
        tft.drawRect(247, 0, 72, vOffset, ILI9341_WHITE);
}

void drawWaveYCursors(){
    // draw new wave cursors
    // -----------------
    // if(waves[3])
    //     drawVCursor(3, DG_SIGNAL2, (currentFocus == L_vPos4));
    // if(waves[2])
    //     drawVCursor(2, DG_SIGNAL1, (currentFocus == L_vPos3));
    // if(waves[1])
    //     drawVCursor(1, AN_SIGNAL2, (currentFocus == L_vPos2));
    // if(waves[0])
    //     drawVCursor(0, AN_SIGNAL1, (currentFocus == L_vPos1));
}

void drawAnalogSwitchPos(VerticalRanges rngPos, CouplingType cplPos){
    // print input switch pos
    // -----------------
    tft.setTextColor(ILI9341_YELLOW, ILI9341_BLACK);
    tft.setCursor(hOffset + 2, GRID_HEIGHT + vOffset + 4);
    tft.print(rngNames[rngPos]);
    tft.setCursor(hOffset + 40, GRID_HEIGHT + vOffset + 4);
    tft.print(cplNames[cplPos]);
}

void drawTimebase(TimeBases timeBase){
    // print new timebase
    // -----------------
    tft.drawRect(80, GRID_HEIGHT + vOffset, 70, vOffset, ILI9341_BLACK);
    
    tft.setTextColor(ILI9341_GREEN, ILI9341_BLACK);
    tft.setCursor(85, GRID_HEIGHT + vOffset + 4);
    if(currentFocus == L_timebase)
        tft.drawRect(80, GRID_HEIGHT + vOffset, 70, vOffset, ILI9341_WHITE);
    tft.print(tbNames[timeBase]);
    //tft.print("/div");
}

void drawTriggerType(TriggerType trigType){
    // print trigger type
      // -----------------
    tft.drawRect(170, GRID_HEIGHT + vOffset, 35, vOffset, ILI9341_BLACK);
    
    tft.setTextColor(ILI9341_PINK, ILI9341_BLACK);
    tft.setCursor(175, GRID_HEIGHT + vOffset + 4);//230
    if(currentFocus == L_triggerType)
        tft.drawRect(170, GRID_HEIGHT + vOffset, 35, vOffset, ILI9341_WHITE);//225

    switch(trigType){
        case TRIGGER_AUTO:
            tft.print("AUTO");
            break;
        case TRIGGER_NORM:
            tft.print("NORM");
            break;
        case TRIGGER_SINGLE:
            tft.print("SING");
            break;
    }
}

void drawTriggerEdge(bool rising){
    // draw trigger edge
    // -----------------
    tft.drawRect(211, GRID_HEIGHT + vOffset, 15, vOffset + 4, ILI9341_BLACK);
    
    if(currentFocus == L_triggerEdge)
        tft.drawRect(211, GRID_HEIGHT + vOffset, 15, vOffset + 4, ILI9341_WHITE);//266

    int trigX = 215;//270

    if(rising){
        tft.drawFastHLine(trigX, TFT_HEIGHT - 3, 5, ILI9341_PINK);
        tft.drawFastVLine(trigX + 4, TFT_HEIGHT - vOffset + 2, vOffset - 4, ILI9341_PINK);
        tft.drawFastHLine(trigX + 4, TFT_HEIGHT - vOffset + 2, 5, ILI9341_PINK);
        tft.fillTriangle(trigX + 2, 232, trigX + 4, 230, trigX + 6, 232, ILI9341_PINK);
    }
    else{
        tft.drawFastHLine(trigX + 4, TFT_HEIGHT - 3, 5, ILI9341_PINK);
        tft.drawFastVLine(trigX + 4, TFT_HEIGHT - vOffset + 2, vOffset - 4, ILI9341_PINK);
        tft.drawFastHLine(trigX - 1, TFT_HEIGHT - vOffset + 2, 5, ILI9341_PINK);
        tft.fillTriangle(trigX + 2, 231, trigX + 4, 233, trigX + 6, 231, ILI9341_PINK);
    }
}

void drawTriggerVoltage(){
    //draw trigger voltage readout
    tft.setTextColor(ILI9341_PINK, ILI9341_BLACK);
    tft.setCursor(275, GRID_HEIGHT + vOffset + 4);
    tft.print(1.25);
    tft.print("V");
}

void drawTriggerPos(){
    // draw trigger level on right side
      // -----------------
    // int cPos = GRID_HEIGHT + vOffset + yCursors[0] - getTriggerLevel() / 3;
    // tft.fillTriangle(TFT_WIDTH, cPos - 5, TFT_WIDTH - hOffset, cPos, TFT_WIDTH, cPos + 5, AN_SIGNAL1);
    // if(currentFocus == L_triggerLevel)
    //     tft.drawRect(GRID_WIDTH + hOffset, cPos - 7, hOffset, 14, ILI9341_WHITE);
}

// ------------------------
void drawAllLabels(){
// ------------------------
    // draw the static labels around the grid

    // erase top bar
    //tft.fillRect(hOffset, 0, TFT_WIDTH, vOffset, ILI9341_BLACK);
    //tft.fillRect(hOffset + GRID_WIDTH, 0, hOffset, TFT_HEIGHT, ILI9341_BLACK);
    drawHoldLabel(false);
    drawXposBar(20);
    drawActiveWaves();

    // erase left side of grid
    //tft.fillRect(0, 0, hOffset, TFT_HEIGHT, ILI9341_BLACK);
    drawWaveYCursors();

    // erase bottom bar
    //tft.fillRect(hOffset, GRID_HEIGHT + vOffset, TFT_WIDTH, vOffset, ILI9341_BLACK);
    drawAnalogSwitchPos(rngPos, cplPos);
    drawTimebase(currentTimeBase);
    drawTriggerType(triggerType);
    drawTriggerEdge(triggerRising);
    drawTriggerVoltage();

    //right
    drawTriggerPos();
}

bool redrawLabel(MainCursorPos which, TimeBases timeBase, TriggerType trigType, bool rising, uint16_t xPos){
    switch(which){
        case L_timebase:
            drawTimebase(timeBase);
            break;
        case L_triggerType:
            drawTriggerType(trigType);
            break;
        case L_triggerEdge:
            drawTriggerEdge(rising);
            break;
        case L_settings:
        break;
        case L_triggerLevel:
            drawTriggerPos();
            break;
        case L_waves:
            drawActiveWaves();
            break;
        case L_hPos:
            drawXposBar(xPos);
            break;
        case L_vPos1:
        case L_vPos2:
        case L_vPos3:
        case L_vPos4:
            drawWaveYCursors();
            break;
        default:
            return false;
    }
    return true;
}


// Display initial startup screen
void banner(){

    tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
    tft.setTextSize(2);
    tft.setCursor(110, 30);
    tft.print("DSO-238");
    tft.drawRect(100, 25, 100, 25, ILI9341_WHITE);

    tft.setTextSize(1);
    tft.setCursor(30, 70);
    tft.print("Dual Channel Oscilloscope with logic analyzer");

    tft.setCursor(30, 100);
    tft.print("Firmware by barty12, web:");
    tft.setTextColor(ILI9341_GREEN, ILI9341_BLACK);
    tft.setCursor(40, 110);
    tft.print("https://github.com/barty32/DSO-238");

    tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
    tft.setCursor(30, 130);
    tft.print("Based on a firmware by Ardyesp, web:");
    tft.setTextColor(ILI9341_YELLOW, ILI9341_BLACK);
    tft.setCursor(40, 140);
    tft.print("https://github.com/ardyesp/DLO-138");

    tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
    tft.setCursor(30, 160);
    tft.print("DSO-138 hardware by JYE-Tech, web: ");
    tft.setTextColor(ILI9341_YELLOW, ILI9341_BLACK);
    tft.setCursor(40, 170);
    tft.print("https://jyetech.com");

    tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
    tft.setCursor(30, 190);
    tft.print("Firmware version: ");
    tft.print(FIRMWARE_VERSION);

    tft.setTextSize(1);
    tft.setCursor(30, 220);
    tft.print("GNU GENERAL PUBLIC LICENSE Version 3");
}
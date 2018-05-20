import javafx.stage.Screen;
import processing.serial.*;
import java.util.Random;

Random rand = new Random();
ArrayList<ScanLineChart> charts;
int chartsNum = -1;

void setup() {
    // init windows
    size(400, 400);
    //fullScreen();
    frameRate(refreshRate);
    
    colorMode(RGB, 255, 255, 255);
    background(BACKGROUND_COLOR);
    noStroke();
    
    if (frame != null) {
        surface.setResizable(true);
    }

    // uart setup
    int id = 1;
    println(Serial.list()[id]);
    uart = new UART(new Serial(this, Serial.list()[id], 38400));

    // init array
    DataCaller_uint8_t = new int[uint8_t_MailBox.MaxTerm.ordinal()+1];
    for (int i = 0; i < uint8_t_MailBox.MaxTerm.ordinal(); i++) {
        DataCaller_uint8_t[i] = (byte) 0;
    }

    DataCaller_double = new double[double_MailBox.MaxTerm.ordinal()+1];
    for (int i = 0; i < double_MailBox.MaxTerm.ordinal(); i++) {
        DataCaller_double[i] = 0;
    }
    
    DataCaller_float = new float[float_MailBox.MaxTerm.ordinal()+1];
    for (int i = 0; i < float_MailBox.MaxTerm.ordinal(); i++) {
        DataCaller_float[i] = 0;
    }
    
    charts = new ArrayList<ScanLineChart>();
    init();
    for (int i = 0; i < charts.size(); i++) {
        charts.get(i).init();
        tiles.add(charts.get(i));
    }
}

int pastWidth  = 0;
int pastHeight = 0;

void draw() {
    uart.tSerialEvent();
    
    if (pastWidth != width || pastHeight != height) {
        clear();
        background(BACKGROUND_COLOR);
        pastWidth = width;
        pastHeight = height;
        tileAllocator();
    }
    
    for (int i = 0; i < tiles.size(); i++) {
        tiles.get(i).over();
        tiles.get(i).tDraw();
    }
}

void mousePressed() {
    for (int i = 0; i < tiles.size(); i++) {
        tiles.get(i).onClick();
    }
};

import java.nio.charset.StandardCharsets;
import javafx.stage.Screen;
import processing.serial.*;
import java.util.Random;

Random rand = new Random();
ArrayList<ScanLineChart> charts;
int chartsNum = -1;
InputBox InputBoxObj = null;
int TotalTrashedByte = 0;

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
    String target = null;
    
    target = ""; 
        
    int id = 0;
    while (((Serial.list().length > id) && (target != null)) && (!Serial.list()[id].contains(target))) {
        id++;
    }
    
    if (!(Serial.list().length > id)) {
        println("UART: ERROR: Can't Connect BT");
        while (true);
    }
    
    id = 0; // change the id value here!!!
    
    println("UART port selected is " + Serial.list()[id]);
    uart = new UART(new Serial(this, Serial.list()[id], 38400));

    // init array
    DataArray_uint8_t = new int[uint8_t_MailBox.MaxTerm.ordinal()+1];
    for (int i = 0; i < uint8_t_MailBox.MaxTerm.ordinal(); i++) {
        DataArray_uint8_t[i] = (byte) 0;
    }

    DataArray_double = new double[double_MailBox.MaxTerm.ordinal()+1];
    for (int i = 0; i < double_MailBox.MaxTerm.ordinal(); i++) {
        DataArray_double[i] = 0;
    }

    DataArray_float = new float[float_MailBox.MaxTerm.ordinal()+1];
    for (int i = 0; i < float_MailBox.MaxTerm.ordinal(); i++) {
        DataArray_float[i] = 0;
    }

    DataArray_int = new int[int_MailBox.MaxTerm.ordinal()+1];
    for (int i = 0; i < int_MailBox.MaxTerm.ordinal(); i++) {
        DataArray_int[i] = 0;
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
    cycle();

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

    if (InputBoxObj != null) {
        InputBoxObj.tDraw();
    }
    GLOBAL_RESTART = GLOBAL_RESTART > 0 ? GLOBAL_RESTART - 1 : 0;
};

void keyPressed() {
    if (InputBoxObj != null) {
        InputBoxObj.tKeyPressed();
    } else {
        print("\nReminder: You are not selecting any tile.\n");
        print("InputBoxObj is null, but key pressed.\n");
    }
};

void mousePressed() {
    for (int i = 0; i < tiles.size(); i++) {
        tiles.get(i).onClick();
    }
};

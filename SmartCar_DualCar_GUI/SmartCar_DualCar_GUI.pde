import java.nio.charset.StandardCharsets;
import javafx.stage.Screen;
import processing.serial.*;
import java.util.Random;

Random rand = new Random();
ArrayList<VectorGraphDot> VectorGraph_ArraryList = null;
ArrayList<CameraDot> CameraGraph_ArraryList = null;
InputBox InputBoxObj = null;
int TotalTrashedByte = 0;
ScanLineChart currentChart = null;

Button MoveLeft, MoveRight, MoveCenter, UARTstatus;

void setup() {
    // init windows
    size(400, 400);
    //fullScreen();
    frameRate(refreshRate);

    colorMode(RGB, 255, 255, 255);
    background(BACKGROUND_COLOR);
    noStroke();

    MoveLeft = new Button(0, 0, 60, 30, "<<");
    MoveCenter = new Button(0, 0, 90, 30, "HOME");
    MoveRight = new Button(0, 0, 60, 30, ">>");
    MoveLeft.setColor(WHITE, WHITE);
    MoveCenter.setColor(WHITE, WHITE);
    MoveRight.setColor(WHITE, WHITE);

    UARTstatus = new Button(0, 0, 300, 30, "null");
    UARTstatus.setColor(WHITE, WHITE);

    if (frame != null) {
        surface.setResizable(true);
    }

    // uart setup
    InitUART(0);

    VectorGraph_ArraryList = new ArrayList<VectorGraphDot>();
    for (int i = 0; i < 34; i++) {
        VectorGraph_ArraryList.add(new VectorGraphDot());
    }

    CameraGraph_ArraryList = new ArrayList<CameraDot>();
    for (int i = 0; i < CAMERA_GRAPH_X * CAMERA_GRAPH_Y / 8; i++) {
        CameraGraph_ArraryList.add(new CameraDot());
    }

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

    DataArray_bool = new boolean[bool_MailBox.MaxTerm.ordinal()+1];
    for (int i = 0; i < bool_MailBox.MaxTerm.ordinal(); i++) {
        DataArray_bool[i] = false;
    }

    init();
    if (currentChart != null) 
        currentChart.init();

    if (uart != null)
        uart.SendWrapper(DATA_TYPE.SYSTEM, SYSTEM_MSG.sayHi.ordinal(), (byte) 0, (byte) 0, false);
}

int pastWidth  = 0;
int pastHeight = 0;

void draw() {
    if (uart != null)
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

    MoveLeft.over();
    MoveRight.over();
    MoveCenter.over();
    UARTstatus.over();

    MoveLeft.tDraw();
    MoveRight.tDraw();
    MoveCenter.tDraw();
    UARTstatus.tDraw();

    if (UARTstatus.getValue().equals("Connecting")) {
        UARTstatus.setValue("Connecting.");
    } else if (UARTstatus.getValue().equals("Connecting.")) {
        if (uart != null) {
            uart.close();
            uart = null;
        }
        InitUART(0);
    }
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
    if (MoveLeft.isHovering) {
        borderX -= 200;

        clear();
        background(BACKGROUND_COLOR);
        tileAllocator();

        println("SYSTEM: MoveLeft pressed");
    } else if (MoveCenter.isHovering) {
        borderX = 40;

        clear();
        background(BACKGROUND_COLOR);
        tileAllocator();

        println("SYSTEM: MoveCenter pressed");
    } else if (MoveRight.isHovering) {
        borderX += 200;

        clear();
        background(BACKGROUND_COLOR);
        tileAllocator();

        println("SYSTEM: MoveRight pressed");
    } else if (UARTstatus.isHovering) {
        UARTstatus.setValue("Connecting");
    } else {
        for (int i = 0; i < tiles.size(); i++) {
            tiles.get(i).onClick();
        }
    }
};

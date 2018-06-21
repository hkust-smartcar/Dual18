import processing.core.*; 
import processing.data.*; 
import processing.event.*; 
import processing.opengl.*; 

import java.nio.charset.StandardCharsets; 
import javafx.stage.Screen; 
import processing.serial.*; 
import java.util.Random; 
import java.util.*; 
import java.nio.file.Files; 
import java.nio.file.Path; 
import java.nio.file.Paths; 
import java.nio.file.StandardOpenOption; 
import processing.serial.*; 
import java.util.*; 
import java.nio.ByteBuffer; 

import java.util.HashMap; 
import java.util.ArrayList; 
import java.io.File; 
import java.io.BufferedReader; 
import java.io.PrintWriter; 
import java.io.InputStream; 
import java.io.OutputStream; 
import java.io.IOException; 

public class SmartCar_DualCar_GUI extends PApplet {






Random rand = new Random();
ArrayList<VectorGraphDot> VectorGraph_ArraryList = null;
ArrayList<CameraDot> CameraGraph_ArraryList = null;
InputBox InputBoxObj = null;
int TotalTrashedByte = 0;
ScanLineChart currentChart = null;

Button MoveLeft, MoveRight, MoveCenter, UARTstatus;

public void setup() {
    // init windows
    
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
    
    println("Working dir is " + EnvPath);

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

public void draw() {
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

public void keyPressed() {
    if (InputBoxObj != null) {
        InputBoxObj.tKeyPressed();
    } else {
        print("\nReminder: You are not selecting any tile.\n");
        print("InputBoxObj is null, but key pressed.\n");
    }
};

public void mousePressed() {
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
// colors from from https://htmlcolorcodes.com
final int WHITE = color(255, 255, 255);
final int SILVER  = color(192, 192, 192);
final int GRAY  = color(128, 128, 128);
final int BLACK  = color(0, 0, 0);
final int RED  = color(255, 0, 0);
final int MAROON  = color(128, 0, 0);
final int YELLOW = color(255, 255, 0);
final int OLIVE = color(128, 128, 0);
final int LIME = color(0, 255, 0);
final int GREEN = color(0, 128, 0);
final int AQUA = color(0, 255, 255);
final int TEAL = color(0, 128, 128);
final int BLUE = color(0, 0, 255);
final int NAVY = color(0, 0, 128);
final int FUCHSIA = color(255, 0, 255);
final int PURPLE = color(128, 0, 128);

// colors:
final int BACKGROUND_COLOR = BLACK;
final int TILE_BACKGROUND_COLOR = WHITE;
final int TEXT_COLOR = BLACK;
final int BUTTON_HOVERING_COLOR = GRAY;
final int BUTTON_NORMAL_COLOR = SILVER;
final int CHART_CURSOR_COLOR = GRAY;

// below stuff is changeable for your personal color preference
final float refreshRate = 20; // per ms, i can't the reason to change it
final int RECT_ANGLE_RADIUS = 0; // button corners radius
final int CHART_HEIGHT = 600;
final int fontSize = 20;
final int LINE_STROKE_WEIGHT = 2;

final int CAMERA_GRAPH_X = 80;
final int CAMERA_GRAPH_Y = 60;
 

public enum DATA_TYPE {
    BUFFER, DOUBLE, UINT8_T, FLOAT, INT, MORRIS_VECTOR, CAMERA_IMG, SYSTEM, BOOLEAN;
}

public enum SYSTEM_MSG {
    ack, 
        sayHi, 
        elpasedTime, 
        vector_StartSend, 
        vector_EndSend, 
        camera_StartSend, 
        camera_EndSend, 
        lastTerm;
};

public int[] DataArray_uint8_t;
public double[] DataArray_double;
public float[] DataArray_float;
public int[] DataArray_int;
public boolean[] DataArray_bool;

final int LOCAL_BUFFER_MAX = 10;

int SYSTEM_MSG_elpasedTime = 0;

boolean GLOBAL_PAUSE = false;
int GLOBAL_RESTART = 0;
int GLOBAL_TXT_NO = 0;

public enum double_MailBox {
    d0, 
        d1, 
        d2, 
        d3, 
        d4, 
        d5, 
        d6, 
        d7, 
        d8, 
        d9, 
        d10, 
        d11, 
        d12, 
        d13, 
        d14, 
        d15, 
        d16, 
        d17, 
        d18, 
        d19, 
        d20, 
        d21, 
        d22, 
        d23, 
        d24, 
        d25, 
        d26, 
        d27, 
        d28, 
        d29, 
        d30, 
        MaxTerm // keep it as the last term
}

public enum float_MailBox {
    f0, 
        f1, 
        f2, 
        f3, 
        f4, 
        f5, 
        f6, 
        f7, 
        f8, 
        f9, 
        f10, 
        f11, 
        f12, 
        f13, 
        f14, 
        f15, 
        f16, 
        f17, 
        f18, 
        f19, 
        f20, 
        f21, 
        f22, 
        f23, 
        f24, 
        f25, 
        f26, 
        f27, 
        f28, 
        f29, 
        f30, 
        MaxTerm // keep it as the last term
}

public enum uint8_t_MailBox {
    // nah, max support for 32 terms so far, tell me if you want more la ^^
    BOOLEAN, 
        u0, 
        u1, 
        u2, 
        u3, 
        u4, 
        u5, 
        u6, 
        u7, 
        u8, 
        u9, 
        u10, 
        u11, 
        u12, 
        u13, 
        u14, 
        u15, 
        u16, 
        u17, 
        u18, 
        u19, 
        u20, 
        u21, 
        u22, 
        u23, 
        u24, 
        u25, 
        u26, 
        u27, 
        u28, 
        u29, 
        MaxTerm
}

public enum int_MailBox {
    // nah, max support for 32 terms so far, tell me if you want more la ^^
    i0, 
        i1, 
        i2, 
        i3, 
        i4, 
        i5, 
        i6, 
        i7, 
        i8, 
        i9, 
        i10, 
        i11, 
        i12, 
        i13, 
        i14, 
        i15, 
        i16, 
        i17, 
        i18, 
        i19, 
        i20, 
        i21, 
        i22, 
        i23, 
        i24, 
        i25, 
        i26, 
        i27, 
        i28, 
        i29, 
        i30, 
        MaxTerm
}

public enum bool_MailBox {
    // nah, max support for 32 terms so far, tell me if you want more la ^^
    b0, 
        b1, 
        b2, 
        b3, 
        b4, 
        b5, 
        b6, 
        b7, 
        b8, 
        b9, 
        b10, 
        b11, 
        b12, 
        b13, 
        b14, 
        b15, 
        b16, 
        b17, 
        b18, 
        b19, 
        b20, 
        b21, 
        b22, 
        b23, 
        b24, 
        b25, 
        b26, 
        b27, 
        b28, 
        b29, 
        b30, 
        MaxTerm
}

public enum Mailbox {
    d0, 
        d1, 
        d2, 
        d3, 
        d4, 
        d5, 
        d6, 
        d7, 
        d8, 
        d9, 
        d10, 
        d11, 
        d12, 
        d13, 
        d14, 
        d15, 
        d16, 
        d17, 
        d18, 
        d19, 
        d20, 
        d21, 
        d22, 
        d23, 
        d24, 
        d25, 
        d26, 
        d27, 
        d28, 
        d29, 
        d30, 
        d_MaxTerm, 

        f0, 
        f1, 
        f2, 
        f3, 
        f4, 
        f5, 
        f6, 
        f7, 
        f8, 
        f9, 
        f10, 
        f11, 
        f12, 
        f13, 
        f14, 
        f15, 
        f16, 
        f17, 
        f18, 
        f19, 
        f20, 
        f21, 
        f22, 
        f23, 
        f24, 
        f25, 
        f26, 
        f27, 
        f28, 
        f29, 
        f30, 
        f_MaxTerm, 

        BOOLEAN, 
        u0, 
        u1, 
        u2, 
        u3, 
        u4, 
        u5, 
        u6, 
        u7, 
        u8, 
        u9, 
        u10, 
        u11, 
        u12, 
        u13, 
        u14, 
        u15, 
        u16, 
        u17, 
        u18, 
        u19, 
        u20, 
        u21, 
        u22, 
        u23, 
        u24, 
        u25, 
        u26, 
        u27, 
        u28, 
        u29, 
        u_MaxTerm, 

        i0, 
        i1, 
        i2, 
        i3, 
        i4, 
        i5, 
        i6, 
        i7, 
        i8, 
        i9, 
        i10, 
        i11, 
        i12, 
        i13, 
        i14, 
        i15, 
        i16, 
        i17, 
        i18, 
        i19, 
        i20, 
        i21, 
        i22, 
        i23, 
        i24, 
        i25, 
        i26, 
        i27, 
        i28, 
        i29, 
        i30, 
        i_MaxTerm, 

        b0, 
        b1, 
        b2, 
        b3, 
        b4, 
        b5, 
        b6, 
        b7, 
        b8, 
        b9, 
        b10, 
        b11, 
        b12, 
        b13, 
        b14, 
        b15, 
        b16, 
        b17, 
        b18, 
        b19, 
        b20, 
        b21, 
        b22, 
        b23, 
        b24, 
        b25, 
        b26, 
        b27, 
        b28, 
        b29, 
        b30, 
        b_MaxTerm
}

// addOutputValueTile

public void addOutputValueTile(Mailbox mailbox, String name) {
    addGeneralVariableTile(mailbox, name, 0);
}

public void addInputIncDecTile(Mailbox mailbox, String name, double initValue) {
    addGeneralVariableTile(mailbox, name, 0);
}

// GeneralVariableTile
public void addGeneralVariableTile(Mailbox mailbox, String name, double initValue) {
    if (mailbox.ordinal() < 32) {
        addGeneralVariableTile(double_MailBox.values()[mailbox.ordinal()], name, (double) initValue);
    } else if (mailbox.ordinal() < 64) {
        addGeneralVariableTile(float_MailBox.values()[mailbox.ordinal() - 32], name, (float) initValue);
    } else if (mailbox.ordinal() < 96) {
        addGeneralVariableTile(uint8_t_MailBox.values()[mailbox.ordinal() - 64], name, (int) initValue);
    } else if (mailbox.ordinal() < 128) {
        addGeneralVariableTile(int_MailBox.values()[mailbox.ordinal() - 96], name, (int) initValue);
    } else if (mailbox.ordinal() < 160) {
        addGeneralVariableTile(bool_MailBox.values()[mailbox.ordinal() - 128], name, !(initValue == 0));
    }
}

public void addGeneralVariableTile(int_MailBox mailbox, String name, int initValue) {
    GeneralVariableTile t = new GeneralVariableTile(mailbox, initValue);
    t.setValue(name);
    tiles.add(t);
}

public void addGeneralVariableTile(uint8_t_MailBox mailbox, String name, int initValue) {

    // !!! missing range check here

    GeneralVariableTile t = new GeneralVariableTile(mailbox, initValue);
    t.setValue(name);
    tiles.add(t);
}

public void addGeneralVariableTile(double_MailBox mailbox, String name, double initValue) {
    GeneralVariableTile t = new GeneralVariableTile(mailbox, initValue);
    t.setValue(name);
    tiles.add(t);
}

public void addGeneralVariableTile(float_MailBox mailbox, String name, float initValue) {
    GeneralVariableTile t = new GeneralVariableTile(mailbox, initValue);
    t.setValue(name);
    tiles.add(t);
}

public void addGeneralVariableTile(bool_MailBox mailbox, String name, boolean initValue) {
    GeneralVariableTile t = new GeneralVariableTile(mailbox, initValue);
    t.setValue(name);
    tiles.add(t);
}

// addElapsedTime
public void addElapsedTime() {
    tiles.add(new ElapsedTime());
}

// addChart
public void addChart(int SampleSize_, int HorizontalSpacing, int chart_height, double LowerBound, double UpperBound) {
    if (currentChart != null) 
        currentChart.init();
    currentChart = new ScanLineChart(SampleSize_, HorizontalSpacing, chart_height, LowerBound, UpperBound);
    tiles.add(currentChart);
}

// addLine
public void addLine(Mailbox mailbox, String name, int c) {
    if (mailbox.ordinal() < 32) {
        addLine(double_MailBox.values()[mailbox.ordinal()], name, c);
    } else if (mailbox.ordinal() < 64) {
        addLine(float_MailBox.values()[mailbox.ordinal() - 32], name, c);
    } else if (mailbox.ordinal() < 96) {
        addLine(uint8_t_MailBox.values()[mailbox.ordinal() - 64], name, c);
    } else if (mailbox.ordinal() < 128) {
        addLine(uint8_t_MailBox.values()[mailbox.ordinal() - 96], name, c);
    }
}

public void addLine(float_MailBox mailbox, String name, int c) {
    if (currentChart != null) {
        ScanLineChart_Line t = new ScanLineChart_Line(mailbox, c);
        t.setValue(name);
        currentChart.lines.add(t);
    }
}

public void addLine(uint8_t_MailBox mailbox, String name, int c) {
    if (currentChart != null) {
        ScanLineChart_Line t = new ScanLineChart_Line(mailbox, c);
        t.setValue(name);
        currentChart.lines.add(t);
    }
}

public void addLine(int_MailBox mailbox, String name, int c) {
    if (currentChart != null) {
        ScanLineChart_Line t = new ScanLineChart_Line(mailbox, c);
        t.setValue(name);
        currentChart.lines.add(t);
    }
}

public void addLine(double_MailBox mailbox, String name, int c) {
    if (currentChart != null) {
        ScanLineChart_Line t = new ScanLineChart_Line(mailbox, c);
        t.setValue(name);
        currentChart.lines.add(t);
    }
}

public void addLine(double value, String name, int c) {
    if (currentChart != null) {
        ScanLineChart_Line t = new ScanLineChart_Line(value, c);
        t.setValue(name);
        currentChart.lines.add(t);
    }
}

// addLineBreak
public void addLineBreak() {
    tiles.add(new BreakColumn(false));
}

public void addCameraGraph() {
    tiles.add(new CameraGraph());
}

public void addButtonTile(Mailbox mailbox, String name) {
    if (mailbox.ordinal() < 160 && mailbox.ordinal() >= 128) {
        addButtonTile(bool_MailBox.values()[mailbox.ordinal() - 128], name);
    } else {
        println("GUI: Button Init Error!");
    }
}

public void addButtonTile(bool_MailBox mailbox, String name) {
    ButtonTile t = new ButtonTile(mailbox);
    t.setValue(name);
    tiles.add(t);
}
public interface GUI_interface {
    int m_TopLeftX = 0, m_TopLeftY = 0;
    int m_Height = 0, m_Width = 0;

    public void tDraw();
    public void over();
    public void onClick();
    public void setValue(String value_);
    public void setValue(double double_);
    public void setValue(float float_);
    public void setValue(int int_);
    public void setValue(boolean bool_);
    public void serialSend();
    public void setBackgroundFill(boolean needBackground);
    public void setTextColor(int TextColor_);
    public void tKeyPressed();
    public String getPos();

    public int getHeight();
    public int getWidth();
    public String getValue();
    public void setPos(int topLeftX_, int topLeftY_);
}

public class GUI_Raw implements GUI_interface {
    public int m_TopLeftX = 0, m_TopLeftY = 0;
    public int m_Height = 0, m_Width = 0;
    public boolean m_NeedBackground = false;
    public int m_TextColor = TEXT_COLOR;
    public boolean isHovering = false;

    GUI_Raw() {
    };
    public void tDraw() {
    };
    public void over() {
        if (mouseX >= m_TopLeftX && mouseX <= (m_TopLeftX+m_Width) && 
            mouseY >= m_TopLeftY && mouseY <= (m_TopLeftY+m_Height)) {
            isHovering = true;
        } else {
            isHovering = false;
        }
    };
    public void onClick() {
    };
    public void setValue(String value_) {
    };
    public void setValue(double double_) {
    };
    public void setValue(float float_) {
    };
    public void setValue(int int_) {
    };
    public void setValue(boolean bool_) {
    };
    public void setBackgroundFill(boolean needBackground) {
        m_NeedBackground = needBackground;
    };
    public int getHeight() {
        return m_Height;
    };
    public int getWidth() {
        return m_Width;
    };
    public String getValue() {
        return "";
    };
    public void setPos(int topLeftX_, int topLeftY_) {
        m_TopLeftX = topLeftX_;
        m_TopLeftY = topLeftY_;
    };
    public void serialSend() {
    };
    public void setTextColor(int TextColor_) {
        m_TextColor = TextColor_;
    };
    public void tKeyPressed() {
    };
    public String getPos() {
        return Integer.toString(m_TopLeftX) + "," + Integer.toString(m_TopLeftY);
    };
}

ArrayList<GUI_interface> tiles = new ArrayList<GUI_interface>();

int borderX = 40;
final int borderY = 40;
final int borderYLower = 40;
final int hSpacing = 20;
final int wSpacing = 20;

public void tileAllocator() {
    int currentX = borderX;
    int currentY = borderY;

    int maxX = 0;

    for (int i = 0; i < tiles.size(); i++) {
        if ((tiles.get(i).getHeight() + currentY) > (height - borderY - borderYLower)) {
            currentX += maxX + wSpacing;
            currentY = borderY;
            maxX = 0;
        }

        tiles.get(i).setPos(currentX, currentY);
        currentY += tiles.get(i).getHeight() + hSpacing;

        maxX = (maxX < tiles.get(i).getWidth()) ? tiles.get(i).getWidth() : maxX;
    }

    MoveLeft.setPos(40, height - borderY - borderYLower+40);
    MoveCenter.setPos(40+60+20, height - borderY - borderYLower+40);
    MoveRight.setPos(40+60+20+20+90, height - borderY - borderYLower+40);

    UARTstatus.setPos(40+60+20+20+90+60+40, height - borderY - borderYLower+40);
    if (InputBoxObj != null)
        InputBoxObj.setPos(0, 0);

    print("\nGUI: tileAllocator \n");
}
class BreakColumn extends GUI_Raw {
    BreakColumn(boolean needBG) {
        super();
        m_Height = 100000000;
        m_Width = 20;
        m_NeedBackground = needBG;
    }

    public @Override void tDraw() {
        noStroke();

        if (m_NeedBackground == true) {
            fill(TILE_BACKGROUND_COLOR);
            rect(m_TopLeftX, m_TopLeftY, m_Width, m_Height, RECT_ANGLE_RADIUS);
        }
    }
}

class TextLabel extends GUI_Raw {
    private final int label_X_Indent = 10;
    private String label = "Empty";
    private boolean m_NeedBackground = true;

    TextLabel(int width_, String label_) {
        super();
        textSize(fontSize);
        m_Height = (int) textAscent() + (int) textDescent();
        m_Width = width_;
        label = label_;
    }

    public @Override void tDraw() {
        textAlign(LEFT);

        noStroke();

        if (m_NeedBackground == true) {
            fill(TILE_BACKGROUND_COLOR);
            rect(m_TopLeftX, m_TopLeftY, m_Width, m_Height, RECT_ANGLE_RADIUS);
        }

        fill(m_TextColor);
        textSize(fontSize);

        if (label == null) {
            label = "TextLabel - error - null";
        }

        text(label, m_TopLeftX + label_X_Indent, m_TopLeftY, m_Width - fontSize, m_Height);
    }
    public @Override void setValue(String value_) {
        label = value_;
    };
    public @Override void setValue(double double_) {
        label = Double.toString(double_);
    };
    public @Override void setValue(float float_) {
        label = Float.toString(float_);
    };
    public @Override void setValue(int int_) {
        label = Integer.toString(int_);
    };
    public @Override String getValue() {
        return label;
    };
}

class Button extends GUI_Raw {
    private String text;
    private int Hovering_Color = BUTTON_HOVERING_COLOR;
    private int Normal_Color = BUTTON_NORMAL_COLOR;

    Button(String text_) {
        super();
        m_Width = (int) textWidth(text_+" ");
        textSize(fontSize);
        m_Height = (int) textAscent() + (int) textDescent();
        text = text_;
        if (text == null) {
            text = "null";
        }
    }
    Button(int topLeftX_, int topLeftY_, String text_) {
        super();
        m_TopLeftX = topLeftX_;
        m_TopLeftY = topLeftY_;
        m_Width = (int) textWidth(text_+" ");
        textSize(fontSize);
        m_Height = (int) textAscent() + (int) textDescent();
        text = text_;
        if (text == null) {
            text = "null";
        }
    }
    Button(int topLeftX_, int topLeftY_, int width_, int height_, String text_) {
        super();
        m_TopLeftX = topLeftX_;
        m_TopLeftY = topLeftY_;
        m_Width = width_;
        m_Height = height_;
        text = text_;
        if (text == null) {
            text = "null";
        }
    }
    public @Override void tDraw() {
        // this might casue extra resouces consumption
        if (isHovering) {
            fill(Hovering_Color);
        } else {
            fill(Normal_Color);
        }

        textAlign(CENTER);

        noStroke();
        rect(m_TopLeftX, m_TopLeftY, m_Width, m_Height, RECT_ANGLE_RADIUS);

        fill(m_TextColor);
        textSize(fontSize);
        text(text, m_TopLeftX, m_TopLeftY, m_Width, m_Height);
    }
    public @Override void setValue(String value_) {
        text = value_;
        if (text == null) {
            text = "null";
        }
    };
    public @Override void setValue(double double_) {
        text = Double.toString(double_);
    };
    public @Override void setValue(float float_) {
        text = Float.toString(float_);
    };
    public @Override void setValue(int int_) {
        text = Integer.toString(int_);
    };
    public @Override String getValue() {
        return text;
    };

    public void setColor(int h_, int n_) {
        Hovering_Color = h_;
        Normal_Color = n_;
    }
}

class GeneralVariableTile extends GUI_Raw {
    private DATA_TYPE m_DataType;
    private uint8_t_MailBox uint8_t_MailBox_id = null;
    private double_MailBox double_MailBox_id = null;
    private float_MailBox float_MailBox_id = null;
    private int_MailBox int_MailBox_id = null;
    private bool_MailBox bool_MailBox_id = null;

    private TextLabel m_DataTypeLabel, m_ValueLabel;

    private void init() {
        m_DataTypeLabel.setBackgroundFill(false);
        m_ValueLabel.setBackgroundFill(false);
        m_Height = m_ValueLabel.getHeight() + m_DataTypeLabel.getHeight();
    }
    GeneralVariableTile(uint8_t_MailBox mail_id_, int initValue) {
        m_Width = 200;

        m_DataType = DATA_TYPE.UINT8_T;
        uint8_t_MailBox_id = mail_id_;

        DataArray_uint8_t[uint8_t_MailBox_id.ordinal()] = initValue;

        m_DataTypeLabel = new TextLabel(m_Width, m_DataType.name() + ": " + uint8_t_MailBox_id.name());
        m_ValueLabel = new TextLabel(m_Width, Integer.toString(DataArray_uint8_t[uint8_t_MailBox_id.ordinal()]));

        init();
    }
    GeneralVariableTile(int_MailBox mail_id_, int initValue) {
        m_Width = 200;

        m_DataType = DATA_TYPE.INT;
        int_MailBox_id = mail_id_;

        DataArray_int[int_MailBox_id.ordinal()] = initValue;

        m_DataTypeLabel = new TextLabel(m_Width, m_DataType.name() + ": " + int_MailBox_id.name());
        m_ValueLabel = new TextLabel(m_Width, Integer.toString(DataArray_int[int_MailBox_id.ordinal()]));

        init();
    }
    GeneralVariableTile(double_MailBox mail_id_, double initValue) {
        m_Width = 200;

        m_DataType = DATA_TYPE.DOUBLE;
        double_MailBox_id = mail_id_;

        DataArray_double[double_MailBox_id.ordinal()] = initValue;

        m_DataTypeLabel = new TextLabel(m_Width, m_DataType.name() + ": " + double_MailBox_id.name());
        m_ValueLabel = new TextLabel(m_Width, Double.toString(DataArray_double[double_MailBox_id.ordinal()]));

        init();
    }
    GeneralVariableTile(float_MailBox mail_id_, float initValue) {
        m_Width = 200;

        m_DataType = DATA_TYPE.FLOAT;
        float_MailBox_id = mail_id_;

        DataArray_float[float_MailBox_id.ordinal()] = initValue;

        m_DataTypeLabel = new TextLabel(m_Width, m_DataType.name() + ": " + float_MailBox_id.name());
        m_ValueLabel = new TextLabel(m_Width, Float.toString(DataArray_float[float_MailBox_id.ordinal()]));

        init();
    }
    GeneralVariableTile(bool_MailBox mail_id_, boolean initValue) {
        m_Width = 200;

        m_DataType = DATA_TYPE.BOOLEAN;
        bool_MailBox_id = mail_id_;

        DataArray_bool[bool_MailBox_id.ordinal()] = initValue;

        m_DataTypeLabel = new TextLabel(m_Width, m_DataType.name() + ": " + bool_MailBox_id.name());
        m_ValueLabel = new TextLabel(m_Width, DataArray_bool[bool_MailBox_id.ordinal()] ? "True" : "False");

        init();
    }
    public @Override void tDraw() {
        if (m_DataType == DATA_TYPE.UINT8_T) {
            m_ValueLabel.setValue(Integer.toString(DataArray_uint8_t[uint8_t_MailBox_id.ordinal()]));
        } else if (m_DataType == DATA_TYPE.DOUBLE) {
            m_ValueLabel.setValue(Double.toString(DataArray_double[double_MailBox_id.ordinal()]));
        } else if (m_DataType == DATA_TYPE.FLOAT) {
            m_ValueLabel.setValue(Float.toString(DataArray_float[float_MailBox_id.ordinal()]));
        } else if (m_DataType == DATA_TYPE.INT) {
            m_ValueLabel.setValue(Integer.toString(DataArray_int[int_MailBox_id.ordinal()]));
        } else if (m_DataType == DATA_TYPE.BOOLEAN) {
            m_ValueLabel.setValue(DataArray_bool[bool_MailBox_id.ordinal()] ? "True" : "False");
        }

        fill(TILE_BACKGROUND_COLOR);
        rect(m_TopLeftX, m_TopLeftY, m_Width, m_Height, RECT_ANGLE_RADIUS);

        m_DataTypeLabel.tDraw();
        m_ValueLabel.tDraw();
    }
    public @Override void onClick() {
        if (isHovering) {
            if (InputBoxObj == null) {
                InputBoxObj = new InputBox();
            }

            if (m_DataType == DATA_TYPE.DOUBLE) {
                InputBoxObj.setValue(double_MailBox_id.ordinal());
            } else if (m_DataType == DATA_TYPE.FLOAT) {
                InputBoxObj.setValue(float_MailBox_id.ordinal()+32);
            } else if (m_DataType == DATA_TYPE.UINT8_T) {
                InputBoxObj.setValue(uint8_t_MailBox_id.ordinal()+64);
            } else if (m_DataType == DATA_TYPE.INT) {
                InputBoxObj.setValue(int_MailBox_id.ordinal()+96);
            } else if (m_DataType == DATA_TYPE.BOOLEAN) {
                InputBoxObj.setValue(bool_MailBox_id.ordinal()+128);
            }
            InputBoxObj.setValue(m_ValueLabel.getPos());
            InputBoxObj.reg(this);
        }
    }
    public @Override void setPos(int topLeftX_, int topLeftY_) {
        m_TopLeftX = topLeftX_;
        m_TopLeftY = topLeftY_;

        m_DataTypeLabel.setPos(topLeftX_, m_TopLeftY);
        m_ValueLabel.setPos(m_TopLeftX, m_TopLeftY + m_DataTypeLabel.getHeight());
    }
    public @Override void setValue(String value_) {
        m_DataTypeLabel.setValue(value_);
    };
    public @Override void setTextColor(int TextColor_) {
        m_TextColor = TextColor_;

        m_DataTypeLabel.setTextColor(m_TextColor);
        m_ValueLabel.setTextColor(m_TextColor);
    };
}

class ButtonTile extends GUI_Raw {
    private final int m_ButHeight = 25;
    private Button m_but;
    private TextLabel m_var;
    private bool_MailBox bool_MailBox_id = null;
    private int deBounce;
    private int AniSpeed = 20;
    private int AniTarget = 0;
    private int AniNowX = 0;
    private boolean AniLastValue;

    ButtonTile(bool_MailBox id) {
        bool_MailBox_id = id;
        m_Width = 200;

        AniLastValue = DataArray_bool[bool_MailBox_id.ordinal()];

        m_but = new Button(0, 0, (m_Width - 40), m_ButHeight, "");
        m_var = new TextLabel(m_Width, "null");

        if (DataArray_bool[bool_MailBox_id.ordinal()]) {
            AniNowX = m_TopLeftX + 20;
            AniTarget = AniNowX;
        } else {
            AniNowX = m_TopLeftX + 20 + (m_Width - 40)/2;
            AniTarget = AniNowX;
        }

        m_Height = m_but.getHeight() + m_var.getHeight() + 10;
        deBounce = 0;
    }

    public @Override 
        void tDraw() {
        deBounce = deBounce == 0 ? 0 : deBounce-1;

        fill(TILE_BACKGROUND_COLOR);
        rect(m_TopLeftX, m_TopLeftY, m_Width, m_Height, RECT_ANGLE_RADIUS);

        if (AniLastValue != DataArray_bool[bool_MailBox_id.ordinal()]) {
            if (DataArray_bool[bool_MailBox_id.ordinal()]) {
                AniTarget = m_TopLeftX + 20;
            } else {
                AniTarget = m_TopLeftX + 20 + (m_Width - 40)/2;
            }

            AniLastValue = DataArray_bool[bool_MailBox_id.ordinal()];
        }

        if (AniNowX < AniTarget) {
            AniNowX += AniSpeed;
        } else if (AniNowX > AniTarget) {
            AniNowX -= AniSpeed;
        }

        fill(BLACK);
        rect(m_TopLeftX + 20-5, m_TopLeftY+m_var.getHeight()+5-5, (m_Width - 40)+10, m_ButHeight+10, RECT_ANGLE_RADIUS);

        if (DataArray_bool[bool_MailBox_id.ordinal()]) {
            m_but.setColor(BLUE, BLUE);
        } else {
            m_but.setColor(GRAY, GRAY);
        }
        m_but.tDraw();

        fill(WHITE);
        rect(AniNowX, m_TopLeftY+m_var.getHeight()+5, (m_Width - 40)/2, m_ButHeight, RECT_ANGLE_RADIUS);

        m_var.tDraw();
    }
    public @Override
        void onClick() {
        if (m_but.isHovering && deBounce == 0) {
            DataArray_bool[bool_MailBox_id.ordinal()] = !DataArray_bool[bool_MailBox_id.ordinal()];
            serialSend();

            deBounce = 4;
        }
    }
    public @Override
        void over() {
        m_but.over();
    }
    public @Override
        void setValue(String name) {
        m_var.setValue(name);
    }
    public @Override
        void setPos(int topLeftX_, int topLeftY_) {
        m_TopLeftX = topLeftX_;
        m_TopLeftY = topLeftY_;

        m_var.setPos(m_TopLeftX, m_TopLeftY);
        m_but.setPos(m_TopLeftX + 20, m_TopLeftY+m_var.getHeight()+5);

        if (DataArray_bool[bool_MailBox_id.ordinal()]) {
            AniNowX = m_TopLeftX + 20;
            AniTarget = AniNowX;
        } else {
            AniNowX = m_TopLeftX + 20 + (m_Width - 40)/2;
            AniTarget = AniNowX;
        }
    }

    public @Override
        void serialSend() {
        if (uart != null)
            uart.Send_bool(bool_MailBox_id, DataArray_bool[bool_MailBox_id.ordinal()]);
    };
}
class CameraDot extends GUI_Raw {
    private Boolean isDirty;
    private Boolean isDirty_n;
    private int ptColor;
    private int scale;
    private byte content;
    private byte content_n;

    CameraDot() {
        super();

        m_Width = 1;
        m_Height = 1;

        isDirty = false;
        isDirty_n = false;

        content = 0;
        content_n = 0;

        scale = 1;

        ptColor = BLACK;
    }

    public void setClean() {
        isDirty = false;
    }

    public void update() {
        isDirty = isDirty_n;
        content= content;
    }

    public void setData(byte d_) {
        isDirty_n = true;
        content_n = d_;
    }

    public void setScale(int _scale) {
        scale = _scale;
    }

    public @Override
        void tDraw() {
        if (isDirty) {
            noStroke();
            fill(ptColor);
            for (int i = 0; i < 8; i++) {
                if (((content >> i) & 1) == 1) {
                    rect(m_TopLeftX + i * scale, m_TopLeftY, scale, scale);
                }
            }
        }
    }
}

class VectorGraphDot extends GUI_Raw {
    private Boolean isDirty;
    private Boolean isDirty_n;
    private int xCor, yCor;
    private int xCor_n, yCor_n;
    private int ptColor;
    private int scale;

    VectorGraphDot() {
        super();

        m_Width = 1;
        m_Height = 1;

        isDirty = false;
        isDirty_n = false;
        xCor = 0;
        yCor = 0;
        xCor_n = 0;
        yCor_n = 0;

        scale = 1;

        ptColor = RED;
    }

    public void setClean() {
        isDirty = false;
    }

    public void update() {
        isDirty = isDirty_n;
        xCor= xCor_n;
        yCor= yCor_n;
    }

    public void setCor(int _x, int _y) {
        isDirty_n = true;
        xCor_n = _x;
        yCor_n = _y;
    }

    public void setScale(int _scale) {
        scale = _scale;
    }

    public @Override
        void tDraw() {
        if (isDirty) {
            noStroke();
            fill(ptColor);
            rect(m_TopLeftX + xCor * scale, m_TopLeftY + yCor * scale, scale, scale);
        } else {
        }
    }
}

class CameraGraph extends GUI_Raw {
    private int borderWidth = 20;
    private int scale = 7;
    private int GraphBG;

    CameraGraph() {
        super();
        m_Width = CAMERA_GRAPH_X * scale + borderWidth*2;
        m_Height = CAMERA_GRAPH_Y*scale + borderWidth*2;

        GraphBG = GRAY;

        if (VectorGraph_ArraryList != null) {
            int len = VectorGraph_ArraryList.size();
            for (int i = 0; i < len; i++) {
                VectorGraph_ArraryList.get(i).setScale(scale);
            }
        } else {
            println("GUI: Critical Error! VectorGraph_ArraryList is null!");
        }

        //if (CameraGraph_ArraryList != null) {
        //    int len = CameraGraph_ArraryList.size();
        //    for (int i = 0; i < len; i++) {
        //        CameraGraph_ArraryList.get(i).setScale(scale);
        //    }
        //} else {
        //    println("GUI: CameraGraph-CameraGraph_ArraryList is null!");
        //}
    }

    public @Override
        void tDraw() {
        noStroke();

        fill(TILE_BACKGROUND_COLOR);
        rect(m_TopLeftX, m_TopLeftY, m_Width, m_Height, RECT_ANGLE_RADIUS);

        fill(GraphBG);
        rect(m_TopLeftX + borderWidth, m_TopLeftY + borderWidth, m_Width - borderWidth*2, m_Height - borderWidth*2, RECT_ANGLE_RADIUS);

        //if (CameraGraph_ArraryList != null) {
        //    int len = CameraGraph_ArraryList.size();
        //    for (int i = 0; i < len; i++) {
        //        CameraGraph_ArraryList.get(i).tDraw();
        //    }
        //} else {
        //    println("GUI: CameraGraph-CameraGraph_ArraryList is null!");
        //}

        if (VectorGraph_ArraryList != null) {
            int len = VectorGraph_ArraryList.size();
            for (int i = 0; i < len; i++) {
                VectorGraph_ArraryList.get(i).tDraw();
            }
        } else {
            println("GUI: CameraGraph-VectorGraph_ArraryList is null!");
        }
    }

    public @Override
        void setPos(int topLeftX_, int topLeftY_) {
        m_TopLeftX = topLeftX_;
        m_TopLeftY = topLeftY_;

        if (VectorGraph_ArraryList != null) {
            println("GUI: VectorGraph-setPos!");

            int len = VectorGraph_ArraryList.size();
            for (int i = 0; i < len; i++) {
                VectorGraph_ArraryList.get(i).setPos(m_TopLeftX + borderWidth, m_TopLeftY + borderWidth);
            }
        } else {
            println("GUI: Critical Error! VectorGraph_ArraryList is null!");
        }

        //if (CameraGraph_ArraryList != null) {
        //    int len = CameraGraph_ArraryList.size();
        //    for (int x = 0; x < CAMERA_GRAPH_X/8; x++) {
        //        for (int y = 0; y < CAMERA_GRAPH_Y; y++) {
        //            println("x : " + x + ", y: " + y);
        //            CameraGraph_ArraryList.get(CAMERA_GRAPH_X*y/8 + x).
        //                setPos(m_TopLeftX + borderWidth + x*scale*8, 
        //                m_TopLeftY + borderWidth + y * scale);
        //        }
        //    }
        //} else {
        //    println("GUI: CameraGraph-CameraGraph_ArraryList is null!");
        //}
    };
}





interface ScanLineChart_Interface extends GUI_interface {
    public void setBoundary(double upperBound, double lowerBound);
    public void setHorizontalSpacing(int spacing);
    public void setSize(int width_, int height_);
    public void getNewValue();
    public void init(int SampleNumber_);

    public void drawClean();
    public void drawNew();

    public String getLineName();
    public int getLineColor();
    public int getCursor();
    public void setCursor(int cursor);
    public void setPause(Boolean m_isPause);
};

class ScanLineChart implements GUI_interface {

    private int m_TopLeftX = 0, m_TopLeftY = 0;
    private int m_Height = CHART_HEIGHT, m_Width = 400;
    private final int m_AxisSpacing = 150;
    private int m_SampleNumber = 100;
    private int m_HorizontalSpacing = 1;
    private double m_LowerBound = 0, m_UpperBound = 1;
    private ArrayList<ScanLineChart_Interface> lines = new ArrayList<ScanLineChart_Interface>();
    private ArrayList<TextLabel> linesLabel = new ArrayList<TextLabel>();
    private boolean needRedraw = true;
    private TextLabel m_UpperBoundLabel, m_LowerBoundLabel;
    private int m_TextColor = TEXT_COLOR;
    private PShape cursorShape;
    private int cursorShapeWidth = 0;

    private Button m_PauseBut;
    private Button m_RestartBut;

    private String m_PauseBut_PauseStr =  "G: Pause   ";
    private String m_PauseBut_ResumeStr = "G: Resume  ";
    private Boolean m_isPause = false;

    private String m_RestartBut_Restart = "G: Restart ";

    String FileToAppend = "test";

    ScanLineChart(int SampleSize_, int HorizontalSpacing, int chart_height, double LowerBound, double UpperBound) {
        m_SampleNumber = SampleSize_ > 0 ? SampleSize_ : m_SampleNumber;
        m_HorizontalSpacing = HorizontalSpacing > 0 ? HorizontalSpacing : m_HorizontalSpacing;
        m_LowerBound = LowerBound;
        m_UpperBound = UpperBound;

        m_UpperBoundLabel = new TextLabel(m_AxisSpacing, Double.toString(UpperBound));
        m_LowerBoundLabel = new TextLabel(m_AxisSpacing, Double.toString(LowerBound));
        m_UpperBoundLabel.setBackgroundFill(false);
        m_LowerBoundLabel.setBackgroundFill(false);

        m_Width = m_AxisSpacing + m_HorizontalSpacing * SampleSize_;
        needRedraw = true;

        cursorShape = createShape(TRIANGLE, 8, 0, 0, 16, 16, 16);
        cursorShape.setStroke(false);
        cursorShapeWidth = 16;

        m_PauseBut = new Button(m_PauseBut_PauseStr);
        m_RestartBut = new Button(m_RestartBut_Restart);

        m_Height = chart_height;
        m_Height = m_Height < 150 ? CHART_HEIGHT: m_Height;
    };

    public void tDraw() {

        if (GLOBAL_RESTART > 0) {
            for (int i = 0; i < lines.size(); i++) {
                lines.get(i).setCursor(0);
            }
            needRedraw = true;
        }

        if (GLOBAL_RESTART == 1) {
            String header = "Ticks";
            for (int i = 0; i < lines.size(); i++) {
                header += "," + lines.get(i).getLineName();
            }

            File file = new File(FileToAppend);
            file.delete();
            appendToFile(header);
        }

        if (GLOBAL_PAUSE == true && m_isPause == false) {

            m_PauseBut.setValue(m_PauseBut_ResumeStr);
            for (int i = 0; i < lines.size(); i++) {
                lines.get(i).setPause(true);
            }
            m_isPause = true;
        } else if (GLOBAL_PAUSE == false && m_isPause == true) {
            m_PauseBut.setValue(m_PauseBut_PauseStr);
            for (int i = 0; i < lines.size(); i++) {
                lines.get(i).setPause(false);
            }
            m_isPause = false;
        }

        if (needRedraw == true) {
            fill(TILE_BACKGROUND_COLOR);
            rect(m_TopLeftX, m_TopLeftY, m_Width, m_Height, RECT_ANGLE_RADIUS);
            needRedraw = false;

            fill(BACKGROUND_COLOR);
            rect(m_TopLeftX + m_AxisSpacing, m_TopLeftY, 3, m_Height);

            textAlign(RIGHT);
            m_UpperBoundLabel.tDraw();
            m_LowerBoundLabel.tDraw();
            textAlign(LEFT);

            for (int i = 0; i < lines.size(); i++) {
                linesLabel.get(i).tDraw();
            }
        }
        if (lines.size() != 0 && m_isPause == false) {
            cursorShape.setVisible(true);

            float x = (lines.get(0).getCursor() == 0) ? m_SampleNumber-1 : lines.get(0).getCursor()-1;
            x = m_AxisSpacing + m_TopLeftX + m_HorizontalSpacing * x + 2 - cursorShapeWidth/2;
            cursorShape.setFill(BACKGROUND_COLOR);
            shape(cursorShape, x, m_TopLeftY + m_Height);

            cursorShape.setFill(CHART_CURSOR_COLOR);
            shape(cursorShape, m_AxisSpacing + m_TopLeftX + m_HorizontalSpacing * lines.get(0).getCursor() + 2 - cursorShapeWidth/2, m_TopLeftY + m_Height);

            // save values
            String line = Integer.toString(SYSTEM_MSG_elpasedTime);
            for (int i = 0; i < lines.size(); i++) {
                line += "," + lines.get(i).getValue();
            }
            appendToFile(line);
        } else {
            cursorShape.setVisible(false);
        }
        for (int i = 0; i < lines.size(); i++) {
            lines.get(i).getNewValue();
            lines.get(i).tDraw();
            lines.get(i).drawClean();
        }
        for (int i = 0; i < lines.size(); i++) {
            lines.get(i).drawNew();
        }
        m_PauseBut.tDraw();
        m_RestartBut.tDraw();
    };
    public void over() {
        m_PauseBut.over();
        m_RestartBut.over();
    };
    public void onClick() {
        m_PauseBut.onClick();
        m_RestartBut.onClick();

        if (m_RestartBut.isHovering == true) {
            GLOBAL_RESTART = 2;
            for (int i = 0; i < lines.size(); i++) {
                lines.get(i).setCursor(0);
            }
            needRedraw = true;
        }

        if (m_PauseBut.isHovering == true) {
            if (m_PauseBut.getValue().equals(m_PauseBut_PauseStr)) {
                GLOBAL_PAUSE = true;
            } else {
                GLOBAL_PAUSE = false;
            }
        }
    };
    public void setValue(String value_) {
    };
    public void setValue(double double_) {
    };
    public void setValue(float float_) {
    };
    public void setValue(int int_) {
    };
    public void setValue(boolean bool_) {
    };
    public void serialSend() {
    };
    public void setBackgroundFill(boolean needBackground) {
    };
    public int getHeight() {
        return m_Height;
    };
    public int getWidth() {
        return m_Width;
    };
    public String getValue() {
        return "";
    };
    public void setPos(int topLeftX_, int topLeftY_) {
        m_TopLeftX = topLeftX_;
        m_TopLeftY = topLeftY_;

        m_UpperBoundLabel.setPos(topLeftX_, topLeftY_);
        m_LowerBoundLabel.setPos(topLeftX_, topLeftY_+m_Height-m_LowerBoundLabel.getHeight());

        m_PauseBut.setPos(topLeftX_, (int) (topLeftY_+m_Height-2.5f*m_PauseBut.getHeight()));
        m_RestartBut.setPos(topLeftX_, topLeftY_+m_Height-m_RestartBut.getHeight());

        for (int i = 0; i < lines.size(); i++) {
            lines.get(i).setPos(m_TopLeftX + m_AxisSpacing + 2, m_TopLeftY);
            linesLabel.get(i).setPos(m_TopLeftX, m_TopLeftY + (i+1) * m_LowerBoundLabel.getHeight());
        }
        needRedraw = true;
    };
    public void init() {
        for (int i = 0; i < lines.size(); i++) {
            lines.get(i).setSize(m_Width - m_AxisSpacing, m_Height);
            lines.get(i).init(m_SampleNumber);
            lines.get(i).setHorizontalSpacing(m_HorizontalSpacing);
            lines.get(i).setBoundary(m_UpperBound, m_LowerBound);

            linesLabel.add(new TextLabel(150, lines.get(i).getLineName()));
            linesLabel.get(i).setTextColor(lines.get(i).getLineColor());
        }

        String header = "Ticks";
        for (int i = 0; i < lines.size(); i++) {
            header += "," + lines.get(i).getLineName();
        }

        getFileName();
        appendToFile(header);
    };
    public void setTextColor(int TextColor_) {
        m_TextColor = TextColor_;

        for (int i = 0; i < lines.size(); i++) {
            linesLabel.get(i).setTextColor(m_TextColor);
        }
    };

    public void getFileName() {
        if (SameFile == false) {
            int n = 0;
            do {
                n++;
            } while (new File(EnvPath + FileToAppend + "-" + Integer.toString(n) + ".txt").isFile());

            FileToAppend = EnvPath + FileToAppend + "-" + Integer.toString(n) + ".txt";
        } else {
            FileToAppend = EnvPath + FileToAppend + "-" + Integer.toString(GLOBAL_TXT_NO) + ".txt";
            GLOBAL_TXT_NO++;
            File file = new File(FileToAppend);
            file.delete();
        }
    }

    public void appendToFile(String str) {
        // buffered writer is not used as i am not sure if the program will be closed normally

        if (FileToAppend != "" || FileToAppend != null) {
            // https://stackoverflow.com/questions/1625234/how-to-append-text-to-an-existing-file-in-java
            try {
                final Path path = Paths.get(FileToAppend);
                Files.write(path, Arrays.asList(str), StandardCharsets.UTF_8, 
                    Files.exists(path) ? StandardOpenOption.APPEND : StandardOpenOption.CREATE);
            } 
            catch (final IOException ioe) {
                // Add your own exception handling...
            }
        }
    };
    public void tKeyPressed() {
    };
    public String getPos() {
        return "";
    };
};

class ScanLineChart_Line implements ScanLineChart_Interface {
    private int m_HorizontalSpacing = 1;
    private int m_TopLeftX = 0, m_TopLeftY = 0;
    private int m_Height = 0, m_Width = 0;
    private DATA_TYPE m_DataType;
    private uint8_t_MailBox uint8_t_MailBox_id = null;
    private double_MailBox double_MailBox_id = null;
    private float_MailBox float_MailBox_id = null;
    private int_MailBox int_MailBox_id = null;
    private double m_LowerBound = 0, m_UpperBound = 1;
    private int m_SampleNumber = 0;
    private int[] uint8_t_value = null;
    private int[] int_value = null;
    private double[] double_value = null;
    private float[] float_value = null;
    private boolean needRedraw = true;
    private double m_NewValue = 0;
    private int m_Cursor = 0;
    private int m_LineColor = color(0, 0, 0);
    private int m_TextColor = TEXT_COLOR;
    private double m_PastPointY = 0;
    private Boolean m_isPause = false;
    private String name = "none";

    private Boolean iskLine = false;
    private double kValue = 0;

    ScanLineChart_Line(double _kValue, int lineColor_) {
        iskLine = true;
        kValue = _kValue;
        needRedraw = true;
        m_LineColor = lineColor_;
    }

    ScanLineChart_Line(uint8_t_MailBox mail_id_, int lineColor_) {
        m_DataType = DATA_TYPE.UINT8_T;
        uint8_t_MailBox_id = mail_id_;
        needRedraw = true;
        m_LineColor = lineColor_;
    }

    ScanLineChart_Line(int_MailBox mail_id_, int lineColor_) {
        m_DataType = DATA_TYPE.INT;
        int_MailBox_id = mail_id_;
        needRedraw = true;
        m_LineColor = lineColor_;
    }

    ScanLineChart_Line(double_MailBox mail_id_, int lineColor_) {
        m_DataType = DATA_TYPE.DOUBLE;
        double_MailBox_id = mail_id_;
        needRedraw = true;
        m_LineColor = lineColor_;
    }

    ScanLineChart_Line(float_MailBox mail_id_, int lineColor_) {
        m_DataType = DATA_TYPE.FLOAT;
        float_MailBox_id = mail_id_;
        needRedraw = true;
        m_LineColor = lineColor_;
    }

    public void tDraw() {
        //println("GUI: LINE-tDraw");

        fill(m_LineColor);
        if (needRedraw == true) {
            double tempY = 0;
            double BoundHeight = m_UpperBound - m_LowerBound;
            stroke(m_LineColor);
            strokeWeight(LINE_STROKE_WEIGHT);
            for (int x = 0; x < m_SampleNumber; x++) {
                if (m_DataType == DATA_TYPE.UINT8_T) {
                    tempY = uint8_t_value[x];
                } else if (m_DataType == DATA_TYPE.DOUBLE) {
                    tempY = double_value[x];
                } else if (m_DataType == DATA_TYPE.FLOAT) {
                    tempY = float_value[x];
                } else if (iskLine) {
                    tempY = kValue;
                }
                tempY = tempY > m_LowerBound ? tempY : m_LowerBound;
                tempY = tempY < m_UpperBound ? tempY : m_UpperBound;
                tempY = m_TopLeftY + m_Height - (tempY - m_LowerBound) / BoundHeight * m_Height - 2;

                if (x > 0) {
                    double tempYp = 0;

                    if (m_DataType == DATA_TYPE.UINT8_T) {
                        tempYp = uint8_t_value[x-1];
                    } else if (m_DataType == DATA_TYPE.DOUBLE) {
                        tempYp = double_value[x-1];
                    } else if (m_DataType == DATA_TYPE.FLOAT) {
                        tempYp = float_value[x-1];
                    } else if (m_DataType == DATA_TYPE.INT) {
                        tempYp = int_value[x-1];
                    } else if (iskLine) {
                        tempYp = kValue;
                    }

                    tempYp = tempYp > m_LowerBound ? tempYp : m_LowerBound;
                    tempYp = tempYp < m_UpperBound ? tempYp : m_UpperBound;
                    tempYp = m_TopLeftY + m_Height - (tempYp - m_LowerBound) / BoundHeight * m_Height - 2;

                    line(m_TopLeftX + m_HorizontalSpacing * (x-1), (int) tempYp, m_TopLeftX + m_HorizontalSpacing * x, (int) tempY);
                }
                ellipse(m_TopLeftX + m_HorizontalSpacing * x, (int) tempY, 2, 2);
            }
            needRedraw = false;
        }
    };
    public void setValue(String value_) {
        name = value_;
    };
    public void setValue(double double_) {
        m_NewValue = double_;
        if (m_isPause == false) {
            m_Cursor++;
        }
        m_Cursor = m_Cursor == m_SampleNumber ? 0 : m_Cursor;
    };
    public void setValue(float float_) {
        m_NewValue = float_;
        if (m_isPause == false) {
            m_Cursor++;
        }
        m_Cursor = m_Cursor == m_SampleNumber ? 0 : m_Cursor;
    };
    public void setValue(int int_) {
        m_NewValue = (double) int_;
        if (m_isPause == false) {
            m_Cursor++;
        }
        m_Cursor = m_Cursor == m_SampleNumber ? 0 : m_Cursor;
    };
    public void setValue(boolean bool_) {
    };
    public String getValue() {
        if (m_DataType == DATA_TYPE.UINT8_T) {
            return Integer.toString(DataArray_uint8_t[uint8_t_MailBox_id.ordinal()]);
        } else if (m_DataType == DATA_TYPE.DOUBLE) {
            return Double.toString(DataArray_double[double_MailBox_id.ordinal()]);
        } else if (m_DataType == DATA_TYPE.FLOAT) {
            return Float.toString(DataArray_float[float_MailBox_id.ordinal()]);
        } else if (m_DataType == DATA_TYPE.INT) {
            return Integer.toString(DataArray_int[int_MailBox_id.ordinal()]);
        } else if (iskLine) {
            return Double.toString(kValue);
        }
        return "";
    };
    public void setPos(int topLeftX_, int topLeftY_) {
        needRedraw = true;
        m_TopLeftX = topLeftX_;
        m_TopLeftY = topLeftY_;
    };

    public void setBoundary(double upperBound, double lowerBound) {
        if (m_LowerBound > m_UpperBound) {
            println("Error! Lower bound > Upper bound");
            while (true) {
                // put here to halt the program
            }
        }
        m_LowerBound = lowerBound;
        m_UpperBound = upperBound;
    }
    public void setHorizontalSpacing(int spacing) {
        m_HorizontalSpacing = spacing > 0 ? spacing : 1;
    };
    public void setSize(int width_, int height_) {
        m_Width = width_;
        m_Height = height_;
    };
    public void getNewValue() {
        if (m_DataType == DATA_TYPE.UINT8_T) {
            m_NewValue = (double) DataArray_uint8_t[uint8_t_MailBox_id.ordinal()];
        } else if (m_DataType == DATA_TYPE.DOUBLE) {
            m_NewValue = DataArray_double[double_MailBox_id.ordinal()];
        } else if (m_DataType == DATA_TYPE.FLOAT) {
            m_NewValue = DataArray_float[float_MailBox_id.ordinal()];
        } else if (m_DataType == DATA_TYPE.INT) {
            m_NewValue = DataArray_int[int_MailBox_id.ordinal()];
        } else if (iskLine) {
            m_NewValue = kValue;
        }

        if (m_isPause == false) {
            m_Cursor++;
        }
        m_Cursor = m_Cursor == m_SampleNumber ? 0 : m_Cursor;
    };
    public void init(int SampleNumber_) {

        m_SampleNumber = SampleNumber_;

        if (m_DataType == DATA_TYPE.UINT8_T) {
            uint8_t_value = new int[m_SampleNumber];
            for (int i = 0; i < m_SampleNumber; i++) {
                uint8_t_value[i] = 0;
            }
        } else if (m_DataType == DATA_TYPE.DOUBLE) {
            double_value = new double[m_SampleNumber];
            for (int i = 0; i < m_SampleNumber; i++) {
                double_value[i] = 0;
            }
        } else if (m_DataType == DATA_TYPE.FLOAT) {
            float_value = new float[m_SampleNumber];
            for (int i = 0; i < m_SampleNumber; i++) {
                float_value[i] = 0;
            }
        } else if (m_DataType == DATA_TYPE.INT) {
            int_value = new int[m_SampleNumber];
            for (int i = 0; i < m_SampleNumber; i++) {
                int_value[i] = 0;
            }
        }
    };
    public void drawClean() {
        //println("GUI: LINE-drawClean");

        if (m_isPause == false) {
            double BoundHeight = m_UpperBound - m_LowerBound;
            double x = m_TopLeftX + m_HorizontalSpacing * m_Cursor;
            double tempY = 0;

            strokeWeight(LINE_STROKE_WEIGHT+1);
            // cover the old point and line
            stroke(TILE_BACKGROUND_COLOR);
            if (m_DataType == DATA_TYPE.UINT8_T) {
                tempY = uint8_t_value[m_Cursor];
            } else if (m_DataType == DATA_TYPE.DOUBLE) {
                tempY = double_value[m_Cursor];
            } else if (m_DataType == DATA_TYPE.FLOAT) {
                tempY = float_value[m_Cursor];
            } else if (m_DataType == DATA_TYPE.INT) {
                tempY = int_value[m_Cursor];
            } else if (iskLine) {
                tempY = kValue;
            }

            tempY = tempY > m_LowerBound ? tempY : m_LowerBound;
            tempY = tempY < m_UpperBound ? tempY : m_UpperBound;
            tempY = m_TopLeftY + m_Height - (tempY - m_LowerBound) / BoundHeight * m_Height - 2;

            if (m_Cursor > 0) {
                line((int) (x - m_HorizontalSpacing), (int) m_PastPointY, (int) x, (int) tempY);
            }

            ellipse((int) x, (int) tempY, (int) 2, (int) 2);
        }
    };
    public void drawNew() {
        //println("GUI: LINE-drawNew");

        if (m_isPause == false) {
            double BoundHeight = m_UpperBound - m_LowerBound;
            double x = m_TopLeftX + m_HorizontalSpacing * m_Cursor;
            double tempY = 0;

            strokeWeight(LINE_STROKE_WEIGHT+1);
            // cover the old point and line
            stroke(TILE_BACKGROUND_COLOR);
            if (m_DataType == DATA_TYPE.UINT8_T) {
                tempY = uint8_t_value[m_Cursor];
            } else if (m_DataType == DATA_TYPE.DOUBLE) {
                tempY = double_value[m_Cursor];
            } else if (m_DataType == DATA_TYPE.FLOAT) {
                tempY = float_value[m_Cursor];
            } else if (m_DataType == DATA_TYPE.INT) {
                tempY = int_value[m_Cursor];
            } else if (iskLine) {
                tempY = kValue;
            }

            tempY = tempY > m_LowerBound ? tempY : m_LowerBound;
            tempY = tempY < m_UpperBound ? tempY : m_UpperBound;
            tempY = m_TopLeftY + m_Height - (tempY - m_LowerBound) / BoundHeight * m_Height - 2;

            m_PastPointY = tempY;

            strokeWeight(LINE_STROKE_WEIGHT);
            // draw the new point and line
            stroke(m_LineColor);
            if (m_DataType == DATA_TYPE.UINT8_T) {
                uint8_t_value[m_Cursor] = (int) m_NewValue;
            } else if (m_DataType == DATA_TYPE.DOUBLE) {
                double_value[m_Cursor] = m_NewValue;
            } else if (m_DataType == DATA_TYPE.FLOAT) {
                float_value[m_Cursor] = (float) m_NewValue;
            } else if (m_DataType == DATA_TYPE.INT) {
                int_value[m_Cursor] = (int) m_NewValue;
            }

            tempY = m_NewValue;
            tempY = tempY > m_LowerBound ? tempY : m_LowerBound;
            tempY = tempY < m_UpperBound ? tempY : m_UpperBound;
            tempY = m_TopLeftY + m_Height - (tempY - m_LowerBound) / BoundHeight * m_Height - 2;

            if (m_Cursor > 0) {
                double tempYp = 0;

                if (m_DataType == DATA_TYPE.UINT8_T) {
                    tempYp = uint8_t_value[m_Cursor-1];
                } else if (m_DataType == DATA_TYPE.DOUBLE) {
                    tempYp = double_value[m_Cursor-1];
                } else if (m_DataType == DATA_TYPE.FLOAT) {
                    tempYp = float_value[m_Cursor-1];
                } else if (m_DataType == DATA_TYPE.INT) {
                    tempYp = int_value[m_Cursor-1];
                } else if (iskLine) {
                    tempYp = kValue;
                }

                tempYp = tempYp > m_LowerBound ? tempYp : m_LowerBound;
                tempYp = tempYp < m_UpperBound ? tempYp : m_UpperBound;
                tempYp = m_TopLeftY + m_Height - (tempYp - m_LowerBound) / BoundHeight * m_Height - 2;

                line((int) (x - m_HorizontalSpacing), (int) tempYp, (int) x, (int) tempY);
            }

            ellipse((int) x, (int) tempY, (int) 2, (int) 2);
        }
    };
    public String getLineName() {
        return name;
    };
    public int getLineColor() {
        return m_LineColor;
    };
    public void setTextColor(int TextColor_) {
        if (m_TextColor != TextColor_) {
            m_TextColor = TextColor_;
        }
    };
    public int getCursor() {
        return m_Cursor;
    };
    public void setCursor(int cursor) {
        m_Cursor = cursor;
    };
    public void setPause(Boolean isPause) {
        m_isPause = isPause;
    };
    public void tKeyPressed() {
    };
    public String getPos() {
        return "";
    };
    public void setBackgroundFill(boolean needBackground) {
    };
    public int getHeight() {
        return m_Height;
    };
    public int getWidth() {
        return m_Width;
    };
    public void over() {
    };
    public void onClick() {
    };
    public void serialSend() {
    };
}
class InputBox extends GUI_Raw {
    String m_Value;
    boolean isHide;
    private final int label_X_Indent = 10;
    private DATA_TYPE m_ValueDataType = null;
    private uint8_t_MailBox uint8_t_MailBox_id = null;
    private double_MailBox double_MailBox_id = null;
    private float_MailBox float_MailBox_id = null;
    private int_MailBox int_MailBox_id = null;
    private bool_MailBox bool_MailBox_id = null;

    GeneralVariableTile current = null;

    InputBox() {
        super();
        m_Value = "0";
        m_Width = 200;
        m_Height = (int) textAscent() + (int) textDescent();
        isHide = true;
    };
    public @Override void setValue(int int_) {
        if (int_ < 32) {
            m_ValueDataType = DATA_TYPE.DOUBLE;
            double_MailBox_id = double_MailBox.values()[int_];
        } else if (int_ < 64) {
            m_ValueDataType = DATA_TYPE.FLOAT;
            float_MailBox_id = float_MailBox.values()[int_-32];
        } else if (int_ < 96) {
            m_ValueDataType = DATA_TYPE.UINT8_T;
            uint8_t_MailBox_id = uint8_t_MailBox.values()[int_-64];
        } else if (int_ < 128) {
            m_ValueDataType = DATA_TYPE.INT;
            int_MailBox_id = int_MailBox.values()[int_-96];
        } else if (int_ < 160) {
            m_ValueDataType = DATA_TYPE.BOOLEAN;
            bool_MailBox_id = bool_MailBox.values()[int_-128];
        }
        m_Value = "";
    };
    public @Override void setValue(String value_) {
        this.setPos(Integer.parseInt(value_.split(",")[0]), Integer.parseInt(value_.split(",")[1]));
    };
    public @Override void tDraw() {
        if (!isHide) {
            stroke(5);
            fill(TILE_BACKGROUND_COLOR);
            rect(m_TopLeftX, m_TopLeftY, m_Width, m_Height, RECT_ANGLE_RADIUS);
            textAlign(LEFT);
            fill(m_TextColor);
            textSize(fontSize);
            text(m_Value, m_TopLeftX + label_X_Indent, m_TopLeftY, m_Width - fontSize, m_Height);
        } else {
            textAlign(RIGHT);
            fill(m_TextColor);
            textSize(fontSize);
            text("<", m_TopLeftX + label_X_Indent, m_TopLeftY, m_Width - fontSize, m_Height);
        }
    };
    public @Override void tKeyPressed() {
        if (m_ValueDataType != null) {
            isHide = false;
            switch (key) {
            case BACKSPACE:

                if (m_Value.length() != 0) {
                    m_Value = m_Value.substring(0, m_Value.length()-1);
                }
                break;

            case ENTER:
                if (m_Value.length() > 0) {
                    double y = Double.parseDouble(m_Value);

                    if (m_ValueDataType == DATA_TYPE.DOUBLE) {
                        DataArray_double[double_MailBox_id.ordinal()] = Double.parseDouble(m_Value);
                        if (uart != null)
                            uart.Send_double(double_MailBox_id, DataArray_double[double_MailBox_id.ordinal()]);
                    } else if (m_ValueDataType == DATA_TYPE.UINT8_T) {
                        while (((int) y) > 255) {
                            y -= 256;
                        };
                        while ((int) y < 0) {
                            y += 256;
                        };
                        DataArray_uint8_t[uint8_t_MailBox_id.ordinal()] = (int) y;
                        m_Value = Double.toString(y);

                        if (uart != null)
                            uart.Send_uint8_t(uint8_t_MailBox_id, DataArray_uint8_t[uint8_t_MailBox_id.ordinal()]);
                    } else if (m_ValueDataType == DATA_TYPE.FLOAT) {
                        DataArray_float[float_MailBox_id.ordinal()] = (float) y;

                        if (uart != null)
                            uart.Send_float(float_MailBox_id, DataArray_float[float_MailBox_id.ordinal()]);
                    } else if (m_ValueDataType == DATA_TYPE.INT) {
                        DataArray_int[int_MailBox_id.ordinal()] = (int) y;

                        if (uart != null)
                            uart.Send_int(int_MailBox_id, DataArray_int[int_MailBox_id.ordinal()]);
                    } else if (m_ValueDataType == DATA_TYPE.BOOLEAN) {
                        DataArray_bool[bool_MailBox_id.ordinal()] = y != 0 ? true : false;

                        if (uart != null)
                            uart.Send_bool(bool_MailBox_id, DataArray_bool[bool_MailBox_id.ordinal()]);
                    }

                    print("GUI: InputBox-ENTER:" + m_Value + "\n\n");

                    m_Value = "0";
                    isHide = true;
                }
                break;

            case '1':
            case '2':
            case '3':
            case '4':
            case '5':
            case '6':
            case '7':
            case '8':
            case '9':
            case '0':
                if (m_Value.equals("0")) {
                    m_Value = key + "";
                } else {
                    m_Value = m_Value + key;
                }
                break;

            case '.':
                if (m_Value.length() > 0 && !m_Value.contains(".")) {
                    m_Value = m_Value + key;
                }
                break;

            case '-':
                if (m_Value.length() == 0 || m_Value.equals("0")) {
                    m_Value = key + "";
                }
                break;

            default:
                break;
            };
        }

        tDraw();
    };

    public @Override
        void setPos(int topLeftX_, int topLeftY_) {
        if (topLeftX_ == 0 && topLeftY_ == 0) {
            if (current != null) 
                this.setValue(current.m_ValueLabel.getPos());
        } else {
            m_TopLeftX = topLeftX_;
            m_TopLeftY = topLeftY_;
        }
    }

    public void reg(GeneralVariableTile current_) {
        current = current_;
    }
}
class ElapsedTime extends GUI_Raw {
    private TextLabel m_AttributeLable;
    private TextLabel m_ValueLabel;

    private int m_Last_SYSTEM_MSG_elpasedTime = 0;
    private int m_ConnLostCount = 0;
    private final int m_ConnLostCountMax = 20;

    ElapsedTime() {
        super();
        m_Width = 200;

        m_AttributeLable = new TextLabel(200, "MCU Run Time:");
        m_ValueLabel = new TextLabel(200, "Unconnected");

        m_AttributeLable.setBackgroundFill(false);
        m_ValueLabel.setBackgroundFill(false);

        m_AttributeLable.setTextColor(m_TextColor);
        m_ValueLabel.setTextColor(m_TextColor);

        m_Height = m_AttributeLable.getHeight()*2;
    };

    public void tDraw() {
        noStroke();
        fill(TILE_BACKGROUND_COLOR);
        rect(m_TopLeftX, m_TopLeftY, m_Width, m_Height, RECT_ANGLE_RADIUS);

        if (SYSTEM_MSG_elpasedTime == m_Last_SYSTEM_MSG_elpasedTime) {
            m_ConnLostCount++;
        } else {
            m_ConnLostCount = 0;
        }

        if (m_ConnLostCount >= m_ConnLostCountMax) {
            m_ValueLabel.setValue("Conn Lost");
        } else {
            m_ValueLabel.setValue(Integer.toString(SYSTEM_MSG_elpasedTime) + " s");
        }

        m_Last_SYSTEM_MSG_elpasedTime = SYSTEM_MSG_elpasedTime;

        m_AttributeLable.tDraw();
        m_ValueLabel.tDraw();
    };
    public @Override void setPos(int topLeftX_, int topLeftY_) {
        m_TopLeftX = topLeftX_;
        m_TopLeftY = topLeftY_;

        m_AttributeLable.setPos(m_TopLeftX, m_TopLeftY);
        m_ValueLabel.setPos(m_TopLeftX, m_TopLeftY+m_AttributeLable.getHeight());
    };
}
//Path path = FileSystems.getDefault().getPath(".");
final String EnvPath = System.getProperty("user.dir");
//"/Users/JosephYim/Desktop/";
final boolean SameFile = true;

//final String BTtarget = "tty.MORRIS";
final String BTtarget = "tty.JOSEPH";
//final String BTtarget = "tty.COLUMN";


public void init() { // add tiles here 

    //addOutputValueTile(Mailbox.f0, "master slope");
    //addOutputValueTile(Mailbox.f1, "slave slope");
    //addOutputValueTile(Mailbox.f2, "middle slope");
    //addOutputValueTile(Mailbox.i2, "9ntt");
    //addOutputValueTile(Mailbox.u3, "echo 3");

    addOutputValueTile(Mailbox.u0, "u0");
    addOutputValueTile(Mailbox.u1, "u1");
    addOutputValueTile(Mailbox.u2, "u2");
    addOutputValueTile(Mailbox.u3, "u3");
    addLineBreak(); // break into next column

    addOutputValueTile(Mailbox.u4, "u4");
    addOutputValueTile(Mailbox.u5, "u5");
    addOutputValueTile(Mailbox.u6, "u6");
    addOutputValueTile(Mailbox.u7, "u7");

    //addOutputValueTile(Mailbox.i0, "i0");

    //addLineBreak(); // break into next column

    //addInputIncDecTile(Mailbox.i0, "i0", 0);
    //addInputIncDecTile(Mailbox.f0, "f0", 1, 1, 0);
    //addInputIncDecTile(Mailbox.d0, "d0", 1, 1, 0);
    //addInputIncDecTile(Mailbox.i0, "i0", 1, 1, 0);
    //addLineBreak(); // break into next column

    //addChart(100, 5, 600, -1, 255); // << start a new chart, i.e. chart 1

    ////DataArray_double[Mailbox.d29.ordinal()] = 100;
    ////addLine(Mailbox.d29, "constant line", BLACK);


    //addLine(Mailbox.f0, "echo 0", RED); // >> put into chart 1
    //addLine(60, "echo 1", BLUE); // >> put into chart 1
    ////addLine(Mailbox.f2, "echo 2", GREEN); // >> put into chart 1
    ////addLine(Mailbox.f3, "echo 3", TEAL); // >> put into chart 1
    //addLine(Mailbox.i0, "echo 3", TEAL); // >> put into chart 1

    //addChart(100, 5, 600, -1, 255); // << start a new chart, i.e. chart 1
    //addLine(Mailbox.u0, "trig", BLACK);
    //addLine(Mailbox.f0, "echo 0", RED); // >> put into chart 1
    //addLine(Mailbox.f1, "echo 1", BLUE); // >> put into chart 1
    //addLine(Mailbox.f2, "echo 2", GREEN); // >> put into chart 1
    //addLine(Mailbox.f3, "echo 3", TEAL); // >> put into chart 1

    //addCameraGraph();

    addElapsedTime();
    //addOutputValueTile(Mailbox.b0, "b0");
    //addButtonTile(Mailbox.b0, "b0");
    //addButtonTile(Mailbox.b0, "b0");

    //addChart(100, 5, 600, -1, 255); // << start a new chart, i.e. chart 1

    ////DataArray_double[Mailbox.d29.ordinal()] = 100;
    ////addLine(Mailbox.d29, "constant line", BLACK);

    //addLine(Mailbox.u1, "u1", BLACK);
}

int tick = 0;
boolean t = false;
public void cycle() {
    tick++;
    if (tick % 10 == 0) {
        t = !t;
        //uart.Send_bool(bool_MailBox.b1, t);
    }
}




UART uart;

public class UART {
    class PACKAGE {
        PACKAGE() {
        }
        Byte START_CODE;
        Byte COMMAND_CODE;
        Byte Data_0;
        Byte Data_1;
    };

    Serial m_port;

    Byte[] RxBuffer;
    int RxBuffer_Cursor;

    Byte[] dataBuffer = new Byte[LOCAL_BUFFER_MAX];

    ArrayList<PACKAGE> SendImmediate;
    ArrayList<PACKAGE> SendBuffer;

    final Byte StartCode0 = -2; // 0b11111110
    final Byte StartCode1 = -1; // 0b11111111

    int SendCooling;
    int CameraOffset = 0;

    UART(Serial port) {
        // init usb serial
        m_port = port; // new Serial(this, "COM6", 9600);
        m_port.buffer(1);

        // init
        RxBuffer = new Byte[4];
        RxBuffer[0] = 0;
        RxBuffer[1] = 0;
        RxBuffer[2] = 0;
        RxBuffer[3] = 0;
        RxBuffer_Cursor = 0;

        SendCooling = 0;

        SendImmediate = new ArrayList<PACKAGE>(0);
        SendBuffer = new ArrayList<PACKAGE>(0);

        print("\nUART: Init success\n");
        print("Port Connected is " + port + "\n");
    }

    public String to8BitBinary(byte t) {
        String x = Integer.toBinaryString(t);
        if (x.length() < 8) {
            return ("00000000" + x).substring(x.length());
        } else {
            return x.substring(24, 32);
        }
    }

    public void tSerialEvent() {
        if (SendBuffer.size() != 0 && SendCooling == 0) {
            PACKAGE pkg = SendBuffer.get(0);
            byte[] pkgByte = new byte[4];
            pkgByte[0] = pkg.START_CODE;
            pkgByte[1] = pkg.COMMAND_CODE;
            pkgByte[2] = pkg.Data_0;
            pkgByte[3] = pkg.Data_1;
            m_port.write(pkgByte);

            println("UART: SendBuffer, ");
            print("sent: ");
            print(to8BitBinary(pkgByte[0]) + " ");
            print(to8BitBinary(pkgByte[1]) + " ");
            print(to8BitBinary(pkgByte[2]) + " ");
            print(to8BitBinary(pkgByte[3]) + "\n");
        }

        while (SendImmediate.size() != 0) {
            PACKAGE pkg = SendImmediate.get(0);
            byte[] pkgByte = new byte[4];
            pkgByte[0] = pkg.START_CODE;
            pkgByte[1] = pkg.COMMAND_CODE;
            pkgByte[2] = pkg.Data_0;
            pkgByte[3] = pkg.Data_1;
            m_port.write(pkgByte);
            SendImmediate.remove(0);

            print("UART: SendImmediate, ");
            print("sent: ");
            print(to8BitBinary(pkgByte[0]) + " ");
            print(to8BitBinary(pkgByte[1]) + " ");
            print(to8BitBinary(pkgByte[2]) + " ");
            print(to8BitBinary(pkgByte[3]) + "\n");
        }

        while (m_port.available() > 0) {
            if (RxBuffer_Cursor == 4) {
                // a complete queue la

                int DATA_TYPE_index = ((RxBuffer[1] & -32) >> 5);
                DATA_TYPE_index += DATA_TYPE_index < 0 ? 8 : 0;

                if ((DATA_TYPE_index >= 0 && DATA_TYPE_index <= 7) &&
                    (RxBuffer[0] == StartCode0 || RxBuffer[0] == StartCode1) && 
                    isOdd(RxBuffer[0], RxBuffer[1], RxBuffer[2], RxBuffer[3])) {

                    DATA_TYPE DataType = DATA_TYPE.values()[DATA_TYPE_index]; // 0b1110 0000

                    int MailBox_ = (RxBuffer[1] & 31); // 0b00011111
                    MailBox_ += MailBox_ < 0 ? 256 : 0;

                    print("\nUART: Received Complete Package. \n");
                    print("Content: ");
                    print(to8BitBinary(RxBuffer[0]) + " ");
                    print(to8BitBinary(RxBuffer[1]) + " ");
                    print(to8BitBinary(RxBuffer[2]) + " ");
                    print(to8BitBinary(RxBuffer[3]) + "\n");

                    if (DataType == DATA_TYPE.BUFFER) {
                        if (MailBox_ < (LOCAL_BUFFER_MAX/2)) {
                            dataBuffer[MailBox_*2] = RxBuffer[2];
                            dataBuffer[MailBox_*2+1] = RxBuffer[3];

                            print("> Buffer\n");
                        }
                        acker(RxBuffer[1]);
                    } else if (DataType == DATA_TYPE.DOUBLE && dataBuffer[0] != null) {
                        if (DataArray_double.length > MailBox_) {
                            byte[] byteArr = new byte[8];
                            for (int i = 0; i < 6; i++) {
                                byteArr[7-i] = dataBuffer[i];
                            }
                            byteArr[1] = RxBuffer[2];
                            byteArr[0] = RxBuffer[3];
                            DataArray_double[MailBox_] = ByteBuffer.wrap(byteArr).getDouble();

                            print("> Double, MailBox: " + MailBox_ + ", Value: " + DataArray_double[MailBox_] + "\n");
                        }
                        acker(RxBuffer[1]);
                    } else if (DataType == DATA_TYPE.FLOAT && dataBuffer[0] != null) {

                        if (DataArray_float.length > MailBox_) {
                            byte[] byteArr = new byte[4];
                            byteArr[3] = dataBuffer[0];
                            byteArr[2] = dataBuffer[1];
                            byteArr[1] = RxBuffer[2];
                            byteArr[0] = RxBuffer[3];
                            DataArray_float[MailBox_] = ByteBuffer.wrap(byteArr).getFloat();

                            print("> Float, MailBox: " + MailBox_ + ", Value: " + DataArray_float[MailBox_] + "\n");
                        }
                        acker(RxBuffer[1]);
                    } else if (DataType == DATA_TYPE.INT) {

                        if (DataArray_int.length > MailBox_ && dataBuffer[0] != null) {
                            byte[] byteArr = new byte[4];
                            byteArr[3] = dataBuffer[0];
                            byteArr[2] = dataBuffer[1];
                            byteArr[1] = RxBuffer[2];
                            byteArr[0] = RxBuffer[3];
                            DataArray_int[MailBox_] = ByteBuffer.wrap(byteArr).getInt();

                            print("> Int, MailBox: " + MailBox_ + ", Value: " + DataArray_int[MailBox_] + "\n");
                        }
                        acker(RxBuffer[1]);
                    } else if (DataType == DATA_TYPE.MORRIS_VECTOR) {

                        VectorGraph_ArraryList.get(MailBox_ + 1).setCor(RxBuffer[2], RxBuffer[3]);
                        print("> MORRIS_VECTOR, Node: " + (MailBox_+1) + ", XPos: " + RxBuffer[2] + ", YPos: " + RxBuffer[3]+ "\n");
                    } else if (DataType == DATA_TYPE.UINT8_T) {

                        if (DataArray_uint8_t.length > MailBox_) {

                            println("Debug Info: MailBox_->" + MailBox_);
                            
                            if (MailBox_ == uint8_t_MailBox.BOOLEAN.ordinal()) {
                                DataArray_bool[RxBuffer[2]] = (RxBuffer[3] == 1);

                                print("> BOOLEAN, MailBox: " + RxBuffer[2] + ", Value: " + DataArray_bool[RxBuffer[2]] + "\n");
                            } else {
                                DataArray_uint8_t[MailBox_] = RxBuffer[2];
                                if (DataArray_uint8_t[MailBox_] < 0) {
                                    DataArray_uint8_t[MailBox_] += 256;

                                    print("> UINT8_T, MailBox: " + MailBox_ + ", Value: " + DataArray_uint8_t[MailBox_] + "\n");
                                }
                            }
                        }
                        acker(RxBuffer[1]);
                    } else if (DataType == DATA_TYPE.SYSTEM) {
                        if (SYSTEM_MSG.lastTerm.ordinal() >= MailBox_) {
                            if (MailBox_ == SYSTEM_MSG.ack.ordinal()) {
                                if (SendBuffer.size() !=0 && SendBuffer.get(0).COMMAND_CODE == RxBuffer[2]) {
                                    // correct ack
                                    SendBuffer.remove(0);
                                    SendCooling = 0;

                                    print("> ACK.\n");
                                } else {
                                    // wrong ack, ignore it
                                }
                                // ack no need to ack ack
                            } else if (MailBox_ == SYSTEM_MSG.sayHi.ordinal()) {
                                acker(RxBuffer[1]);
                                for (int i = 0; i < tiles.size(); i++) {
                                    tiles.get(i).serialSend();
                                }

                                print("> Say Hi.\n");
                            } else if (MailBox_ == SYSTEM_MSG.elpasedTime.ordinal()) {
                                int upperHalf = RxBuffer[2];
                                int lowerHalf = RxBuffer[3];
                                upperHalf += upperHalf < 0 ? 256: 0;
                                lowerHalf += lowerHalf < 0 ? 256: 0;

                                SYSTEM_MSG_elpasedTime = (upperHalf<<8) + lowerHalf;
                                acker(RxBuffer[1]);

                                print("> Elapsed time: " + SYSTEM_MSG_elpasedTime + ".\n");
                            } else if (MailBox_ == SYSTEM_MSG.vector_StartSend.ordinal()) {

                                print("> vector_StartSend: Node: 0, XPos: " + RxBuffer[2] + ", YPos: " + RxBuffer[3]+ "\n");

                                if (VectorGraph_ArraryList != null) {
                                    int len = VectorGraph_ArraryList.size();
                                    for (int i = 0; i < len; i++) {
                                        VectorGraph_ArraryList.get(i).setClean();
                                    }
                                } else {
                                    println("GUI: Critical Error! VectorGraph_ArraryList is null!");
                                }

                                VectorGraph_ArraryList.get(0).setCor(RxBuffer[2], RxBuffer[3]);
                            } else if (MailBox_ == SYSTEM_MSG.vector_EndSend.ordinal()) {

                                print("> vector_EndSend, NumOfNode: " + RxBuffer[2] + ".\n");

                                if (VectorGraph_ArraryList != null) {
                                    int len = VectorGraph_ArraryList.size();
                                    for (int i = 0; i < len; i++) {
                                        VectorGraph_ArraryList.get(i).update();
                                    }
                                } else {
                                    println("GUI: Critical Error! VectorGraph_ArraryList is null!");
                                }
                            }

                            //else if (MailBox_ == SYSTEM_MSG.camera_StartSend.ordinal()) {

                            //    print("> camera_StartSend, Width: " + RxBuffer[2] + ", Height: " + RxBuffer[3] + ".\n");

                            //    if (RxBuffer[2] != CAMERA_GRAPH_X || RxBuffer[3] != CAMERA_GRAPH_Y) {
                            //        println("UART: Critical ERROR! Camera Config Wrong!");
                            //    }

                            //    if (CameraGraph_ArraryList != null) {
                            //        int len = CameraGraph_ArraryList.size();
                            //        for (int i = 0; i < len; i++) {
                            //            CameraGraph_ArraryList.get(i).setClean();
                            //        }
                            //    } else {
                            //        println("GUI: Critical Error! CameraGraph_ArraryList is null!");
                            //    }
                            //} else if (MailBox_ == SYSTEM_MSG.camera_EndSend.ordinal()) {

                            //    print("> camera_EndSend.\n");

                            //    if (CameraGraph_ArraryList != null) {
                            //        int len = CameraGraph_ArraryList.size();
                            //        for (int i = 0; i < len; i++) {
                            //            CameraGraph_ArraryList.get(i).update();
                            //        }
                            //    } else {
                            //        println("GUI: Critical Error! CameraGraph_ArraryList is null!");
                            //    }
                            //}
                        }
                    }

                    // info extracted and reset the buffer
                    RxBuffer_Cursor = 0;
                } else {
                    // parity check failed
                    // discard the first one and shift everything left
                    println("UART: Trashed 1 byte: " + to8BitBinary(RxBuffer[0]));

                    TotalTrashedByte++;
                    println("TotalTrashedByte: " + TotalTrashedByte);

                    RxBuffer[0] = RxBuffer[1];
                    RxBuffer[1] = RxBuffer[2];
                    RxBuffer[2] = RxBuffer[3];
                    RxBuffer[3] = 0;
                    RxBuffer_Cursor = 3;

                    if (!(isOdd(RxBuffer[0], RxBuffer[1], RxBuffer[2], RxBuffer[3]))) {
                        println("UART: Parity check failed");
                    } else if (!(DATA_TYPE_index >= 0 && DATA_TYPE_index <= 7)) {
                        println("UART: Wrong data type");
                    } else if (!(RxBuffer[0] == StartCode0 || RxBuffer[0] == StartCode1)) {
                        println("UART: Wrong start code");
                    }

                    println();
                }
            } else {
                // rxBuffer is not full
                RxBuffer[RxBuffer_Cursor] = m_port.readBytes(1)[0];
                RxBuffer_Cursor++;
            }
        }
    }

    public boolean isOdd(Byte t0, Byte t1, Byte t2, Byte t3) {
        int t = 0;
        for (int i = 0; i < 8; i++) {
            if ((t0 & 1) == 1) {
                t++;
            }
            if ((t1 & 1) == 1) {
                t++;
            }
            if ((t2 & 1) == 1) {
                t++;
            }
            if ((t3 & 1) == 1) {
                t++;
            }

            t0 = (byte) (t0 >> 1);
            t1 = (byte) (t1 >> 1);
            t2 = (byte) (t2 >> 1);
            t3 = (byte) (t3 >> 1);
        }

        return (t % 2 == 1);
    }

    public void SendWrapper(DATA_TYPE DataType, int MailBox, byte byte0, byte byte1, boolean sendImmediateBool) {
        PACKAGE pkg = new PACKAGE();
        pkg.START_CODE = StartCode0;
        pkg.COMMAND_CODE = (byte) (DataType.ordinal()*32 + MailBox);
        pkg.Data_0 = byte0;
        pkg.Data_1 = byte1;

        if (!isOdd(pkg.START_CODE, pkg.COMMAND_CODE, pkg.Data_0, pkg.Data_1)) {
            pkg.START_CODE = StartCode1;
        }

        SendImmediate.add(SendImmediate.size(), pkg);
        if (sendImmediateBool == true) {
        }
        //if (sendImmediateBool == true) {
        //    SendImmediate.add(SendImmediate.size(), pkg);
        //} else {
        //    SendBuffer.add(SendBuffer.size(), pkg);
        //}
    }

    public void Send_uint8_t(uint8_t_MailBox MailBox, int num) {
        SendWrapper(DATA_TYPE.UINT8_T, MailBox.ordinal(), (byte) num, (byte) 0, false);

        print("\nUART: Send_uint8_t, ");
        print("MailBox: " + MailBox + " Value is " + num + "\n");
    }

    public void Send_bool(bool_MailBox MailBox, boolean num) {
        SendWrapper(DATA_TYPE.UINT8_T, uint8_t_MailBox.BOOLEAN.ordinal(), (byte) MailBox.ordinal(), (byte)(num == true ? 1 : 0), false);

        print("\nUART: Send_bool, ");
        print("MailBox: " + MailBox + " Value is " + (num == true ? "True" : "False") + "\n");
    }

    public void Send_double(double_MailBox MailBox, double num) {
        byte[] doublePtr = new byte[8];
        ByteBuffer.wrap(doublePtr).putDouble(num);
        SendWrapper(DATA_TYPE.BUFFER, 0, doublePtr[7], doublePtr[6], false);
        SendWrapper(DATA_TYPE.BUFFER, 1, doublePtr[5], doublePtr[4], false);
        SendWrapper(DATA_TYPE.BUFFER, 2, doublePtr[3], doublePtr[2], false);
        SendWrapper(DATA_TYPE.DOUBLE, MailBox.ordinal(), doublePtr[1], doublePtr[0], false);

        print("\nUART: Send_double, ");
        print("MailBox: " + MailBox + " Value is " + num + "\n");
    }

    public void Send_float(float_MailBox MailBox, float num) {
        byte[] floatPtr = new byte[4];
        ByteBuffer.wrap(floatPtr).putFloat(num);
        SendWrapper(DATA_TYPE.BUFFER, 0, floatPtr[3], floatPtr[2], false);
        SendWrapper(DATA_TYPE.FLOAT, MailBox.ordinal(), floatPtr[1], floatPtr[0], false);

        print("\nUART: Send_float, ");
        print("MailBox: " + MailBox + " Value is " + num + "\n");
    }

    public void Send_int(int_MailBox MailBox, int num) {
        byte[] intPtr = new byte[4];
        ByteBuffer.wrap(intPtr).putInt(num);
        SendWrapper(DATA_TYPE.BUFFER, 0, intPtr[3], intPtr[2], false);
        SendWrapper(DATA_TYPE.INT, MailBox.ordinal(), intPtr[1], intPtr[0], false);

        print("\nUART: Send_int, ");
        print("MailBox: " + MailBox + " Value is " + num + "\n");
    }

    public void acker(byte code) {
        //SendWrapper(DATA_TYPE.SYSTEM, SYSTEM_MSG.ack.ordinal(), code, (byte) 0, true);
        //print("\nAck sent.\n");
    }
    
    public void close() {
        m_port.stop();
    }
}

public void InitUART(int id_) {
    int id = id_;

    println("UART init: Start Trying at " + id_);
    println("Available UART ports:");
    for (int i = 0; i < Serial.list().length; i++) {
        println(i + ". " + Serial.list()[i]);
    }

    while (((Serial.list().length > id) && (BTtarget != null)) 
        && (!Serial.list()[id].contains(BTtarget))) {
        id++;
    }

    if (!(Serial.list().length > id)) {
        println("UART: ERROR: Can't Connect BT");
        UARTstatus.setValue("Disconnected. Click to Init.");
    } else {
        println("ID " + id + ":" + Serial.list()[id] + " is selected, attempting to connect it");

        Serial port = null;

        try {
            port = new Serial(this, Serial.list()[id], 115200);
        } 
        catch (Exception e) {
            port = null;

            println("UART connection FAILED!");
            println();

            UARTstatus.setValue("Disconnected. Click to Init.");

            if (id+1 < Serial.list().length) 
                InitUART(id+1);
        }

        if (port != null) {
            UARTstatus.setValue("Connected. Click to ReInit.");
            uart = new UART(port);
        }
    }
}
    public void settings() {  size(400, 400); }
    static public void main(String[] passedArgs) {
        String[] appletArgs = new String[] { "SmartCar_DualCar_GUI" };
        if (passedArgs != null) {
          PApplet.main(concat(appletArgs, passedArgs));
        } else {
          PApplet.main(appletArgs);
        }
    }
}

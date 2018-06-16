import java.util.*; 

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

void addOutputValueTile(Mailbox mailbox, String name) {
    addGeneralVariableTile(mailbox, name, 0);
}

void addInputIncDecTile(Mailbox mailbox, String name, double initValue) {
    addGeneralVariableTile(mailbox, name, 0);
}

// GeneralVariableTile
void addGeneralVariableTile(Mailbox mailbox, String name, double initValue) {
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

void addGeneralVariableTile(int_MailBox mailbox, String name, int initValue) {
    GeneralVariableTile t = new GeneralVariableTile(mailbox, initValue);
    t.setValue(name);
    tiles.add(t);
}

void addGeneralVariableTile(uint8_t_MailBox mailbox, String name, int initValue) {

    // !!! missing range check here

    GeneralVariableTile t = new GeneralVariableTile(mailbox, initValue);
    t.setValue(name);
    tiles.add(t);
}

void addGeneralVariableTile(double_MailBox mailbox, String name, double initValue) {
    GeneralVariableTile t = new GeneralVariableTile(mailbox, initValue);
    t.setValue(name);
    tiles.add(t);
}

void addGeneralVariableTile(float_MailBox mailbox, String name, float initValue) {
    GeneralVariableTile t = new GeneralVariableTile(mailbox, initValue);
    t.setValue(name);
    tiles.add(t);
}

void addGeneralVariableTile(bool_MailBox mailbox, String name, boolean initValue) {
    GeneralVariableTile t = new GeneralVariableTile(mailbox, initValue);
    t.setValue(name);
    tiles.add(t);
}

// addElapsedTime
void addElapsedTime() {
    tiles.add(new ElapsedTime());
}

// addChart
void addChart(int SampleSize_, int HorizontalSpacing, int chart_height, double LowerBound, double UpperBound) {
    if (currentChart != null) 
        currentChart.init();
    currentChart = new ScanLineChart(SampleSize_, HorizontalSpacing, chart_height, LowerBound, UpperBound);
    tiles.add(currentChart);
}

// addLine
void addLine(Mailbox mailbox, String name, color c) {
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

void addLine(float_MailBox mailbox, String name, color c) {
    if (currentChart != null) {
        ScanLineChart_Line t = new ScanLineChart_Line(mailbox, c);
        t.setValue(name);
        currentChart.lines.add(t);
    }
}

void addLine(uint8_t_MailBox mailbox, String name, color c) {
    if (currentChart != null) {
        ScanLineChart_Line t = new ScanLineChart_Line(mailbox, c);
        t.setValue(name);
        currentChart.lines.add(t);
    }
}

void addLine(int_MailBox mailbox, String name, color c) {
    if (currentChart != null) {
        ScanLineChart_Line t = new ScanLineChart_Line(mailbox, c);
        t.setValue(name);
        currentChart.lines.add(t);
    }
}

void addLine(double_MailBox mailbox, String name, color c) {
    if (currentChart != null) {
        ScanLineChart_Line t = new ScanLineChart_Line(mailbox, c);
        t.setValue(name);
        currentChart.lines.add(t);
    }
}

void addLine(double value, String name, color c) {
    if (currentChart != null) {
        ScanLineChart_Line t = new ScanLineChart_Line(value, c);
        t.setValue(name);
        currentChart.lines.add(t);
    }
}

// addLineBreak
void addLineBreak() {
    tiles.add(new BreakColumn(false));
}

void addCameraGraph() {
    tiles.add(new CameraGraph());
}

void addButtonTile(Mailbox mailbox, String name) {
    if (mailbox.ordinal() < 160 && mailbox.ordinal() >= 128) {
        addButtonTile(bool_MailBox.values()[mailbox.ordinal() - 128], name);
    } else {
        println("GUI: Button Init Error!");
    }
}

void addButtonTile(bool_MailBox mailbox, String name) {
    ButtonTile t = new ButtonTile(mailbox);
    t.setValue(name);
    tiles.add(t);
}

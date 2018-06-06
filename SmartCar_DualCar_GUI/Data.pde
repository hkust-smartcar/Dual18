import java.util.*; 

public enum DATA_TYPE {
    BUFFER, DOUBLE, UINT8_T, FLOAT, INT, DIUDIU, DIUDIUDIU, SYSTEM;
}

public enum SYSTEM_MSG {
    ack, 
        sayHi, 
        elpasedTime, 
        lastTerm;
};

public int[] DataArray_uint8_t;
public double[] DataArray_double;
public float[] DataArray_float;
public int[] DataArray_int;

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
        u30, 
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
        u30, 
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
        i_MaxTerm
}

// addOutputValueTile
void addOutputValueTile(Mailbox mailbox, String name) {
    if (mailbox.ordinal() < 32) {
        addOutputValueTile(double_MailBox.values()[mailbox.ordinal()], name);
    } else if (mailbox.ordinal() < 64) {
        addOutputValueTile(float_MailBox.values()[mailbox.ordinal() - 32], name);
    } else if (mailbox.ordinal() < 96) {
        addOutputValueTile(uint8_t_MailBox.values()[mailbox.ordinal() - 64], name);
    } else if (mailbox.ordinal() < 128) {
        addOutputValueTile(int_MailBox.values()[mailbox.ordinal() - 96], name);
    }
}

void addOutputValueTile(uint8_t_MailBox mailbox, String name) {
    OutputValueTile t = new OutputValueTile(mailbox);
    t.setValue(name);
    tiles.add(t);
}

void addOutputValueTile(double_MailBox mailbox, String name) {
    OutputValueTile t = new OutputValueTile(mailbox);
    t.setValue(name);
    tiles.add(t);
}

void addOutputValueTile(float_MailBox mailbox, String name) {
    OutputValueTile t = new OutputValueTile(mailbox);
    t.setValue(name);
    tiles.add(t);
}

void addOutputValueTile(int_MailBox mailbox, String name) {
    OutputValueTile t = new OutputValueTile(mailbox);
    t.setValue(name);
    tiles.add(t);
}

// addInputIncDecTile
void addInputIncDecTile(Mailbox mailbox, String name, double small, double large, double initValue) {
    if (mailbox.ordinal() < 32) {
        addInputIncDecTile(double_MailBox.values()[mailbox.ordinal()], name, (double) small, (double) large, (double) initValue);
    } else if (mailbox.ordinal() < 64) {
        addInputIncDecTile(float_MailBox.values()[mailbox.ordinal() - 32], name, (float) small, (float) large, (float) initValue);
    } else if (mailbox.ordinal() < 96) {
        addInputIncDecTile(uint8_t_MailBox.values()[mailbox.ordinal() - 64], name, (int) small, (int) large, (int) initValue);
    } else if (mailbox.ordinal() < 128) {
        addInputIncDecTile(int_MailBox.values()[mailbox.ordinal() - 96], name, (int) small, (int) large, (int) initValue);
    }
}

void addInputIncDecTile(int_MailBox mailbox, String name, int small, int large, int initValue) {
    InputIncDecTile t = new InputIncDecTile(mailbox, small, large, initValue);
    t.setValue(name);
    tiles.add(t);
}

void addInputIncDecTile(uint8_t_MailBox mailbox, String name, int small, int large, int initValue) {
    
    // !!! missing range check here
    
    InputIncDecTile t = new InputIncDecTile(mailbox, small, large, initValue);
    t.setValue(name);
    tiles.add(t);
}

void addInputIncDecTile(double_MailBox mailbox, String name, double small, double large, double initValue) {
    InputIncDecTile t = new InputIncDecTile(mailbox, small, large, initValue);
    t.setValue(name);
    tiles.add(t);
}

void addInputIncDecTile(float_MailBox mailbox, String name, float small, float large, float initValue) {
    InputIncDecTile t = new InputIncDecTile(mailbox, small, large, initValue);
    t.setValue(name);
    tiles.add(t);
}

// addElapsedTime
void addElapsedTime() {
    tiles.add(new ElapsedTime());
}

// addChart
void addChart(int SampleSize_, int HorizontalSpacing, int chart_height, double LowerBound, double UpperBound) {
    ScanLineChart chart = new ScanLineChart(SampleSize_, HorizontalSpacing, chart_height, LowerBound, UpperBound);
    charts.add(chart);
    chartsNum++;
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
    ScanLineChart_Line t = new ScanLineChart_Line(mailbox, c);
    t.setValue(name);
    charts.get(chartsNum).lines.add(t);
}

void addLine(uint8_t_MailBox mailbox, String name, color c) {
    ScanLineChart_Line t = new ScanLineChart_Line(mailbox, c);
    t.setValue(name);
    charts.get(chartsNum).lines.add(t);
}

void addLine(int_MailBox mailbox, String name, color c) {
    ScanLineChart_Line t = new ScanLineChart_Line(mailbox, c);
    t.setValue(name);
    charts.get(chartsNum).lines.add(t);
}

void addLine(double_MailBox mailbox, String name, color c) {
    ScanLineChart_Line t = new ScanLineChart_Line(mailbox, c);
    t.setValue(name);
    charts.get(chartsNum).lines.add(t);
}

// addLineBreak
void addLineBreak() {
    tiles.add(new BreakColumn(false));
}

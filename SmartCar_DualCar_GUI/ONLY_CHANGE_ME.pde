/*
mailbox naming format:
 uint8_t_Mailbox: u0 to u30
 float_Mailbox: f0 to f30
 double_Mailbox: d0 to d30
 
 func:
 >>> addOutputValueTile(mailbox, name)
 void addOutputValueTile(uint8_t_Mailbox mailbox, String name);
 void addOutputValueTile(double_Mailbox mailbox, String name);
 void addOutputValueTile(float_Mailbox mailbox, String name);
 Sample:
 addOutputValueTile(double_Mailbox.d0, "333");
 
 >>> addInputIncDecTile(mailbox, name, SmallValueChange, LargeValueChange, InitValue)
 void addInputIncDecTile(uint8_t_Mailbox mailbox, String name, int small, int large, int initValue);
 void addInputIncDecTile(double_Mailbox mailbox, String name, double small, double large, double initValue);
 void addInputIncDecTile(float_Mailbox mailbox, String name, float small, float large, float initValue);
 Sample:
 addInputIncDecTile(uint8_t_Mailbox.u0, "it's u0", 1, 2, 30);
 addInputIncDecTile(double_Mailbox.d0, "it's d0", 1, 2, 30);
 
 >>> addElapsedTime() 
 void addElapsedTime() 
 Sample:
 addElapsedTime() 
 
 >>> addChart(NumOfSamplesToBeShown, PixelDistanceBetweenEveryPoint, height, LowerLimit, UpperLimit)
 void addChart(int SampleSize_, int HorizontalSpacing, int chart_height, double LowerBound, double UpperBound)
 Sample:
 addChart(100, 5, -5, 100, 105);
 
 >>> addLine(mailbox, name, color)
 void addLine(float_Mailbox mailbox, String name, color c);
 void addLine(uint8_t_Mailbox mailbox, String name, color c);
 void addLine(double_Mailbox mailbox, String name, color c);
 Sample:
 addLine(double_Mailbox.d0, "d0", RED);
 addLine(double_Mailbox.d1, "d1", BLUE);
 addLine(double_Mailbox.d2, "d2", WHITE);
 addLine(double_Mailbox.d3, "d3", YELLOW);
 
 >> addLineBreak()
 void addLineBreak()
 Sample:
 addLineBreak();
 */

final String EnvPath = "/Users/JosephYim/Desktop/";
final boolean SameFile = true;

//final String BTtarget = "tty.MORRIS";
final String BTtarget = "tty.COLUMN";

void init() { // add tiles here 

    //addOutputValueTile(Mailbox.f0, "master slope");
    //addOutputValueTile(Mailbox.f1, "slave slope");
    //addOutputValueTile(Mailbox.f2, "middle slope");
    //addOutputValueTile(Mailbox.i2, "9ntt");
    //addOutputValueTile(Mailbox.u3, "echo 3");

    addOutputValueTile(Mailbox.b0, "b0");
    addOutputValueTile(Mailbox.b1, "b1");
    
    addElapsedTime();
    addOutputValueTile(Mailbox.u0, "u0");
    addButtonTile(Mailbox.b1, "b1");
    
    addLineBreak(); // break into next column
    
    addCameraGraph();
    
    //addInputIncDecTile(Mailbox.i0, "i0", 1, 1, 0);
    //addInputIncDecTile(Mailbox.f0, "f0", 1, 1, 0);
    //addInputIncDecTile(Mailbox.d0, "d0", 1, 1, 0);
    //addInputIncDecTile(Mailbox.i0, "i0", 1, 1, 0);
    //addLineBreak(); // break into next column

    //addChart(100, 5, 600, -1, 255); // << start a new chart, i.e. chart 1

    ////DataArray_double[Mailbox.d29.ordinal()] = 100;
    ////addLine(Mailbox.d29, "constant line", BLACK);

    //addLine(Mailbox.u0, "trig", BLACK);
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
}

int tick = 0;
boolean t = false;
void cycle() {
    tick++;
    if (tick % 10 == 0) {
        t = !t;
        //uart.Send_bool(bool_MailBox.b1, t);
    }
}

//Path path = FileSystems.getDefault().getPath(".");
final String EnvPath = System.getProperty("user.dir");
//"/Users/JosephYim/Desktop/";
final boolean SameFile = true;

//final String BTtarget = "tty.MORRIS";
final String BTtarget = "tty.JOSEPH";
//final String BTtarget = "tty.COLUMN";


void init() { // add tiles here 

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
void cycle() {
    tick++;
    if (tick % 10 == 0) {
        t = !t;
        //uart.Send_bool(bool_MailBox.b1, t);
    }
}

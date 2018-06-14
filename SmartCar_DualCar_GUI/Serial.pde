import processing.serial.*;
import java.util.*;
import java.nio.ByteBuffer;

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

    String to8BitBinary(byte t) {
        String x = Integer.toBinaryString(t);
        if (x.length() < 8) {
            return ("00000000" + x).substring(x.length());
        } else {
            return x.substring(24, 32);
        }
    }

    void tSerialEvent() {
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

    boolean isOdd(Byte t0, Byte t1, Byte t2, Byte t3) {
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

    void SendWrapper(DATA_TYPE DataType, int MailBox, byte byte0, byte byte1, boolean sendImmediateBool) {
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

    void Send_uint8_t(uint8_t_MailBox MailBox, int num) {
        SendWrapper(DATA_TYPE.UINT8_T, MailBox.ordinal(), (byte) num, (byte) 0, false);

        print("\nUART: Send_uint8_t, ");
        print("MailBox: " + MailBox + " Value is " + num + "\n");
    }

    void Send_bool(bool_MailBox MailBox, boolean num) {
        SendWrapper(DATA_TYPE.UINT8_T, uint8_t_MailBox.BOOLEAN.ordinal(), (byte) MailBox.ordinal(), (byte)(num == true ? 1 : 0), false);

        print("\nUART: Send_bool, ");
        print("MailBox: " + MailBox + " Value is " + (num == true ? "True" : "False") + "\n");
    }

    void Send_double(double_MailBox MailBox, double num) {
        byte[] doublePtr = new byte[8];
        ByteBuffer.wrap(doublePtr).putDouble(num);
        SendWrapper(DATA_TYPE.BUFFER, 0, doublePtr[7], doublePtr[6], false);
        SendWrapper(DATA_TYPE.BUFFER, 1, doublePtr[5], doublePtr[4], false);
        SendWrapper(DATA_TYPE.BUFFER, 2, doublePtr[3], doublePtr[2], false);
        SendWrapper(DATA_TYPE.DOUBLE, MailBox.ordinal(), doublePtr[1], doublePtr[0], false);

        print("\nUART: Send_double, ");
        print("MailBox: " + MailBox + " Value is " + num + "\n");
    }

    void Send_float(float_MailBox MailBox, float num) {
        byte[] floatPtr = new byte[4];
        ByteBuffer.wrap(floatPtr).putFloat(num);
        SendWrapper(DATA_TYPE.BUFFER, 0, floatPtr[3], floatPtr[2], false);
        SendWrapper(DATA_TYPE.FLOAT, MailBox.ordinal(), floatPtr[1], floatPtr[0], false);

        print("\nUART: Send_float, ");
        print("MailBox: " + MailBox + " Value is " + num + "\n");
    }

    void Send_int(int_MailBox MailBox, int num) {
        byte[] intPtr = new byte[4];
        ByteBuffer.wrap(intPtr).putInt(num);
        SendWrapper(DATA_TYPE.BUFFER, 0, intPtr[3], intPtr[2], false);
        SendWrapper(DATA_TYPE.INT, MailBox.ordinal(), intPtr[1], intPtr[0], false);

        print("\nUART: Send_int, ");
        print("MailBox: " + MailBox + " Value is " + num + "\n");
    }

    void acker(byte code) {
        //SendWrapper(DATA_TYPE.SYSTEM, SYSTEM_MSG.ack.ordinal(), code, (byte) 0, true);
        //print("\nAck sent.\n");
    }
    
    void close() {
        m_port.stop();
    }
}

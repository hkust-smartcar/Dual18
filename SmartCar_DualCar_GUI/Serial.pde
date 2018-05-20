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

    public Byte[] dataBuffer;

    ArrayList<PACKAGE> SendImmediate;
    ArrayList<PACKAGE> SendBuffer;

    final Byte StartCode0 = -2; // 0b11111110
    final Byte StartCode1 = -1; // 0b11111111

    int SendCooling;

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

        dataBuffer = new Byte[LOCAL_BUFFER_MAX];

        SendImmediate = new ArrayList<PACKAGE>(0);
        SendBuffer = new ArrayList<PACKAGE>(0);
    }

    void tSerialEvent() {
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

                    if (DataType == DATA_TYPE.BUFFER) {
                        if (MailBox_ < (LOCAL_BUFFER_MAX/2)) {
                            dataBuffer[MailBox_*2] = RxBuffer[2];
                            dataBuffer[MailBox_*2+1] = RxBuffer[3];
                        }
                        acker(RxBuffer[1]);
                    } else if (DataType == DATA_TYPE.DOUBLE) {
                        if (DataArray_double.length > MailBox_) {
                            byte[] byteArr = new byte[8];
                            for (int i = 0; i < 6; i++) {
                                byteArr[7-i] = dataBuffer[i];
                            }
                            byteArr[1] = RxBuffer[2];
                            byteArr[0] = RxBuffer[3];
                            DataArray_double[MailBox_] = ByteBuffer.wrap(byteArr).getDouble();
                        }
                        acker(RxBuffer[1]);
                    }  else if (DataType == DATA_TYPE.FLOAT) {
                        
                        if (DataArray_float.length > MailBox_) {
                            byte[] byteArr = new byte[4];
                            byteArr[3] = dataBuffer[1];
                            byteArr[2] = dataBuffer[0];
                            byteArr[1] = RxBuffer[2];
                            byteArr[0] = RxBuffer[3];
                            DataArray_float[MailBox_] = ByteBuffer.wrap(byteArr).getFloat();
                        }
                        acker(RxBuffer[1]);
                    } else if (DataType == DATA_TYPE.UINT8_T) {
                    
                        if (DataArray_uint8_t.length > MailBox_) {
                            
                            DataArray_uint8_t[MailBox_] = RxBuffer[2];
                            if (DataArray_uint8_t[MailBox_] < 0) {
                                DataArray_uint8_t[MailBox_] += 256;
                            }
                        }
                        acker(RxBuffer[1]);
                    } else if (DataType == DATA_TYPE.SYSTEM) {
                        if (SYSTEM_MSG.lastTerm.ordinal() >= MailBox_) {
                            if (MailBox_ == SYSTEM_MSG.ack.ordinal()) {
                                if (SendBuffer.get(0).COMMAND_CODE == RxBuffer[2]) {
                                    // correct ack
                                    SendBuffer.remove(0);
                                    SendCooling = 0;
                                } else {
                                    // wrong ack, ignore it
                                }
                                // ack no need to ack ack
                            } else if (MailBox_ == SYSTEM_MSG.sayHi.ordinal()) {
                                acker(RxBuffer[1]);
                                for (int i = 0; i < tiles.size(); i++) {
                                    tiles.get(i).serialSend();
                                }
                            } else if (MailBox_ == SYSTEM_MSG.elpasedTime.ordinal()) {
                                int upperHalf = RxBuffer[2];
                                int lowerHalf = RxBuffer[3];
                                upperHalf += upperHalf < 0 ? 256: 0;
                                lowerHalf += lowerHalf < 0 ? 256: 0;

                                SYSTEM_MSG_elpasedTime = (upperHalf<<8) + lowerHalf;
                                acker(RxBuffer[1]);
                            }
                        }
                    }

                    // info extracted and reset the buffer
                    RxBuffer_Cursor = 0;
                } else {
                    // parity check failed
                    // discard the first one and shift everything left
                    RxBuffer[0] = RxBuffer[1];
                    RxBuffer[1] = RxBuffer[2];
                    RxBuffer[2] = RxBuffer[3];
                    RxBuffer[3] = 0;
                    RxBuffer_Cursor = 3;

                    if (!(DATA_TYPE_index >= 0 && DATA_TYPE_index <= 7)) {
                        print("wrong data type   ");
                    }

                    if (!(RxBuffer[0] == StartCode0 || RxBuffer[0] == StartCode1)) {
                        print("wrong start code   ");
                    }

                    if (!(isOdd(RxBuffer[0], RxBuffer[1], RxBuffer[2], RxBuffer[3]))) {
                        print("Parity check failed la " + DATA_TYPE_index);
                    }

                    println();
                }
            } else {
                // rxBuffer is not full
                RxBuffer[RxBuffer_Cursor] = m_port.readBytes(1)[0];
                RxBuffer_Cursor++;
            }

            //print(RxBuffer[0] < 0 ? RxBuffer[0] + 256 : RxBuffer[0]);
            //print(" ");
            //print(RxBuffer[1] < 0 ? RxBuffer[1] + 256 : RxBuffer[1]);
            //print(" ");
            //print(RxBuffer[2] < 0 ? RxBuffer[2] + 256 : RxBuffer[2]);
            //print(" ");
            //print(RxBuffer[3] < 0 ? RxBuffer[3] + 256 : RxBuffer[3]);
            //println(" ");
        }


        if (SendBuffer.size() != 0 && SendCooling == 0) {
            PACKAGE pkg = SendBuffer.get(0);
            byte[] pkgByte = new byte[4];
            pkgByte[0] = pkg.START_CODE;
            pkgByte[1] = pkg.COMMAND_CODE;
            pkgByte[2] = pkg.Data_0;
            pkgByte[3] = pkg.Data_1;
            m_port.write(pkgByte);
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
        }
    }

    boolean isOdd(Byte t0, Byte t1, Byte t2, Byte t3) {
        int t = 0;
        t ^= t0;
        t ^= t1 << 8;
        t ^= t2 << 16;
        t ^= t3 << 24;

        t ^= t >> 16;
        t ^= t >> 8;
        t ^= t >> 4;
        t ^= t >> 2;
        t ^= t >> 1;

        return ((~t) & 1) == 1;
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
        if (sendImmediateBool == true) {}
        //if (sendImmediateBool == true) {
        //    SendImmediate.add(SendImmediate.size(), pkg);
        //} else {
        //    SendBuffer.add(SendBuffer.size(), pkg);
        //}
    }

    void Send_uint8_t(uint8_t_MailBox MailBox, int num) {
        SendWrapper(DATA_TYPE.UINT8_T, MailBox.ordinal(), (byte) num, (byte) 0, false);
    }

    void Send_double(double_MailBox MailBox, double num) {
        byte[] doublePtr = new byte[8];
        ByteBuffer.wrap(doublePtr).putDouble(num);
        SendWrapper(DATA_TYPE.BUFFER, 0, doublePtr[7], doublePtr[6], false);
        SendWrapper(DATA_TYPE.BUFFER, 1, doublePtr[5], doublePtr[4], false);
        SendWrapper(DATA_TYPE.BUFFER, 2, doublePtr[3], doublePtr[2], false);
        SendWrapper(DATA_TYPE.DOUBLE, MailBox.ordinal(), doublePtr[1], doublePtr[0], false);
    }

    void Send_float(float_MailBox MailBox, float num) {
        byte[] floatPtr = new byte[4];
        ByteBuffer.wrap(floatPtr).putFloat(num);
        SendWrapper(DATA_TYPE.BUFFER, 0, floatPtr[3], floatPtr[2], false);
        SendWrapper(DATA_TYPE.FLOAT, MailBox.ordinal(), floatPtr[1], floatPtr[0], false);
    }

    void acker(byte code) {
        SendWrapper(DATA_TYPE.SYSTEM, SYSTEM_MSG.ack.ordinal(), code, (byte) 0, true);
    }
}

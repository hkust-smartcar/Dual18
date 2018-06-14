class InputBox extends GUI_Raw {
    String m_Value;
    boolean isHide;
    private final int label_X_Indent = 10;
    private DATA_TYPE m_ValueDataType = null;
    private uint8_t_MailBox uint8_t_MailBox_id = null;
    private double_MailBox double_MailBox_id = null;
    private float_MailBox float_MailBox_id = null;
    private int_MailBox int_MailBox_id = null;

    InputBox() {
        super();
        m_Value = "0";
        m_Width = 200;
        m_Height = (int) textAscent() + (int) textDescent();
        isHide = true;
    };
    @Override void setValue(int int_) {
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
        }
        m_Value = "";
    };
    @Override void setValue(String value_) {
        setPos(Integer.parseInt(value_.split(",")[0]), Integer.parseInt(value_.split(",")[1]));
    };
    @Override void tDraw() {
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
    @Override void tKeyPressed() {
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
}

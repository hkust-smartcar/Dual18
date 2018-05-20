class BreakColumn implements GUI_interface {
    private int m_TopLeftX = 0, m_TopLeftY = 0;
    private int m_Height = 0, m_Width = 0;
    private boolean m_NeedBackground = false;

    BreakColumn(boolean needBG) {
        m_Height = 100000000;
        m_Width = 20;
        m_NeedBackground = needBG;
    }

    void tDraw() {
        noStroke();

        if (m_NeedBackground == true) {
            fill(TILE_BACKGROUND_COLOR);
            rect(m_TopLeftX, m_TopLeftY, m_Width, m_Height, RECT_ANGLE_RADIUS);
        }
    }

    void over() {
    };
    void onClick() {
    };
    void setValue(String value_) {
    };
    void setValue(double double_) {
    };
    void setValue(float float_) {
    };
    void setValue(int int_) {
    };
    void setBackgroundFill(boolean needBackground) {
        m_NeedBackground = needBackground;
    };
    int getHeight() {
        return m_Height;
    };
    int getWidth() {
        return m_Width;
    };
    String getValue() {
        return "";
    };
    void setPos(int topLeftX_, int topLeftY_) {
        m_TopLeftX = topLeftX_;
        m_TopLeftY = topLeftY_;
    };
    void serialSend() {
    };
    void setTextColor(color TextColor_) {
    };
}

class TextLabel implements GUI_interface {

    private int m_TopLeftX = 0, m_TopLeftY = 0;
    private int m_Height = 0, m_Width = 0;
    private final int label_X_Indent = 10;
    private String label = "Empty";
    private boolean m_NeedBackground = true;
    private color m_TextColor = TEXT_COLOR;

    TextLabel(int width_, String label_) {
        textSize(fontSize);
        m_Height = (int) textAscent() + (int) textDescent();
        m_Width = width_;
        label = label_;
    }

    void tDraw() {
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

    void over() {
    };
    void onClick() {
    };
    void setValue(String value_) {
        label = value_;
    };
    void setValue(double double_) {
        label = Double.toString(double_);
    };
    void setValue(float float_) {
        label = Float.toString(float_);
    };
    void setValue(int int_) {
        label = Integer.toString(int_);
    };
    void setBackgroundFill(boolean needBackground) {
        m_NeedBackground = needBackground;
    };
    int getHeight() {
        return m_Height;
    };
    int getWidth() {
        return m_Width;
    };
    String getValue() {
        return label;
    };
    void setPos(int topLeftX_, int topLeftY_) {
        m_TopLeftX = topLeftX_;
        m_TopLeftY = topLeftY_;
    };
    void serialSend() {
    };
    void setTextColor(color TextColor_) {
        m_TextColor = TextColor_;
    };
}

class Button implements GUI_interface {
    private int m_TopLeftX = 0, m_TopLeftY = 0;
    private int m_Height = 0, m_Width = 0;
    private color m_TextColor = TEXT_COLOR;
    private String text;
    boolean isHovering = false;

    Button(String text_) {
        m_TopLeftX = 0;
        m_TopLeftY = 0;
        m_Width = (int) textWidth(text_+" ");
        textSize(fontSize);
        m_Height = (int) textAscent() + (int) textDescent();
        text = text_;

        tDraw();
    }

    Button(int topLeftX_, int topLeftY_, String text_) {
        m_TopLeftX = topLeftX_;
        m_TopLeftY = topLeftY_;
        m_Width = (int) textWidth(text_+" ");
        textSize(fontSize);
        m_Height = (int) textAscent() + (int) textDescent();
        text = text_;

        tDraw();
    }

    Button(int topLeftX_, int topLeftY_, int width_, int height_, String text_) {
        m_TopLeftX = topLeftX_;
        m_TopLeftY = topLeftY_;
        m_Width = width_;
        m_Height = height_;
        text = text_;

        tDraw();
    }

    void tDraw() {
        // this might casue extra resouces consumption
        if (isHovering) {
            fill(BUTTON_HOVERING_COLOR);
        } else {
            fill(BUTTON_NORMAL_COLOR);
        }

        noStroke();
        rect(m_TopLeftX, m_TopLeftY, m_Width, m_Height, RECT_ANGLE_RADIUS);

        fill(m_TextColor);
        textSize(fontSize);
        text(text, m_TopLeftX, m_TopLeftY, m_Width, m_Height);
    }
    void over() {
        if (mouseX >= m_TopLeftX && mouseX <= m_TopLeftX+m_Width && 
            mouseY >= m_TopLeftY && mouseY <= m_TopLeftY+m_Height) {
            isHovering = true;
        } else {
            isHovering = false;
        }
    }
    void onClick() {
    }
    void setValue(String value_) {
        text = value_;
    };
    void setValue(double double_) {
        text = Double.toString(double_);
    };
    void setValue(float float_) {
        text = Float.toString(float_);
    };
    void setValue(int int_) {
        text = Integer.toString(int_);
    };
    void setBackgroundFill(boolean needBackground) {
    };
    int getHeight() {
        return m_Height;
    };
    int getWidth() {
        return m_Width;
    };
    String getValue() {
        return text;
    };
    void setPos(int topLeftX_, int topLeftY_) {
        m_TopLeftX = topLeftX_;
        m_TopLeftY = topLeftY_;
    };
    void serialSend() {
    };
    void setTextColor(color TextColor_) {
        m_TextColor = TextColor_;
    };
}

class OutputValueTile implements GUI_interface {

    private int m_TopLeftX = 0, m_TopLeftY = 0;
    private int m_Height = 0, m_Width = 200;

    private TextLabel m_DataTypeLabel;
    private TextLabel m_ValueLabel;

    private DATA_TYPE m_ValueDataType;

    private uint8_t_MailBox uint8_t_MailBox_id = null;
    private double_MailBox double_MailBox_id = null;
    private float_MailBox float_MailBox_id = null;

    private color m_TextColor = TEXT_COLOR;

    OutputValueTile(uint8_t_MailBox mail_id_) {
        uint8_t_MailBox_id = mail_id_;
        m_ValueDataType = DATA_TYPE.UINT8_T;

        textSize(fontSize);
        m_DataTypeLabel = new TextLabel(m_Width, m_ValueDataType.name() + ": " + uint8_t_MailBox_id.name());
        m_ValueLabel = new TextLabel(m_Width, Integer.toString(DataArray_uint8_t[uint8_t_MailBox_id.ordinal()]));

        m_DataTypeLabel.setBackgroundFill(false);
        m_ValueLabel.setBackgroundFill(false);

        m_Height = m_DataTypeLabel.getHeight() + m_ValueLabel.getHeight();
    }

    OutputValueTile(double_MailBox mail_id_) {
        double_MailBox_id = mail_id_;
        m_ValueDataType = DATA_TYPE.DOUBLE;

        textSize(fontSize);
        m_DataTypeLabel = new TextLabel(m_Width, m_ValueDataType.name() + ": " + double_MailBox_id.name());
        m_ValueLabel = new TextLabel(m_Width, Double.toString(DataArray_double[double_MailBox_id.ordinal()]));

        m_DataTypeLabel.setBackgroundFill(false);
        m_ValueLabel.setBackgroundFill(false);

        m_Height = m_DataTypeLabel.getHeight() + m_ValueLabel.getHeight();
    }

    OutputValueTile(float_MailBox mail_id_) {
        float_MailBox_id = mail_id_;
        m_ValueDataType = DATA_TYPE.FLOAT;

        textSize(fontSize);
        m_DataTypeLabel = new TextLabel(m_Width, m_ValueDataType.name() + ": " + float_MailBox_id.name());
        m_ValueLabel = new TextLabel(m_Width, Float.toString(DataArray_float[float_MailBox_id.ordinal()]));

        m_DataTypeLabel.setBackgroundFill(false);
        m_ValueLabel.setBackgroundFill(false);

        m_Height = m_DataTypeLabel.getHeight() + m_ValueLabel.getHeight();
    }

    void tDraw() {
        if (m_ValueDataType == DATA_TYPE.UINT8_T) {
            m_ValueLabel.setValue(Integer.toString(DataArray_uint8_t[uint8_t_MailBox_id.ordinal()]));
        } else if (m_ValueDataType == DATA_TYPE.DOUBLE) {
            m_ValueLabel.setValue(Double.toString(DataArray_double[double_MailBox_id.ordinal()]));
        } else if (m_ValueDataType == DATA_TYPE.FLOAT) {
            m_ValueLabel.setValue(Float.toString(DataArray_float[float_MailBox_id.ordinal()]));
        }

        fill(TILE_BACKGROUND_COLOR);
        rect(m_TopLeftX, m_TopLeftY, m_Width, m_Height, RECT_ANGLE_RADIUS);

        m_DataTypeLabel.tDraw();
        m_ValueLabel.tDraw();
    }
    void over() {
    };
    void onClick() {
    };
    void setValue(String value_) {
        m_DataTypeLabel.setValue(value_);
    };
    void setValue(double double_) {
        m_DataTypeLabel.setValue(double_);
    };
    void setValue(float float_) {
        m_DataTypeLabel.setValue(float_);
    };
    void setValue(int int_) {
        m_DataTypeLabel.setValue(int_);
    };
    void setBackgroundFill(boolean needBackground) {
    };
    int getHeight() {
        return m_Height;
    };
    int getWidth() {
        return m_Width;
    };
    String getValue() {
        return "";
    };
    void setPos(int topLeftX_, int topLeftY_) {
        m_TopLeftX = topLeftX_;
        m_TopLeftY = topLeftY_;

        m_DataTypeLabel.setPos(m_TopLeftX, m_TopLeftY);
        m_ValueLabel.setPos(m_TopLeftX, m_TopLeftY+m_DataTypeLabel.getHeight());
    };
    void serialSend() {
    };
    void setTextColor(color TextColor_) {
        m_TextColor = TextColor_;
        m_DataTypeLabel.setTextColor(m_TextColor);
        m_ValueLabel.setTextColor(m_TextColor);
    };
}

class InputIncDecTile implements GUI_interface {

    private final int m_ButHeight = 30, m_ButWidth = 40;
    private final int m_ButHorizontalSpacing = 10;
    private int m_TopLeftX = 0, m_TopLeftY = 0;
    private int m_Height = 0, m_Width = 200;

    private double m_SmallChange = 0, m_LargeChange = 0;

    private DATA_TYPE m_DataType;
    private uint8_t_MailBox uint8_t_MailBox_id = null;
    private double_MailBox double_MailBox_id = null;
    private float_MailBox float_MailBox_id = null;

    private TextLabel m_DataTypeLabel, m_ValueLabel;
    private Button m_ButLargeDec, m_ButSmallDec, m_ButSmallInc, m_ButLargeInc;

    private color m_TextColor = TEXT_COLOR;
    boolean isHovering = false;

    private void init() {
        m_DataTypeLabel.setBackgroundFill(false);
        m_ValueLabel.setBackgroundFill(false);

        m_ButLargeDec = new Button(0, 0, m_ButWidth, m_ButHeight, " --");
        m_ButSmallDec = new Button(0, 0, m_ButWidth, m_ButHeight, "  -");
        m_ButSmallInc = new Button(0, 0, m_ButWidth, m_ButHeight, "  +");
        m_ButLargeInc = new Button(0, 0, m_ButWidth, m_ButHeight, " ++");
    }

    InputIncDecTile(uint8_t_MailBox mail_id_, int smallChange_, int largeChange_, int initValue) {
        m_SmallChange = (double) smallChange_;
        m_LargeChange = (double) largeChange_;

        m_DataType = DATA_TYPE.UINT8_T;
        uint8_t_MailBox_id = mail_id_;

        DataArray_uint8_t[uint8_t_MailBox_id.ordinal()] = initValue;

        m_DataTypeLabel = new TextLabel(m_Width, m_DataType.name() + ": " + uint8_t_MailBox_id.name());
        m_ValueLabel = new TextLabel(m_Width, Integer.toString(DataArray_uint8_t[uint8_t_MailBox_id.ordinal()]));

        init();
        m_Height = m_ButLargeDec.getHeight() + m_DataTypeLabel.getHeight() + m_ValueLabel.getHeight();
    }

    InputIncDecTile(double_MailBox mail_id_, double smallChange_, double largeChange_, double initValue) {
        m_SmallChange = smallChange_;
        m_LargeChange = largeChange_;

        m_DataType = DATA_TYPE.DOUBLE;
        double_MailBox_id = mail_id_;

        DataArray_double[double_MailBox_id.ordinal()] = initValue;

        m_DataTypeLabel = new TextLabel(m_Width, m_DataType.name() + ": " + double_MailBox_id.name());
        m_ValueLabel = new TextLabel(m_Width, Double.toString(DataArray_double[double_MailBox_id.ordinal()]));

        init();
        m_Height = m_ButLargeDec.getHeight() + m_DataTypeLabel.getHeight() + m_ValueLabel.getHeight();
    }

    InputIncDecTile(float_MailBox mail_id_, double smallChange_, double largeChange_, float initValue) {
        m_SmallChange = smallChange_;
        m_LargeChange = largeChange_;

        m_DataType = DATA_TYPE.FLOAT;
        float_MailBox_id = mail_id_;

        DataArray_float[float_MailBox_id.ordinal()] = initValue;

        m_DataTypeLabel = new TextLabel(m_Width, m_DataType.name() + ": " + float_MailBox_id.name());
        m_ValueLabel = new TextLabel(m_Width, Float.toString(DataArray_float[float_MailBox_id.ordinal()]));

        init();
        m_Height = m_ButLargeDec.getHeight() + m_DataTypeLabel.getHeight() + m_ValueLabel.getHeight();
    }

    void tDraw() {
        if (m_DataType == DATA_TYPE.UINT8_T) {
            m_ValueLabel.setValue(Integer.toString(DataArray_uint8_t[uint8_t_MailBox_id.ordinal()]));
        } else if (m_DataType == DATA_TYPE.DOUBLE) {
            m_ValueLabel.setValue(Double.toString(DataArray_double[double_MailBox_id.ordinal()]));
        }

        fill(TILE_BACKGROUND_COLOR);
        rect(m_TopLeftX, m_TopLeftY, m_Width, m_Height, RECT_ANGLE_RADIUS);

        m_DataTypeLabel.tDraw();
        m_ValueLabel.tDraw();

        m_ButLargeDec.tDraw();
        m_ButSmallDec.tDraw();
        m_ButSmallInc.tDraw();
        m_ButLargeInc.tDraw();
    }
    void over() {
        if (mouseX >= m_TopLeftX && mouseX <= m_TopLeftX+m_Width && 
            mouseY >= m_TopLeftY && mouseY <= m_TopLeftY+m_Height) {
            isHovering = true;
        } else {
            isHovering = false;
        }

        m_ButLargeDec.over();
        m_ButSmallDec.over();
        m_ButSmallInc.over();
        m_ButLargeInc.over();
    }

    void onClick() {
        if (isHovering) {
            if (m_DataType == DATA_TYPE.UINT8_T) {
                if (m_ButLargeDec.isHovering) {
                    DataArray_uint8_t[uint8_t_MailBox_id.ordinal()] -= (int) m_LargeChange;
                } else if (m_ButSmallDec.isHovering) {
                    DataArray_uint8_t[uint8_t_MailBox_id.ordinal()] -= (int) m_SmallChange;
                } else if (m_ButSmallInc.isHovering) {
                    DataArray_uint8_t[uint8_t_MailBox_id.ordinal()] += (int) m_SmallChange;
                } else if (m_ButLargeInc.isHovering) {
                    DataArray_uint8_t[uint8_t_MailBox_id.ordinal()] += (int) m_LargeChange;
                }

                if (DataArray_uint8_t[uint8_t_MailBox_id.ordinal()] <0) {
                    DataArray_uint8_t[uint8_t_MailBox_id.ordinal()] = 255;
                } else if (DataArray_uint8_t[uint8_t_MailBox_id.ordinal()] > 255) {
                    DataArray_uint8_t[uint8_t_MailBox_id.ordinal()] = 0;
                }

                m_ValueLabel.setValue(Integer.toString(DataArray_uint8_t[uint8_t_MailBox_id.ordinal()]));
                serialSend();
            } else if (m_DataType == DATA_TYPE.DOUBLE) {
                if (m_ButLargeDec.isHovering) {
                    DataArray_double[double_MailBox_id.ordinal()] -= m_LargeChange;
                } else if (m_ButSmallDec.isHovering) {
                    DataArray_double[double_MailBox_id.ordinal()] -= m_SmallChange;
                } else if (m_ButSmallInc.isHovering) {
                    DataArray_double[double_MailBox_id.ordinal()] += m_SmallChange;
                } else if (m_ButLargeInc.isHovering) {
                    DataArray_double[double_MailBox_id.ordinal()] += m_LargeChange;
                }

                m_ValueLabel.setValue(Double.toString(DataArray_double[double_MailBox_id.ordinal()]));
                serialSend();
            } else if (m_DataType == DATA_TYPE.FLOAT) {
                if (m_ButLargeDec.isHovering) {
                    DataArray_float[float_MailBox_id.ordinal()] -= (float) m_LargeChange;
                } else if (m_ButSmallDec.isHovering) {
                    DataArray_float[float_MailBox_id.ordinal()] -= (float) m_SmallChange;
                } else if (m_ButSmallInc.isHovering) {
                    DataArray_float[float_MailBox_id.ordinal()] += (float) m_SmallChange;
                } else if (m_ButLargeInc.isHovering) {
                    DataArray_float[float_MailBox_id.ordinal()] += (float) m_LargeChange;
                }

                m_ValueLabel.setValue(Float.toString(DataArray_float[float_MailBox_id.ordinal()]));
                serialSend();
            }
        }
    }

    void setValue(String value_) {
        m_DataTypeLabel.setValue(value_);
    };
    void setValue(double double_) {
        DataArray_double[double_MailBox_id.ordinal()] = double_;
    };
    void setValue(float float_) {
        DataArray_float[float_MailBox_id.ordinal()] = float_;
    };
    void setValue(int int_) {
        DataArray_uint8_t[uint8_t_MailBox_id.ordinal()] = int_;
    };
    void setBackgroundFill(boolean needBackground) {
    };
    int getHeight() {
        return m_Height;
    };
    int getWidth() {
        return m_Width;
    };
    String getValue() {
        return "";
    };
    void setPos(int topLeftX_, int topLeftY_) {
        m_TopLeftX = topLeftX_;
        m_TopLeftY = topLeftY_;

        m_DataTypeLabel.setPos(m_TopLeftX, m_TopLeftY);
        m_ValueLabel.setPos(m_TopLeftX, m_TopLeftY + m_DataTypeLabel.getHeight());
        textSize(fontSize);     

        int xOffset = (m_Width - (m_ButWidth*4 + m_ButHorizontalSpacing*3))/2;

        m_ButLargeDec.setPos(xOffset + m_TopLeftX, m_TopLeftY + m_DataTypeLabel.getHeight() + m_ValueLabel.getHeight());
        m_ButSmallDec.setPos(xOffset + m_TopLeftX + (m_ButWidth + m_ButHorizontalSpacing)*1, m_TopLeftY + m_DataTypeLabel.getHeight() + m_ValueLabel.getHeight());
        m_ButSmallInc.setPos(xOffset + m_TopLeftX + (m_ButWidth + m_ButHorizontalSpacing)*2, m_TopLeftY + m_DataTypeLabel.getHeight() + m_ValueLabel.getHeight());
        m_ButLargeInc.setPos(xOffset + m_TopLeftX + (m_ButWidth + m_ButHorizontalSpacing)*3, m_TopLeftY + m_DataTypeLabel.getHeight() + m_ValueLabel.getHeight());
    }
    void serialSend() {
        if (m_DataType == DATA_TYPE.UINT8_T) {
            uart.Send_uint8_t(uint8_t_MailBox_id, DataArray_uint8_t[uint8_t_MailBox_id.ordinal()]);
        } else if (m_DataType == DATA_TYPE.DOUBLE) {
            uart.Send_double(double_MailBox_id, DataArray_double[double_MailBox_id.ordinal()]);
        } else if (m_DataType == DATA_TYPE.FLOAT) {
            uart.Send_float(float_MailBox_id, DataArray_float[float_MailBox_id.ordinal()]);
        }
    };
    void setTextColor(color TextColor_) {
        m_TextColor = TextColor_;
        
        m_DataTypeLabel.setTextColor(m_TextColor);
        m_ValueLabel.setTextColor(m_TextColor);

        m_ButLargeDec.setTextColor(m_TextColor);
        m_ButSmallDec.setTextColor(m_TextColor);
        m_ButSmallInc.setTextColor(m_TextColor);
        m_ButLargeInc.setTextColor(m_TextColor);
    };
}

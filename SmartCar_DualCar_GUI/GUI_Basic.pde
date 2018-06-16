class BreakColumn extends GUI_Raw {
    BreakColumn(boolean needBG) {
        super();
        m_Height = 100000000;
        m_Width = 20;
        m_NeedBackground = needBG;
    }

    @Override void tDraw() {
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

    @Override void tDraw() {
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
    @Override void setValue(String value_) {
        label = value_;
    };
    @Override void setValue(double double_) {
        label = Double.toString(double_);
    };
    @Override void setValue(float float_) {
        label = Float.toString(float_);
    };
    @Override void setValue(int int_) {
        label = Integer.toString(int_);
    };
    @Override String getValue() {
        return label;
    };
}

class Button extends GUI_Raw {
    private String text;
    private color Hovering_Color = BUTTON_HOVERING_COLOR;
    private color Normal_Color = BUTTON_NORMAL_COLOR;

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
    @Override void tDraw() {
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
    @Override void setValue(String value_) {
        text = value_;
        if (text == null) {
            text = "null";
        }
    };
    @Override void setValue(double double_) {
        text = Double.toString(double_);
    };
    @Override void setValue(float float_) {
        text = Float.toString(float_);
    };
    @Override void setValue(int int_) {
        text = Integer.toString(int_);
    };
    @Override String getValue() {
        return text;
    };

    void setColor(color h_, color n_) {
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
    @Override void tDraw() {
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
    @Override void onClick() {
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
    @Override void setPos(int topLeftX_, int topLeftY_) {
        m_TopLeftX = topLeftX_;
        m_TopLeftY = topLeftY_;

        m_DataTypeLabel.setPos(topLeftX_, m_TopLeftY);
        m_ValueLabel.setPos(m_TopLeftX, m_TopLeftY + m_DataTypeLabel.getHeight());
    }
    @Override void setValue(String value_) {
        m_DataTypeLabel.setValue(value_);
    };
    @Override void setTextColor(color TextColor_) {
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

    @Override 
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
    @Override
        void onClick() {
        if (m_but.isHovering && deBounce == 0) {
            DataArray_bool[bool_MailBox_id.ordinal()] = !DataArray_bool[bool_MailBox_id.ordinal()];
            serialSend();

            deBounce = 4;
        }
    }
    @Override
        void over() {
        m_but.over();
    }
    @Override
        void setValue(String name) {
        m_var.setValue(name);
    }
    @Override
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

    @Override
        void serialSend() {
        if (uart != null)
            uart.Send_bool(bool_MailBox_id, DataArray_bool[bool_MailBox_id.ordinal()]);
    };
}

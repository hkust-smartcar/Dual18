import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.StandardOpenOption;

interface ScanLineChart_Interface extends GUI_interface {
    void setBoundary(double upperBound, double lowerBound);
    void setHorizontalSpacing(int spacing);
    void setSize(int width_, int height_);
    void getNewValue();
    void init(int SampleNumber_);

    void drawClean();
    void drawNew();

    String getLineName();
    color getLineColor();
    int getCursor();
    void setCursor(int cursor);
    void setPause(Boolean m_isPause);
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
    private color m_TextColor = TEXT_COLOR;
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

    void tDraw() {

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
    void over() {
        m_PauseBut.over();
        m_RestartBut.over();
    };
    void onClick() {
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
    void setValue(String value_) {
    };
    void setValue(double double_) {
    };
    void setValue(float float_) {
    };
    void setValue(int int_) {
    };
    void serialSend() {
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

        m_UpperBoundLabel.setPos(topLeftX_, topLeftY_);
        m_LowerBoundLabel.setPos(topLeftX_, topLeftY_+m_Height-m_LowerBoundLabel.getHeight());

        m_PauseBut.setPos(topLeftX_, (int) (topLeftY_+m_Height-2.5*m_PauseBut.getHeight()));
        m_RestartBut.setPos(topLeftX_, topLeftY_+m_Height-m_RestartBut.getHeight());

        for (int i = 0; i < lines.size(); i++) {
            lines.get(i).setPos(m_TopLeftX + m_AxisSpacing + 2, m_TopLeftY);
            linesLabel.get(i).setPos(m_TopLeftX, m_TopLeftY + (i+1) * m_LowerBoundLabel.getHeight());
        }
        needRedraw = true;
    };
    void init() {
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
    void setTextColor(color TextColor_) {
        m_TextColor = TextColor_;

        for (int i = 0; i < lines.size(); i++) {
            linesLabel.get(i).setTextColor(m_TextColor);
        }
    };

    void getFileName() {
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

    void appendToFile(String str) {
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
};

class ScanLineChart_Line implements ScanLineChart_Interface {
    private int m_HorizontalSpacing = 1;
    private int m_TopLeftX = 0, m_TopLeftY = 0;
    private int m_Height = 0, m_Width = 0;
    private DATA_TYPE m_DataType;
    private uint8_t_MailBox uint8_t_MailBox_id = null;
    private double_MailBox double_MailBox_id = null;
    private float_MailBox float_MailBox_id = null;
    private double m_LowerBound = 0, m_UpperBound = 1;
    private int m_SampleNumber = 0;
    private int[] uint8_t_value = null;
    private double[] double_value = null;
    private float[] float_value = null;
    private boolean needRedraw = true;
    private double m_NewValue = 0;
    private int m_Cursor = 0;
    private color m_LineColor = color(0, 0, 0);
    private color m_TextColor = TEXT_COLOR;
    private double m_PastPointY = 0;
    private Boolean m_isPause = false;
    private String name = "none";

    ScanLineChart_Line(uint8_t_MailBox mail_id_, color lineColor_) {
        m_DataType = DATA_TYPE.UINT8_T;
        uint8_t_MailBox_id = mail_id_;
        needRedraw = true;
        m_LineColor = lineColor_;
    }

    ScanLineChart_Line(double_MailBox mail_id_, color lineColor_) {
        m_DataType = DATA_TYPE.DOUBLE;
        double_MailBox_id = mail_id_;
        needRedraw = true;
        m_LineColor = lineColor_;
    }

    ScanLineChart_Line(float_MailBox mail_id_, color lineColor_) {
        m_DataType = DATA_TYPE.FLOAT;
        float_MailBox_id = mail_id_;
        needRedraw = true;
        m_LineColor = lineColor_;
    }

    void tDraw() {
        fill(m_LineColor);
        //if (m_DataType == DATA_TYPE.UINT8_T || m_DataType == DATA_TYPE.DOUBLE) {
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
                    }

                    tempYp = tempYp > m_LowerBound ? tempYp : m_LowerBound;
                    tempYp = tempYp < m_UpperBound ? tempYp : m_UpperBound;
                    tempYp = m_TopLeftY + m_Height - (tempYp - m_LowerBound) / BoundHeight * m_Height - 2;

                    //line(m_TopLeftX + m_HorizontalSpacing * (x-1), (float) tempYp, m_TopLeftX + m_HorizontalSpacing * x, (float) tempY);
                    line(m_TopLeftX + m_HorizontalSpacing * (x-1), (int) tempYp, m_TopLeftX + m_HorizontalSpacing * x, (int) tempY);
                }
                //ellipse(m_TopLeftX + m_HorizontalSpacing * x, (float) tempY, 2, 2);
                ellipse(m_TopLeftX + m_HorizontalSpacing * x, (int) tempY, 2, 2);
            }
            needRedraw = false;
        }
    };
    void over() {
    };
    void onClick() {
    };
    void setValue(String value_) {
        name = value_;
    };
    void setValue(double double_) {
        m_NewValue = double_;
        if (m_isPause == false) {
            m_Cursor++;
        }
        m_Cursor = m_Cursor == m_SampleNumber ? 0 : m_Cursor;
    };
    void setValue(float float_) {
        m_NewValue = float_;
        if (m_isPause == false) {
            m_Cursor++;
        }
        m_Cursor = m_Cursor == m_SampleNumber ? 0 : m_Cursor;
    };
    void setValue(int int_) {
        m_NewValue = (double) int_;
        if (m_isPause == false) {
            m_Cursor++;
        }
        m_Cursor = m_Cursor == m_SampleNumber ? 0 : m_Cursor;
    };
    void serialSend() {
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
        if (m_DataType == DATA_TYPE.UINT8_T) {
            return Integer.toString(DataArray_uint8_t[uint8_t_MailBox_id.ordinal()]);
        } else if (m_DataType == DATA_TYPE.DOUBLE) {
            return Double.toString(DataArray_double[double_MailBox_id.ordinal()]);
        } else if (m_DataType == DATA_TYPE.FLOAT) {
            return Float.toString(DataArray_float[float_MailBox_id.ordinal()]);
        }
        return "";
    };
    void setPos(int topLeftX_, int topLeftY_) {
        needRedraw = true;
        m_TopLeftX = topLeftX_;
        m_TopLeftY = topLeftY_;
    };

    void setBoundary(double upperBound, double lowerBound) {
        if (m_LowerBound > m_UpperBound) {
            println("Error! Lower bound > Upper bound");
            while (true) {
                // put here to halt the program
            }
        }
        m_LowerBound = lowerBound;
        m_UpperBound = upperBound;
    }
    void setHorizontalSpacing(int spacing) {
        m_HorizontalSpacing = spacing > 0 ? spacing : 1;
    };
    void setSize(int width_, int height_) {
        m_Width = width_;
        m_Height = height_;
    };
    void getNewValue() {
        if (m_DataType == DATA_TYPE.UINT8_T) {
            m_NewValue = (double) DataArray_uint8_t[uint8_t_MailBox_id.ordinal()];
        } else if (m_DataType == DATA_TYPE.DOUBLE) {
            m_NewValue = DataArray_double[double_MailBox_id.ordinal()];
        } else if (m_DataType == DATA_TYPE.FLOAT) {
            m_NewValue = DataArray_float[float_MailBox_id.ordinal()];
        }
        if (m_isPause == false) {
            m_Cursor++;
        }
        m_Cursor = m_Cursor == m_SampleNumber ? 0 : m_Cursor;
    };
    void init(int SampleNumber_) {

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
        }
    };
    void drawClean() {
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
    void drawNew() {
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
                }

                tempYp = tempYp > m_LowerBound ? tempYp : m_LowerBound;
                tempYp = tempYp < m_UpperBound ? tempYp : m_UpperBound;
                tempYp = m_TopLeftY + m_Height - (tempYp - m_LowerBound) / BoundHeight * m_Height - 2;

                //line((float) (x - m_HorizontalSpacing), (float) tempYp, (float) x, (float) tempY);
                line((int) (x - m_HorizontalSpacing), (int) tempYp, (int) x, (int) tempY);
            }

            //ellipse((float) x, (float) tempY, (float) 2, (float) 2);
            ellipse((int) x, (int) tempY, (int) 2, (int) 2);
        }
    };
    String getLineName() {
        return name;
    };
    color getLineColor() {
        return m_LineColor;
    };
    void setTextColor(color TextColor_) {
        if (m_TextColor != TextColor_) {
            m_TextColor = TextColor_;
        }
    };
    int getCursor() {
        return m_Cursor;
    };
    void setCursor(int cursor) {
        m_Cursor = cursor;
    };
    void setPause(Boolean isPause) {
        m_isPause = isPause;
    };
}

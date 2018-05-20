class ElapsedTime implements GUI_interface {
    private int m_TopLeftX = 0, m_TopLeftY = 0;
    private int m_Height = 0, m_Width = 0;

    private TextLabel m_AttributeLable;
    private TextLabel m_ValueLabel;

    private color m_TextColor = TEXT_COLOR;
    private int m_Last_SYSTEM_MSG_elpasedTime = 0;
    private int m_ConnLostCount = 0;
    private final int m_ConnLostCountMax = 2;

    ElapsedTime() {
        m_Width = 200;

        m_AttributeLable = new TextLabel(200, "MCU Run Time:");
        m_ValueLabel = new TextLabel(200, "Unconnected");

        m_AttributeLable.setBackgroundFill(false);
        m_ValueLabel.setBackgroundFill(false);

        m_AttributeLable.setTextColor(m_TextColor);
        m_ValueLabel.setTextColor(m_TextColor);

        m_Height = m_AttributeLable.getHeight()*2;
    };

    void tDraw() {
        noStroke();
        fill(TILE_BACKGROUND_COLOR);
        rect(m_TopLeftX, m_TopLeftY, m_Width, m_Height, RECT_ANGLE_RADIUS);
        
        if (SYSTEM_MSG_elpasedTime == m_Last_SYSTEM_MSG_elpasedTime) {
            m_ConnLostCount++;
        } else {
            m_ConnLostCount = 0;
        }

        if (m_ConnLostCount >= m_ConnLostCountMax) {
            m_ValueLabel.setValue("Conn Lost");
        } else {
            m_ValueLabel.setValue(Integer.toString(SYSTEM_MSG_elpasedTime) + " ?? unit");
        }
        
        m_Last_SYSTEM_MSG_elpasedTime = SYSTEM_MSG_elpasedTime;
        
        m_AttributeLable.tDraw();
        m_ValueLabel.tDraw();
    };
    void over() {
    };
    void onClick() {
    };
    void setValue(String value_) {
        SYSTEM_MSG_elpasedTime = Integer.parseInt(value_);
    };
    void setValue(double double_) {
        SYSTEM_MSG_elpasedTime = (int) Math.round(double_);
    };
    void setValue(float float_) {
        SYSTEM_MSG_elpasedTime = (int) Math.round(float_);
    };
    void setValue(int int_) {
        SYSTEM_MSG_elpasedTime = int_;
    };
    void serialSend() {
    };
    void setBackgroundFill(boolean needBackground) {
    };
    void setTextColor(color TextColor_) {
        m_TextColor = TextColor_;
    };
    int getHeight() {
        return m_Height;
    };
    int getWidth() {
        return m_Width;
    };
    String getValue() {
        return m_ValueLabel.getValue();
    };
    void setPos(int topLeftX_, int topLeftY_) {
        m_TopLeftX = topLeftX_;
        m_TopLeftY = topLeftY_;

        m_AttributeLable.setPos(m_TopLeftX, m_TopLeftY);
        m_ValueLabel.setPos(m_TopLeftX, m_TopLeftY+m_AttributeLable.getHeight());
    };
}

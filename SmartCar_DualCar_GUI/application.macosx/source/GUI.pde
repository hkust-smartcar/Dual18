public interface GUI_interface {
    int m_TopLeftX = 0, m_TopLeftY = 0;
    int m_Height = 0, m_Width = 0;

    void tDraw();
    void over();
    void onClick();
    void setValue(String value_);
    void setValue(double double_);
    void setValue(float float_);
    void setValue(int int_);
    void setValue(boolean bool_);
    void serialSend();
    void setBackgroundFill(boolean needBackground);
    void setTextColor(color TextColor_);
    void tKeyPressed();
    String getPos();

    int getHeight();
    int getWidth();
    String getValue();
    void setPos(int topLeftX_, int topLeftY_);
}

public class GUI_Raw implements GUI_interface {
    public int m_TopLeftX = 0, m_TopLeftY = 0;
    public int m_Height = 0, m_Width = 0;
    public boolean m_NeedBackground = false;
    public color m_TextColor = TEXT_COLOR;
    public boolean isHovering = false;

    GUI_Raw() {
    };
    void tDraw() {
    };
    void over() {
        if (mouseX >= m_TopLeftX && mouseX <= (m_TopLeftX+m_Width) && 
            mouseY >= m_TopLeftY && mouseY <= (m_TopLeftY+m_Height)) {
            isHovering = true;
        } else {
            isHovering = false;
        }
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
    void setValue(boolean bool_) {
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
        m_TextColor = TextColor_;
    };
    void tKeyPressed() {
    };
    String getPos() {
        return Integer.toString(m_TopLeftX) + "," + Integer.toString(m_TopLeftY);
    };
}

ArrayList<GUI_interface> tiles = new ArrayList<GUI_interface>();

int borderX = 40;
final int borderY = 40;
final int borderYLower = 40;
final int hSpacing = 20;
final int wSpacing = 20;

void tileAllocator() {
    int currentX = borderX;
    int currentY = borderY;

    int maxX = 0;

    for (int i = 0; i < tiles.size(); i++) {
        if ((tiles.get(i).getHeight() + currentY) > (height - borderY - borderYLower)) {
            currentX += maxX + wSpacing;
            currentY = borderY;
            maxX = 0;
        }

        tiles.get(i).setPos(currentX, currentY);
        currentY += tiles.get(i).getHeight() + hSpacing;

        maxX = (maxX < tiles.get(i).getWidth()) ? tiles.get(i).getWidth() : maxX;
    }

    MoveLeft.setPos(40, height - borderY - borderYLower+40);
    MoveCenter.setPos(40+60+20, height - borderY - borderYLower+40);
    MoveRight.setPos(40+60+20+20+90, height - borderY - borderYLower+40);

    UARTstatus.setPos(40+60+20+20+90+60+40, height - borderY - borderYLower+40);
    if (InputBoxObj != null)
        InputBoxObj.setPos(0, 0);

    print("\nGUI: tileAllocator \n");
}

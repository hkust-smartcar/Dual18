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
    void serialSend();
    void setBackgroundFill(boolean needBackground);
    void setTextColor(color TextColor_);

    int getHeight();
    int getWidth();
    String getValue();
    void setPos(int topLeftX_, int topLeftY_);
}

ArrayList<GUI_interface> tiles = new ArrayList<GUI_interface>();

final int borderX = 40;
final int borderY = 40;
final int hSpacing = 20;
final int wSpacing = 20;

void tileAllocator() {
    int currentX = borderX;
    int currentY = borderY;

    int maxX = 0;

    for (int i = 0; i < tiles.size(); i++) {
        if ((tiles.get(i).getHeight() + currentY) > (height - borderY)) {
            currentX += maxX + wSpacing;
            currentY = borderY;
            maxX = 0;
        }

        tiles.get(i).setPos(currentX, currentY);
        currentY += tiles.get(i).getHeight() + hSpacing;

        maxX = (maxX < tiles.get(i).getWidth()) ? tiles.get(i).getWidth() : maxX;
    }
}

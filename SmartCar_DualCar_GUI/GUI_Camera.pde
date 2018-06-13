class CameraDot extends GUI_Raw {
    private Boolean isDirty;
    private Boolean isDirty_n;
    private color ptColor;
    private int scale;
    private byte content;
    private byte content_n;

    CameraDot() {
        super();

        m_Width = 1;
        m_Height = 1;

        isDirty = false;
        isDirty_n = false;

        content = 0;
        content_n = 0;

        scale = 1;

        ptColor = BLACK;
    }

    void setClean() {
        isDirty = false;
    }

    void update() {
        isDirty = isDirty_n;
        content= content;
    }

    void setData(byte d_) {
        isDirty_n = true;
        content_n = d_;
    }

    void setScale(int _scale) {
        scale = _scale;
    }

    @Override
        void tDraw() {
        if (isDirty) {
            noStroke();
            fill(ptColor);
            for (int i = 0; i < 8; i++) {
                if (((content >> i) & 1) == 1) {
                    rect(m_TopLeftX + i * scale, m_TopLeftY, scale, scale);
                }
            }
        }
    }
}

class VectorGraphDot extends GUI_Raw {
    private Boolean isDirty;
    private Boolean isDirty_n;
    private int xCor, yCor;
    private int xCor_n, yCor_n;
    private color ptColor;
    private int scale;

    VectorGraphDot() {
        super();

        m_Width = 1;
        m_Height = 1;

        isDirty = false;
        isDirty_n = false;
        xCor = 0;
        yCor = 0;
        xCor_n = 0;
        yCor_n = 0;

        scale = 1;

        ptColor = RED;
    }

    void setClean() {
        isDirty = false;
    }

    void update() {
        isDirty = isDirty_n;
        xCor= xCor_n;
        yCor= yCor_n;
    }

    void setCor(int _x, int _y) {
        isDirty_n = true;
        xCor_n = _x;
        yCor_n = _y;
    }

    void setScale(int _scale) {
        scale = _scale;
    }

    @Override
        void tDraw() {
        if (isDirty) {
            noStroke();
            fill(ptColor);
            rect(m_TopLeftX + xCor * scale, m_TopLeftY + yCor * scale, scale, scale);
        } else {
        }
    }
}

class CameraGraph extends GUI_Raw {
    private int borderWidth = 20;
    private int scale = 7;
    private color GraphBG;

    CameraGraph() {
        super();
        m_Width = CAMERA_GRAPH_X * scale + borderWidth*2;
        m_Height = CAMERA_GRAPH_Y*scale + borderWidth*2;

        GraphBG = GRAY;

        if (VectorGraph_ArraryList != null) {
            int len = VectorGraph_ArraryList.size();
            for (int i = 0; i < len; i++) {
                VectorGraph_ArraryList.get(i).setScale(scale);
            }
        } else {
            println("GUI: Critical Error! VectorGraph_ArraryList is null!");
        }

        //if (CameraGraph_ArraryList != null) {
        //    int len = CameraGraph_ArraryList.size();
        //    for (int i = 0; i < len; i++) {
        //        CameraGraph_ArraryList.get(i).setScale(scale);
        //    }
        //} else {
        //    println("GUI: CameraGraph-CameraGraph_ArraryList is null!");
        //}
    }

    @Override
        void tDraw() {
        noStroke();

        fill(TILE_BACKGROUND_COLOR);
        rect(m_TopLeftX, m_TopLeftY, m_Width, m_Height, RECT_ANGLE_RADIUS);

        fill(GraphBG);
        rect(m_TopLeftX + borderWidth, m_TopLeftY + borderWidth, m_Width - borderWidth*2, m_Height - borderWidth*2, RECT_ANGLE_RADIUS);

        //if (CameraGraph_ArraryList != null) {
        //    int len = CameraGraph_ArraryList.size();
        //    for (int i = 0; i < len; i++) {
        //        CameraGraph_ArraryList.get(i).tDraw();
        //    }
        //} else {
        //    println("GUI: CameraGraph-CameraGraph_ArraryList is null!");
        //}

        if (VectorGraph_ArraryList != null) {
            int len = VectorGraph_ArraryList.size();
            for (int i = 0; i < len; i++) {
                VectorGraph_ArraryList.get(i).tDraw();
            }
        } else {
            println("GUI: CameraGraph-VectorGraph_ArraryList is null!");
        }
    }

    @Override
        void setPos(int topLeftX_, int topLeftY_) {
        m_TopLeftX = topLeftX_;
        m_TopLeftY = topLeftY_;

        if (VectorGraph_ArraryList != null) {
            println("GUI: VectorGraph-setPos!");

            int len = VectorGraph_ArraryList.size();
            for (int i = 0; i < len; i++) {
                VectorGraph_ArraryList.get(i).setPos(m_TopLeftX + borderWidth, m_TopLeftY + borderWidth);
            }
        } else {
            println("GUI: Critical Error! VectorGraph_ArraryList is null!");
        }

        //if (CameraGraph_ArraryList != null) {
        //    int len = CameraGraph_ArraryList.size();
        //    for (int x = 0; x < CAMERA_GRAPH_X/8; x++) {
        //        for (int y = 0; y < CAMERA_GRAPH_Y; y++) {
        //            println("x : " + x + ", y: " + y);
        //            CameraGraph_ArraryList.get(CAMERA_GRAPH_X*y/8 + x).
        //                setPos(m_TopLeftX + borderWidth + x*scale*8, 
        //                m_TopLeftY + borderWidth + y * scale);
        //        }
        //    }
        //} else {
        //    println("GUI: CameraGraph-CameraGraph_ArraryList is null!");
        //}
    };
}

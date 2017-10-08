package org.firstinspires.ftc.teamcode;

//enum that stores fixed positions for glyph wrist

public enum GlyphWristState
{
    //wrist position(encoder value)
    START(0),
    FRONT(-1890),
    BACK(1890);
    
    private final int wristEncoderPos;
    
    private GlyphWristState(int wristEncoderPos)
    {
        this.wristEncoderPos = wristEncoderPos;
    }
    
    public int getWristEncoderPos()
    {
        return wristEncoderPos;
    }
}
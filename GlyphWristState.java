package org.firstinspires.ftc.teamcode;

public enum GlyphWristState
{
    START(0),
    FRONT(360),
    BACK(-360);
    
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
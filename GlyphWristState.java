package org.firstinspires.ftc.teamcode;

//enum that stores fixed positions for glyph wrist

public enum GlyphWristState
{
    //wrist position(encoder value)
    START(0),
    FRONT(-1240), //was -1240
    BACK(1240), //was 1240
    RELIC_PICKUP(804), //was 804 waswas 1150
    RELIC_PLACE(80), //was 80 waswas 0
    RELIC_DONE(-800); //was -800 waswas -1100
    
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
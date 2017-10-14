package org.firstinspires.ftc.teamcode;

//enum that stores fixed positions for glyph arm

public enum GlyphArmState
{
    //arm position(encoder value, is it a front position)
    START(-30, true),
    FRONT_PICKUP(-30, true),
    FRONT1(186, true),
    FRONT2(688, true), //was 474
    FRONT3(1116, true), //was 916
    FRONT4(1600, true), //was 1392
    BACK4(3680, false), //was 3962
    BACK3(4171, false), //was 4416
    BACK2(4692, false), //was 4875
    BACK1(5150, false),
    BACK_PICKUP(5350, false),  //5384 max
    RELIC_PICKUP(3000, false),
    RELIC_PLACE(4470, false),
    RELIC_DONE(-30, true);
    
    private final int armEncoderPos;
    private final boolean isFrontPos;
    
    private GlyphArmState(int armEncoderPos, boolean isFrontPos)
    {
        this.armEncoderPos = armEncoderPos;
        this.isFrontPos = isFrontPos;
    }
    
    public int getArmEncoderPos()
    {
        return armEncoderPos;
    }
    
    public boolean isFrontPos()
    {
        return isFrontPos;
    }
}
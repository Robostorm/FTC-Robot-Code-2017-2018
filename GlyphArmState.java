package org.firstinspires.ftc.teamcode;

//enum that stores fixed positions for glyph arm

public enum GlyphArmState
{
    //arm position(encoder value, is it a front position)
    //START(0, true), //was -30
    FRONT_PICKUP(-15, true), //was -30
    FRONT1(186, true),
    FRONT_PICKUP_2(488, true),
    FRONT2(688, true), //was 474
    FRONT3(1116, true), //was 916
    FRONT4(1600, true), //was 1392
    BACK4(3680, false), //was 3962
    BACK3(4171, false), //was 4416
    BACK2(4692, false), //was 4875
    BACK1(5150, false),
    BACK_PICKUP(5350, false),  //5384 max
    RELIC_PICKUP(2327, false), //was 3000
    RELIC_PLACE(4500, false), //was 4327
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
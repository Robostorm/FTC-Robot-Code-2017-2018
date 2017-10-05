package org.firstinspires.ftc.teamcode;

public enum GlyphArmState
{
    /*START(0),
    FRONT1(0),
    FRONT2(300),
    FRONT3(600),
    FRONT4(900),
    BACK4(1900),
    BACK3(2200),
    BACK2(2500),
    BACK1(2800);*/

    /*start 0
    front1 0
    front2 474
    front3 916
    front4 1392
    back4 3962
    back3 4416
    back2 4875
    back1 5384
    end limit 5398 old
    */

    START(0, true),
    FRONT1(0, true),
    FRONT2(474, true),
    FRONT3(916, true),
    FRONT4(1392, true),
    BACK4(3962, false),
    BACK3(4416, false),
    BACK2(4875, false),
    BACK1(5384, false);
    
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
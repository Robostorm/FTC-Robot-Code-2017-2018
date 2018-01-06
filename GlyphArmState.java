package org.firstinspires.ftc.teamcode;

/**
 * Stores fixed positions for glyph/relic arm
 * @author Andrew Hollabaugh
 * @since 2017-10-08
 */
public enum GlyphArmState
{
    //format:
    //arm_position(encoderValue, isFrontPosition)

    //START(0, true), //was -30

    /**
     * Bottom front position for pickup up glyphs from front
     */
    FRONT_PICKUP(-15, true), //was -30

    /**
     * For placing glyphs in front of the robot in the bottom row
     */
    FRONT1(186, true),

    /**
     * For pickup up glyphs stacked 2 high
     */
    FRONT_PICKUP_2(488, true),

    /**
     * For placing glyphs in front of the robot in the 2nd row
     */
    FRONT2(688, true), //was 474

    /**
     * For placing glyphs in front of the robot in the 3rd row
     */
    FRONT3(1116, true), //was 916

    /**
     * For placing glyphs in front of the robot in the 4th row
     */
    FRONT4(1600, true), //was 1392

    /**
     * For placing glyphs from the back of the robot in the 4th row
     */
    BACK4(3680, false), //was 3962

    /**
     * For placing glyphs from the back of the robot in the 3rd row
     */
    BACK3(4171, false), //was 4416

    /**
     * For placing glyphs from the back of the robot in the 2nd row
     */
    BACK2(4692, false), //was 4875

    /**
     * For placing glyphs from the back of the robot in the 1st row
     */
    BACK1(5150, false),

    /**
     * Bottom back position for picking up glyphs from the back
     */
    BACK_PICKUP(5350, false),  //5384 max

    /**
     * For picking up the relic farthest from the recovery zone
     */
    RELIC_PICKUP(2024, false), //was 2327

    /**
     * For pickup up the relic closest to the recovery zone
     */
    RELIC_PICKUP2(1800, false), //was 2327

    /**
     * For placing the relic, fully extended
     */
    RELIC_PLACE(4450, false), //was 4500

    /**
     * For after relic has been placed
     */
    RELIC_DONE(-30, true);
    
    private final int armEncoderPos;
    private final boolean isFrontPos;
    
    GlyphArmState(int armEncoderPos, boolean isFrontPos)
    {
        this.armEncoderPos = armEncoderPos;
        this.isFrontPos = isFrontPos;
    }

    /**
     * Gets the arm position's encoder value
     * @return armEncoderPos - the encoder value of the position
     */
    public int getArmEncoderPos()
    {
        return armEncoderPos;
    }

    /**
     * Returns if the position is a front position
     * @return isFrontPos - true if front position; false if back position
     */
    public boolean isFrontPos()
    {
        return isFrontPos;
    }
}
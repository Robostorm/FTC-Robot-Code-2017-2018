package org.firstinspires.ftc.teamcode;

//enum that stores fixed positions for glyph wrist

/**
 * Stores fixed positions for wrist
 * @author Andrew Hollabaugh
 * @since 2017-10-08
 */
public enum GlyphWristState
{
    //wrist_position(encoder_value)

    //40:1 gearbox
    /*START(0),
    FRONT(-1240), //was -1240
    BACK(1240), //was 1240
    RELIC_PICKUP(804), //was 804 waswas 1150
    RELIC_PICKUP2(680),
    RELIC_PLACE(80), //was 80 waswas 0
    RELIC_DONE(-800); //was -800 waswas -1100*/

    //20:1 gearbox
    /**
     * Start (vertical) position
     */
    START(0),

    /**
     * Front position, horizontal
     */
    FRONT(1582), //was 625

    /**
     * Back position, horizontal
     */
    BACK(645), //was 1562 wawass -625

    /**
     * For picking up the relic farthest from the recovery zone
     */
    RELIC_PICKUP(-343), //was -370

    /**
     * For picking up the relic closest to the recovery zone
     */
    RELIC_PICKUP2(-255), // -267 was -320

    /**
     * For placing the relic; fully extended
     */
    RELIC_PLACE(-20), //was -40

    /**
     * For after the relic has been placed
     */
    RELIC_DONE(300);
    
    private final int wristEncoderPos;
    
    GlyphWristState(int wristEncoderPos)
    {
        this.wristEncoderPos = wristEncoderPos;
    }

    /**
     * Gets the wrist position's encoder value
     * @return wristEncoderPos - the encoder value of the position
     */
    public int getWristEncoderPos()
    {
        return wristEncoderPos;
    }
}
# FTC12601RobotCode2017
Robot code for FTC Team #12601's robot for 2017-2018 season

### Current classes
- GlyphArmState: enum that stores set encoder values for glyph/relic arm positions
- GlyphWristState: enum that stores set encoder values for wrist positions
- RRBotAutoGlyph: autonomous opmode that knocks the correct color jewel and places the glyph in the correct column
- RRBotAutoGlyphOld: depreciated
- RRBotAutoMultiGlyph: unfinished; gets second glyph from glyph pit
- RRBotAutoNoGlyph: autonomous opmode that knocks the correct color jewel and drives to safe zone
- RRBotGlyphArm: controls the robot's glyph/relic arm, wrist, and grabbers (largest and most complex class)
- RRBotHardware: hardware class that specifies the sensors/actuators on the robot and initializes them
- RRBotMecanumDrive: contains the algorithms for mecanum drive
- RRBotTeleop: teleop opmode
- RRBotTeleop2: depreciated/unused teleop class
- RRBotVuforiaClass: uses vuforia to recognize the pictograph VuMarks (copied and slightly modified example code)

package org.firstinspires.ftc.teamcode.utils;

//TODO: Where should the executeState() be? In the teleop class? pass it to movement controllers
public enum RobotStates {
    DRIVE_FORWARD,
    DRIVE_REVERSE,
    DRIVE_LEFT,
    DRIVE_RIGHT,
    DRIVE_BRAKE,

    ROTATE_CLOCKWISE,
    ROTATE_COUNTERCLOCKWISE,
    ROTATE_BRAKE,

    SLOW,

    RAISE_LIFT,
    LOWER_LIFT,

    INCREMENT_LIFT,
    DECREMENT_LIFT,

    OPEN_LOWER_GLYPH, //TODO: What if wanted to have the joystick value control the setting?
    CLOSE_LOWER_GLYPH,

    OPEN_UPPER_GLYPH,
    CLOSE_UPPER_GLYPH
}
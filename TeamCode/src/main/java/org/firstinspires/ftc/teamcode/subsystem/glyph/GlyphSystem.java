package org.firstinspires.ftc.teamcode.subsystem.glyph;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.consts.PrometheusConstants;
import org.firstinspires.ftc.teamcode.hardware.controllers.RobotServoController;
import org.firstinspires.ftc.teamcode.hardware.devices.RobotServo;
import org.firstinspires.ftc.teamcode.opmodes.GameMode;
import org.firstinspires.ftc.teamcode.subsystem.Subsystem;
import org.firstinspires.ftc.teamcode.utils.RobotStates;

import java.util.ArrayList;

public class GlyphSystem extends Subsystem {

    private RobotServoController glyphGrabberUpperServos = null;
    private RobotServoController glyphGrabberLowerServos = null;

    private RobotServo upperLeftServo = null;
    private RobotServo upperRightServo = null;
    private RobotServo lowerLeftServo = null;
    private RobotServo lowerRightServo = null;

    public GlyphSystem(Robot robot, GameMode gameMode) {
        super(robot);
        setGameMode(gameMode);

        init();
    }

    @Override
    public void init() {
//        glyphGrabberUpperServos = getRobot().addServoController(PrometheusConstants.GLYPH_SERVO_CONTRLLER);
//        glyphGrabberLowerServos = getRobot().addServoController(PrometheusConstants.GLYPH_SERVO_CONTRLLER);

        upperLeftServo = getRobot().addServo(PrometheusConstants.UPPER_LEFT_SERVO);
        upperRightServo = getRobot().addServo(PrometheusConstants.UPPER_RIGHT_SERVO);
        lowerLeftServo = getRobot().addServo(PrometheusConstants.LOWER_LEFT_SERVO);
        lowerRightServo = getRobot().addServo(PrometheusConstants.LOWER_RIGHT_SERVO);

        setDirections();
    }

    @Override
    public void handle(ArrayList<RobotStates> robotStates) {
        if (robotStates.contains(RobotStates.OPEN_LOWER_GLYPH)) {
            lowerLeftServo.setPosition(0.15);
            lowerRightServo.setPosition(0.81);
        }

        if (robotStates.contains(RobotStates.CLOSE_LOWER_GLYPH)) {
            lowerLeftServo.setPosition(0.02);
            lowerRightServo.setPosition(0.90);
        }

        if (robotStates.contains(RobotStates.OPEN_UPPER_GLYPH)) {
            upperLeftServo.setPosition(0.80);
            upperRightServo.setPosition(0.30);
        }

        if (robotStates.contains(RobotStates.CLOSE_UPPER_GLYPH)) {
            upperLeftServo.setPosition(0.93);
            upperRightServo.setPosition(0.20);
        }
    }

    @Override
    public void stop() {

    }

    //Disabled for now as not supported
    private void setDirections() {
//        lowerLeftServo.setDirection(Servo.Direction.FORWARD);
//        lowerRightServo.setDirection(Servo.Direction.REVERSE);
//        upperLeftServo.setDirection(Servo.Direction.FORWARD);
//        upperRightServo.setDirection(Servo.Direction.REVERSE);
    }

    public void initServos() {
        upperLeftServo.setPosition(0.06);
        upperRightServo.setPosition(0.44);
        lowerLeftServo.setPosition(0.13);
        lowerRightServo.setPosition(0.78);
    }

    public void openLowers() {
        lowerLeftServo.setPosition(0.13);
        lowerRightServo.setPosition(0.78);
    }

    public void openUppers() {
        upperLeftServo.setPosition(0.06);
        upperRightServo.setPosition(0.44);
    }

    public void closeLowers() {
        lowerLeftServo.setPosition(0.02);
        lowerRightServo.setPosition(0.90);
    }

    public void closeUppers() {
        upperLeftServo.setPosition(0.34);
        upperRightServo.setPosition(0.20);
    }
}

package org.firstinspires.ftc.teamcode.subsystem.glyph;

import com.qualcomm.robotcore.hardware.Servo;

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
        glyphGrabberUpperServos = getRobot().addServoController(PrometheusConstants.UPPER_GLYPH_SERVO_CONTRLLER);
        glyphGrabberLowerServos = getRobot().addServoController(PrometheusConstants.LOWER_GLYPH_SERVO_CONTRLLER);

        upperLeftServo = getRobot().addServo(glyphGrabberUpperServos, PrometheusConstants.UPPER_LEFT_SERVO);
        upperRightServo = getRobot().addServo(glyphGrabberUpperServos, PrometheusConstants.UPPER_RIGHT_SERVO);
        lowerLeftServo = getRobot().addServo(glyphGrabberLowerServos, PrometheusConstants.LOWER_LEFT_SERVO);
        lowerRightServo = getRobot().addServo(glyphGrabberUpperServos, PrometheusConstants.LOWER_RIGHT_SERVO);

        setDirections();
    }

    @Override
    public void handle(ArrayList<RobotStates> robotStates) {
        if (robotStates.contains(RobotStates.OPEN_LOWER_GLYPH)) {
            lowerLeftServo.setPosition(15.0);
            lowerRightServo.setPosition(15.0);
        }

        if (robotStates.contains(RobotStates.CLOSE_LOWER_GLYPH)) {
            lowerLeftServo.setPosition(0.0);
            lowerRightServo.setPosition(0.0);
        }

        if (robotStates.contains(RobotStates.OPEN_UPPER_GLYPH)) {
            upperLeftServo.setPosition(15.0);
            upperRightServo.setPosition(15.0);
        }

        if (robotStates.contains(RobotStates.CLOSE_UPPER_GLYPH)) {
            upperLeftServo.setPosition(0.0);
            upperRightServo.setPosition(0.0);
        }
    }

    @Override
    public void stop() {

    }

    private void setDirections() {
        lowerLeftServo.setDirection(Servo.Direction.FORWARD);
        lowerRightServo.setDirection(Servo.Direction.REVERSE);
        upperLeftServo.setDirection(Servo.Direction.FORWARD);
        upperRightServo.setDirection(Servo.Direction.REVERSE);
    }
}

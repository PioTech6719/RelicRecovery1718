package org.firstinspires.ftc.teamcode.opmodes.tele.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.gamepads.configs.PrometheusConfigOne;
import org.firstinspires.ftc.teamcode.hardware.robots.test.ServoLiftRobot;
import org.firstinspires.ftc.teamcode.opmodes.tele.BaseTeleOp;

@TeleOp(name = "Servo Lift Test", group = "test")
public class ServoLiftTest extends BaseTeleOp {

    private ServoLiftRobot servoLiftRobot = new ServoLiftRobot(this);
    private PrometheusConfigOne gamepadConfig = new PrometheusConfigOne(gamepad1, gamepad2);

    @Override
    public void initializeClass() {
        setRobot(servoLiftRobot);
        setGamepadConfig(gamepadConfig);
    }


    @Override
    public void loop() {
        servoLiftRobot.handleOperations(gamepadConfig.getStates());
    }
}

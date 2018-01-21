package org.firstinspires.ftc.teamcode.opmodes.auto.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Directions;
import org.firstinspires.ftc.teamcode.hardware.gamepads.Button;
import org.firstinspires.ftc.teamcode.hardware.gamepads.ButtonType;
import org.firstinspires.ftc.teamcode.hardware.robots.Prometheus;
import org.firstinspires.ftc.teamcode.match.Alliance;
import org.firstinspires.ftc.teamcode.opmodes.auto.BaseAutoOp;
import org.firstinspires.ftc.teamcode.subsystem.drive.DriveSystem;
import org.firstinspires.ftc.teamcode.subsystem.drive.drivecontroller.PID.PIDComplexity;
import org.firstinspires.ftc.teamcode.utils.PioTimer;

@Autonomous(name = "Sensor Test - SimplePID Move", group = "test")
public class SimplePIDMoveTest extends BaseAutoOp {

    private Alliance alliance = Alliance.BLUE;
    private Prometheus prometheus;

    private double distance = 10;

    @Override
    public void initializeClass() {
        prometheus = new Prometheus(this);

        setRobot(prometheus);
        setGamepadConfig(null);
        setAlliance(alliance);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        Button buttonX = new Button(gamepad1, ButtonType.X);
        Button buttonY = new Button(gamepad1, ButtonType.Y);
        Button buttonA = new Button(gamepad1, ButtonType.A);
        Button buttonB = new Button(gamepad1, ButtonType.B);
        Button buttonDUp = new Button(gamepad1, ButtonType.D_PAD_UP);
        Button buttonDDown = new Button(gamepad1, ButtonType.D_PAD_DOWN);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.x) {
                ((DriveSystem) prometheus.getSubsystem(DriveSystem.class)).getMovementController()
                        .move(Directions.WEST,
                                distance / 3,
                                0.8,
                                new PioTimer(ElapsedTime.Resolution.SECONDS, 3, 1),
                                PIDComplexity.LOW);

                sleep(2000);
            }

            if (gamepad1.y) {
                ((DriveSystem) prometheus.getSubsystem(DriveSystem.class)).getMovementController()
                        .move(Directions.NORTH,
                                distance / 3,
                                0.8,
                                new PioTimer(ElapsedTime.Resolution.SECONDS, 3, 1),
                                PIDComplexity.LOW);

                sleep(2000);
            }

            if (gamepad1.a) {
                ((DriveSystem) prometheus.getSubsystem(DriveSystem.class)).getMovementController()
                        .move(Directions.SOUTH,
                                distance / 3,
                                0.8,
                                new PioTimer(ElapsedTime.Resolution.SECONDS, 3, 1),
                                PIDComplexity.LOW);

                sleep(2000);
            }

            if (gamepad1.b) {
                ((DriveSystem) prometheus.getSubsystem(DriveSystem.class)).getMovementController()
                        .move(Directions.EAST,
                                distance / 3,
                                0.8,
                                new PioTimer(ElapsedTime.Resolution.SECONDS, 3, 1),
                                PIDComplexity.LOW);

                sleep(2000);
            }

            if (buttonDUp.isClicked()) {
                distance += 10;
            }

            if (buttonDDown.isClicked()) {
                distance -= 10;
            }
        }
    }
}

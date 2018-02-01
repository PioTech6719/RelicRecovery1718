package org.firstinspires.ftc.teamcode.opmodes.auto.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Directions;
import org.firstinspires.ftc.teamcode.hardware.robots.Prometheus;
import org.firstinspires.ftc.teamcode.match.Alliance;
import org.firstinspires.ftc.teamcode.opmodes.auto.BaseAutoOp;
import org.firstinspires.ftc.teamcode.subsystem.drive.DriveSystem;
import org.firstinspires.ftc.teamcode.subsystem.drive.drivecontroller.PID.PIDComplexity;
import org.firstinspires.ftc.teamcode.utils.PioTimer;

@Autonomous(name = "Sensor Test - North South Distance", group = "test")
public class NorthSouthDistanceTest extends BaseAutoOp {
    private Alliance alliance = Alliance.RED;
    private Prometheus prometheus;

    @Override
    public void initializeClass() {
        prometheus = new Prometheus(this);

        setRobot(prometheus);
        setGamepadConfig(null);
        setAlliance(Alliance.BLUE);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        telemetry.addData("Status : ", "Do NOT have Timer on");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            ((DriveSystem) prometheus.getSubsystem(DriveSystem.class)).getMovementController()
                    .move(Directions.NORTH, //TODO: Def not going to work bc encoders
                            12 / 2,
                            0.5,
                            new PioTimer(ElapsedTime.Resolution.SECONDS, 7, 1),
                            PIDComplexity.LOW);

            telemetry.addData("Status : ", "Moved 12 North");
            telemetry.update();
            sleep(10000);

            ((DriveSystem) prometheus.getSubsystem(DriveSystem.class)).getMovementController()
                    .move(Directions.SOUTH, //TODO: Def not going to work bc encoders
                            12 / 2,
                            0.5,
                            new PioTimer(ElapsedTime.Resolution.SECONDS, 7, 1),
                            PIDComplexity.LOW);

            telemetry.addData("Status : ", "Moved 12 South");
            telemetry.update();
            sleep(10000);

            ((DriveSystem) prometheus.getSubsystem(DriveSystem.class)).getMovementController()
                    .move(Directions.NORTH, //TODO: Def not going to work bc encoders
                            24 / 2,
                            0.5,
                            new PioTimer(ElapsedTime.Resolution.SECONDS, 7, 1),
                            PIDComplexity.LOW);

            telemetry.addData("Status : ", "Moved 24 North");
            telemetry.update();
            sleep(10000);

            ((DriveSystem) prometheus.getSubsystem(DriveSystem.class)).getMovementController()
                    .move(Directions.SOUTH, //TODO: Def not going to work bc encoders
                            24 / 2,
                            0.5,
                            new PioTimer(ElapsedTime.Resolution.SECONDS, 7, 1),
                            PIDComplexity.LOW);

            telemetry.addData("Status : ", "Moved 24 South");
            telemetry.update();
            sleep(10000);

            ((DriveSystem) prometheus.getSubsystem(DriveSystem.class)).getMovementController()
                    .move(Directions.NORTH, //TODO: Def not going to work bc encoders
                            48 / 2,
                            0.5,
                            new PioTimer(ElapsedTime.Resolution.SECONDS, 10, 1),
                            PIDComplexity.LOW);

            telemetry.addData("Status : ", "Moved 48 North");
            telemetry.update();
            sleep(10000);

            ((DriveSystem) prometheus.getSubsystem(DriveSystem.class)).getMovementController()
                    .move(Directions.SOUTH, //TODO: Def not going to work bc encoders
                            48 / 2,
                            0.5,
                            new PioTimer(ElapsedTime.Resolution.SECONDS, 10, 1),
                            PIDComplexity.LOW);

            telemetry.addData("Status : ", "Moved 48 South");
            telemetry.update();
            sleep(10000);
            stop();
        }
    }
}

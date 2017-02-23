package org.firstinspires.ftc.teamcode.tasks;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.taskutil.Task;

public class MoveToWallTask extends Task {
    public MoveToWallTask(Object e) {
        super(e);
    }

    @Override
    public void init() {

    }

    @Override
    public void run() throws InterruptedException {
        while (true) {
            Robot.update();

            Robot.rightMotors(0.6f);
            Robot.leftMotors(0.6f);

            if (Robot.range.getDistance(DistanceUnit.INCH) < 8) {
                Robot.leftMotors(0.0f);
                Robot.rightMotors(0.0f);
                return;
            }

            Robot.telemetry.addData("Distance From Wall", Robot.range.getDistance(DistanceUnit.INCH));
            Robot.telemetry.update();

            Robot.idle();
        }
    }
}

package org.firstinspires.ftc.teamcode.tasks;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.taskutil.Task;

public class toBeaconTask extends Task {
    public toBeaconTask(Object e) {
        super(e);
    }

    @Override
    public void init() {

    }

    @Override
    public void run() throws InterruptedException {
        while (true) {
            Robot.update();

            double leftLine = Robot.leftLineLight.getLightDetected();
            double rightLine = Robot.rightLineLight.getLightDetected();

            if (0.005 < leftLine && leftLine < 0.07 && 0.005 < rightLine && rightLine < 0.07) { //if on the line
                Robot.leftMotors(0.65f);
                Robot.rightMotors(0.65f);
            } else if (leftLine > rightLine) { //to the right of the line
                Robot.leftMotors(0.1f);
                Robot.rightMotors(0.6f);
            } else if (rightLine > leftLine) { //to the left of the line
                Robot.leftMotors(0.6f);
                Robot.rightMotors(0.1f);
            } else {
                Robot.leftMotors(0.0f);
                Robot.rightMotors(0.0f);
            }

            if (Robot.range.getDistance(DistanceUnit.INCH) < 8) {
                Robot.leftMotors(0.0f);
                Robot.rightMotors(0.0f);
                return;
            }

            Robot.telemetry.addData("Left line light", leftLine);
            Robot.telemetry.addData("Right line light", rightLine);
            Robot.telemetry.update();

            Robot.idle();
        }
    }
}

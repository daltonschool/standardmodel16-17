package org.firstinspires.ftc.teamcode.tasks;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.taskutil.Task;

public class AlignmentTaskIdk extends Task {
    public AlignmentTaskIdk(Object e) {
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

            if (0.05 < leftLine && leftLine > 0.07 && 0.05 < rightLine && rightLine > 0.07) {
                Robot.leftMotors(0.3f);
                Robot.rightMotors(0.3f);
            } else if (leftLine > rightLine) {
                Robot.leftMotors(0.0f);
                Robot.rightMotors(0.3f);
            } else if (rightLine > leftLine) {
                Robot.leftMotors(0.3f);
                Robot.rightMotors(0.0f);
            } else {
                Robot.leftMotors(0.0f);
                Robot.rightMotors(0.0f);
            }

            if (Robot.range.getDistance(DistanceUnit.INCH) < 8) {
                Robot.leftMotors(0.0f);
                Robot.rightMotors(0.0f);
                return;
            }

            Robot.telemetry.update();

            Robot.idle();
        }
    }
}

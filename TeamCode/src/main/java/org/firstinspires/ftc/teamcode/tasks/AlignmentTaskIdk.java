package org.firstinspires.ftc.teamcode.tasks;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.taskutil.Task;
import com.qualcomm.robotcore.util.ElapsedTime;

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
//            while (Robot.range.getDistance(DistanceUnit.INCH) < 8) {
//                Robot.telemetry.addLine("too close...");
//                Robot.telemetry.update();
//                Robot.leftMotors(-.6f);
//                Robot.rightMotors(-.6f);
//                return;
//            }

            double leftLine = Robot.leftLineLight.getLightDetected();
            double rightLine = Robot.rightLineLight.getLightDetected();

            if (0.005 < leftLine && leftLine < 0.07 && 0.005 < rightLine && rightLine < 0.07) {
                Robot.leftMotors(0.65f);
                Robot.rightMotors(0.65f);
            } else if (leftLine > rightLine) {
                Robot.leftMotors(0.0f);
                Robot.rightMotors(0.6f);
            } else if (rightLine > leftLine) {
                Robot.leftMotors(0.6f);
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

//            if(currentTime - startTime > 100) {
                Robot.telemetry.addLine("Distance to beaon: " + Robot.range.getDistance(DistanceUnit.INCH));
                Robot.telemetry.update();
//            }

            Robot.telemetry.addData("Left line light", leftLine);
            Robot.telemetry.addData("Right line light", rightLine);
            Robot.telemetry.update();

            Robot.idle();
        }
    }
}

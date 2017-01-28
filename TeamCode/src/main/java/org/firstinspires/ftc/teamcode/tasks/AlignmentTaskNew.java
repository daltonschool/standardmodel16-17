package org.firstinspires.ftc.teamcode.tasks;

import android.util.Log;

import com.vuforia.CameraDevice;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.teamcode.Blackbox;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Utils;
import org.firstinspires.ftc.teamcode.taskutil.Task;

public class AlignmentTaskNew extends Task {
    public AlignmentTaskNew(Object e) {
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

            double leftOffset = Math.abs(leftLine - Robot.ODS_GRAY_VALUE);
            double rightOffset = Math.abs(rightLine - Robot.ODS_GRAY_VALUE);

            double basePower = 0.2;

            // TODO: mess with what the offset is multiplied by

            if (leftLine > rightLine) {
                // we are too far to the left, increase power to right motors
                Robot.leftMotors(basePower);
                Robot.rightMotors(basePower + 0.1 + (0.1) * leftOffset);
            } else if (rightLine > leftLine) {
                // we are too far to the right, increase power to left motors
                Robot.leftMotors(basePower + 0.1 + (0.1) * rightOffset);
                Robot.rightMotors(basePower);
            } else {
                // we are straight, just go forward
                Robot.leftMotors(basePower);
                Robot.rightMotors(basePower);
            }

            if (Robot.range.getDistance(DistanceUnit.INCH) < 4) {
                // we are close enough, just stop
                // TODO: check if we are straight. if we aren't, try to recover
                return;
            }

            Robot.telemetry.update();

            Robot.idle();
        }
    }
}

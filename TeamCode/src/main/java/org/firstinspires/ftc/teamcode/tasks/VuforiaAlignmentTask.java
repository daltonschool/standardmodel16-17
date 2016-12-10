package org.firstinspires.ftc.teamcode.tasks;

import com.vuforia.CameraDevice;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.teamcode.Blackbox;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.taskutil.Task;

public class VuforiaAlignmentTask extends Task {
    public VuforiaAlignmentTask(Object e) {
        super(e);
    }

    @Override
    public void init() {

    }

    @Override
    public void run() throws InterruptedException {
        CameraDevice.getInstance().setFlashTorchMode(true);
        VuforiaTrackable trackable = (VuforiaTrackable) extra;
        Thread.sleep(200);
        Robot.vuforia.update();
        Thread.sleep(200);
        Robot.vuforia.update();
        Thread.sleep(200);
        Robot.vuforia.update();
        while (true) {
            if (!Robot.vuforia.hasLocation()) {
                Robot.vuforia.update();
                continue;
            }
            VectorF translation = Robot.vuforia.getLocation();
            Orientation orientation = Robot.vuforia.getOrientation();
            OpenGLMatrix trackableLoc = trackable.getLocation();
            Orientation trackableOrientation = Orientation.getOrientation(trackableLoc, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
            float targetAngle = trackableOrientation.thirdAngle - 90;
            boolean orientationGood = ((orientation.thirdAngle > (targetAngle - 5) && orientation.thirdAngle < targetAngle) ||
                    (orientation.thirdAngle > targetAngle && orientation.thirdAngle < (targetAngle + 5)));
            int targetPosition = 0;
            if (trackable.getName().equals("Gears")) {
                targetPosition = -1150;
            } else {
                targetPosition = -600;
            }

            if (translation.get(0) < targetPosition) {
                // we're too close, just stop
                Blackbox.log("WARN", "We're too close, cancelling alignment!");
                Robot.telemetry.addLine("ALIGNTEST IS DONE");
                Robot.telemetry.addData("stat", "DONE2");
                Robot.leftMotors(0.0f);
                Robot.rightMotors(0.0f);
                return;
            }


            if (!orientationGood && orientation.thirdAngle < targetAngle) {
                Robot.telemetry.addData("stat", "RIGHT");
                Robot.leftMotors(0.1f);
                Robot.rightMotors(0.4f);
            } else if (!orientationGood && orientation.thirdAngle > targetAngle) {
                Robot.telemetry.addData("stat", "LEFT");
                Robot.leftMotors(0.4f);
                Robot.rightMotors(0.1f);
            } else if (translation.get(0) < targetPosition) {
                Robot.telemetry.addData("stat", "FORWARD");
                Robot.leftMotors(0.5f);
                Robot.rightMotors(0.5f);
            } else {
                Robot.telemetry.addData("stat", "DONE2");
                Blackbox.log("INFO", "Alignment complete!");
                Robot.telemetry.addLine("ALIGNTEST IS DONE");
                Robot.leftMotors(0.0f);
                Robot.rightMotors(0.0f);
                return;
            }

            Robot.telemetry.addData("Pos", Robot.vuforia.getLocationAsString());
            Robot.telemetry.addData("Ptest", translation.get(0));
            Robot.telemetry.addData("Target", targetPosition);
            Robot.telemetry.addData("Still going", (translation.get(0) < targetPosition));
            Robot.telemetry.addData("Test", Robot.leftMotor.getCurrentPosition());
            Robot.telemetry.addData("Orientation good?", orientationGood);
            Robot.telemetry.addData("Target angle", targetAngle);
            Robot.telemetry.update();
            Robot.update();
            Robot.idle();
        }
    }
}
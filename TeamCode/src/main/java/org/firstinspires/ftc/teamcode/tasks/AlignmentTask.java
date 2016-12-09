package org.firstinspires.ftc.teamcode.tasks;

import android.app.Activity;
import android.util.Log;
import android.view.View;

import com.vuforia.CameraDevice;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.teamcode.Blackbox;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Utils;
import org.firstinspires.ftc.teamcode.sensors.Vuforia;
import org.firstinspires.ftc.teamcode.taskutil.Task;

public class AlignmentTask extends Task {
    public AlignmentTask(Object e) {
        super(e);
    }

    @Override
    public void init() {
        VuforiaTrackable trackable = (VuforiaTrackable) extra;

        Blackbox.log("VUFO", trackable.getLocation().formatAsTransform());

    }

    @Override
    public void run() throws InterruptedException {
        CameraDevice.getInstance().setFlashTorchMode(true);
        VuforiaTrackable trackable = (VuforiaTrackable) extra;

        OpenGLMatrix targetLocation = trackable.getLocation();
        float targetPosition = targetLocation.getTranslation().get(0) - 200;
        Orientation targetOrientation = Utils.matrixToOrientation(targetLocation);
        //targetOrientation.thirdAngle

        int targetColor = 25;
        float baseSpeed = 0.075f;

        while (true) {
            Robot.update();

            Robot.telemetry.addData("leftLine", Robot.leftLineColor.whiteReading());
            Robot.telemetry.addData("rightLine", Robot.rightLineColor.whiteReading());
            Robot.telemetry.addData("dist", Robot.frontDist.getLightDetected());
            Robot.telemetry.addData("targetDist", targetPosition);
            if (Robot.vuforia.hasLocation()) {
                Robot.telemetry.addData("vufDist", Robot.vuforia.getLocationAsString());
                Robot.telemetry.addData("vufDistA", Robot.vuforia.getLocation().get(0));
            } else {
                Robot.telemetry.addData("vufDist", "no");
                Robot.telemetry.addData("vufDistA", "no");
            }

            int leftReading = Robot.leftLineColor.whiteReading();
            int rightReading = Robot.rightLineColor.whiteReading();

            if (leftReading == rightReading) {
                Robot.leftMotors(0.3f);
                Robot.rightMotors(0.3f);
            } else if (leftReading > rightReading) {
                Robot.leftMotors(0.0f);
                Robot.rightMotors(0.4f * Math.max(0.25f, Math.min(0.9f, ((leftReading - rightReading) * 0.1f))));
            } else if (leftReading < rightReading) {
                Robot.leftMotors(0.4f * Math.max(0.25f, Math.min(0.9f, ((rightReading - leftReading) * 0.1f))));
                Robot.rightMotors(0.0f);
            }
            if (Robot.vuforia.hasLocation() && Robot.vuforia.getLocation().get(0) < -1300) {
                Robot.telemetry.addData("vufDist", Robot.vuforia.getLocationAsString());
                Robot.telemetry.addData("vufDistA", Robot.vuforia.getLocation().get(0));
                Robot.telemetry.update();
                Robot.leftMotors(0.0f);
                Robot.rightMotors(0.0f);
                Thread.sleep(1000);
                return;
            }

            /*if (Robot.frontDist.getLightDetected() > 0.018) {
                return;
            }*/

            Robot.telemetry.update();

            Robot.idle();
        }
    }
}

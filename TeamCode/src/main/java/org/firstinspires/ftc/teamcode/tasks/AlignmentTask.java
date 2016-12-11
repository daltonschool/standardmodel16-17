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
        //VuforiaTrackable trackable = (VuforiaTrackable) extra;
    }

    @Override
    public void run() throws InterruptedException {
        CameraDevice.getInstance().setFlashTorchMode(true);
        VuforiaTrackable trackable = (VuforiaTrackable) extra;

        OpenGLMatrix targetLocation = trackable.getLocation();
        float targetPosition = targetLocation.getTranslation().get(0) - 200;
        Orientation targetOrientation = Utils.matrixToOrientation(targetLocation);

        int targetColor = 20;
        float baseSpeed = 0.075f;
        int maxEncoderDistance = 0;
        double maxRobotDistance = 0.016;

        if (trackable.getName().equals("Gears")) {
            targetPosition = -1330;
        } else if (trackable.getName().equals("Tools")) {
            targetPosition = -1230;
        } else if (trackable.getName().equals("Legos")) {
            targetPosition = -660;
        } else if (trackable.getName().equals("Wheels")) {
            targetPosition = 610;
        }

        boolean negativeField = (targetPosition < 0);

        Blackbox.log("LINE", "=== Line following ===");
        Blackbox.log("LINE", "Trackable name: " + trackable.getName());
        Blackbox.log("LINE", "Target position: " + targetPosition);
        Blackbox.log("LINE", "Base speed: " + baseSpeed);
        Blackbox.log("LINE", "Max encoder distance: " + maxEncoderDistance);
        Blackbox.log("LINE", "Max robot distance: " + maxRobotDistance);

        int leftAvgTop = 0;
        int rightAvgTop = 0;
        int avgSamples = 0;
        while (true) {
            Robot.update();

            int leftReading = Robot.leftLineColor.whiteReading() - 6;
            int rightReading = Robot.rightLineColor.whiteReading() - 6;

            leftAvgTop += leftReading;
            rightAvgTop += rightReading;
            avgSamples++;
            int leftAvg = leftAvgTop / avgSamples;
            int rightAvg = rightAvgTop / avgSamples;

            Robot.telemetry.addData("leftLine", Robot.leftLineColor.whiteReading());
            Robot.telemetry.addData("rightLine", Robot.rightLineColor.whiteReading());
            Robot.telemetry.addData("dist", Robot.frontDist.getLightDetected());
            Robot.telemetry.addData("targetDist", targetPosition);
            Robot.telemetry.addData("leftAvg", leftAvg);
            Robot.telemetry.addData("rightAvg", rightAvg);
            if (Robot.vuforia.hasLocation()) {
                Log.i("vufDistQ", Float.toString(Robot.vuforia.getLocation().get(0)));
                Robot.telemetry.addData("vufDist", Robot.vuforia.getLocationAsString());
                Robot.telemetry.addData("vufDistA", Robot.vuforia.getLocation().get(0));
            } else {
                Robot.telemetry.addData("vufDist", "no");
                Robot.telemetry.addData("vufDistA", "no");
            }

            if (true) {
                /*if (
                        (Math.abs(leftAvg - leftReading) > 5) ||
                        (Math.abs(rightAvg - rightReading) > 5)
                        ) {
                    Robot.leftMotors(0.0f);
                    Robot.rightMotors(0.0f);
                    return;
                }*/
                if (leftReading == rightReading) {// || leftReading - 1 == rightReading || leftReading + 1 == rightReading) {
                    Robot.leftMotors(0.25f);
                    Robot.rightMotors(0.25f);
                } else if (leftReading > rightReading) {
                    float calculated = 0.4f * Math.max(0.5f, Math.min(0.9f, ((leftReading - rightReading) * 0.02f)));
                    Robot.leftMotors(calculated / 2);
                    Robot.rightMotors(calculated * 2);
                } else if (leftReading < rightReading) {
                    float calculated = 0.4f * Math.max(0.5f, Math.min(0.9f, ((rightReading - leftReading) * 0.02f)));
                    Robot.leftMotors(calculated * 2);
                    Robot.rightMotors(calculated / 2);
                }
                if (
                        (negativeField && Robot.vuforia.hasLocation() && Robot.vuforia.getLocation().get(0) < targetPosition) ||
                                (!negativeField && Robot.vuforia.hasLocation() && Robot.vuforia.getLocation().get(0) > targetPosition) ||
                                (Robot.frontDist.getLightDetected() > maxRobotDistance)
                        ) {
                    Log.i("stopped", "stopped!!!!");
                    Log.i("stopped", "stopped!!!!");
                    Log.i("stopped", "stopped!!!!");
                    if (Robot.vuforia.hasLocation()) {
                        Robot.telemetry.addData("vufDist", Robot.vuforia.getLocationAsString());
                        Robot.telemetry.addData("vufDistA", Robot.vuforia.getLocation().get(0));
                    } else {
                        Robot.telemetry.addData("vufDist", "(null)");
                        Robot.telemetry.addData("vufDistA", "(null)");
                    }
                    Robot.telemetry.update();
                    Robot.leftMotors(0.0f);
                    Robot.rightMotors(0.0f);
                    Thread.sleep(1000);
                    return;
                }
            }

            /*if (Robot.frontDist.getLightDetected() > 0.018) {
                return;
            }*/

            Robot.telemetry.update();

            Robot.idle();
        }
    }
}

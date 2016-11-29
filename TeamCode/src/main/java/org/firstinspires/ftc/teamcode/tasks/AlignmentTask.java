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
        Robot.leftLineColor.enableLed(true);
        Robot.rightLineColor.enableLed(true);
        VuforiaTrackable trackable = (VuforiaTrackable) extra;

        OpenGLMatrix targetLocation = trackable.getLocation();
        float targetPosition = targetLocation.getTranslation().get(0) - 200;
        Orientation targetOrientation = Utils.matrixToOrientation(targetLocation);
        //targetOrientation.thirdAngle

        while (true) {
            Robot.update();

            Robot.telemetry.addData("leftLine", Utils.getColorString(Robot.leftLineColor));
            Robot.telemetry.addData("rightLine", Utils.getColorString(Robot.rightLineColor));
            Robot.telemetry.addData("dist", Robot.frontDist.getLightDetected());
            if (Robot.vuforia.hasLocation()) {
                Robot.telemetry.addData("vufDist", Robot.vuforia.getLocationAsString());
            } else {
                Robot.telemetry.addData("vufDist", "no");
            }

            if (Robot.leftLineColor.blue() == Robot.rightLineColor.blue()) {
                Robot.leftMotors(0.2f);
                Robot.rightMotors(0.2f);
            } else if (Robot.leftLineColor.blue() > Robot.rightLineColor.blue()) {
                Robot.leftMotors(0.0f);
                Robot.rightMotors(0.4f);
            } else if (Robot.leftLineColor.blue() < Robot.rightLineColor.blue()) {
                Robot.leftMotors(0.4f);
                Robot.rightMotors(0.0f);
            }

            if (Robot.vuforia.hasLocation() && Robot.vuforia.getLocation().get(0) < -1300) {
                Robot.leftMotors(0.0f);
                Robot.rightMotors(0.0f);
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

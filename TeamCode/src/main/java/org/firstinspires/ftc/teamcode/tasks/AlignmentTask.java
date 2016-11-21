package org.firstinspires.ftc.teamcode.tasks;

import android.app.Activity;
import android.util.Log;
import android.view.View;

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
        VuforiaTrackable trackable = (VuforiaTrackable) extra;

        OpenGLMatrix targetLocation = trackable.getLocation();
        float targetPosition = targetLocation.getTranslation().get(0) - 200;
        Orientation targetOrientation = Utils.matrixToOrientation(targetLocation);
        //targetOrientation.thirdAngle

        while (true) {
            Robot.update();
            if (!Robot.vuforia.hasLocation()) {
                continue;
            }


            Robot.idle();
        }
    }
}

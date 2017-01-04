package org.firstinspires.ftc.teamcode.tasks;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.sensors.MRColorSensor;
import org.firstinspires.ftc.teamcode.taskutil.Task;

public class SpookyTestTask extends Task {
    public SpookyTestTask(Object e) {
        super(e);
    }

    @Override
    public void init() {

    }

    @Override
    public void run() throws InterruptedException {
        while (true) {
            if (Robot.vuforia.hasLocation()) {
                VectorF translation = Robot.vuforia.getLocation();
                Orientation orientation = Robot.vuforia.getOrientation();
                Robot.telemetry.addData("Pos", Robot.vuforia.getLocationAsString());
                Robot.telemetry.addData("Dist", translation.get(0));
                Robot.telemetry.addData("Light Detected", Robot.frontDist.getLightDetected());
                Robot.telemetry.addData("Raw Light Detected", Robot.frontDist.getRawLightDetected());
                Robot.telemetry.update();
            } else {
                Robot.telemetry.addLine("No Vuforia position!");
                Robot.telemetry.update();
            }
            Robot.vuforia.update();
        }
    }
}

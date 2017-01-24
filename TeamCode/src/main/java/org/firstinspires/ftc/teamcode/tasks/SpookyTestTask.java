package org.firstinspires.ftc.teamcode.tasks;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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
            Robot.telemetry.addData("Left line light", Robot.leftLineLight.getLightDetected());
            Robot.telemetry.addData("Right line light", Robot.rightLineLight.getLightDetected());
            Robot.telemetry.addData("Distance (in)", Robot.range.getDistance(DistanceUnit.INCH));

            Robot.telemetry.update();
            Robot.idle();
        }
    }
}

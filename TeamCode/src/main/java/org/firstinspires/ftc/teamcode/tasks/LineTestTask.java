package org.firstinspires.ftc.teamcode.tasks;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.taskutil.Task;

public class LineTestTask extends Task {
    public LineTestTask(Object e) {
        super(e);
    }

    @Override
    public void init() {

    }

    @Override
    public void run() throws InterruptedException {
        while (true) {
            Robot.leftMotors(0.18f);
            Robot.rightMotors(0.18f);

            while (true) {
                if (Robot.centerLineLight.getLightDetected() > Robot.ODS_BLACK_VALUE) {
                    break;
                }
            }

            Robot.moveForward_encoder(-2000, 0.3f);

            Robot.telemetry.update();
            Robot.update();
            Robot.idle();
        }
    }
}

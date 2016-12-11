package org.firstinspires.ftc.teamcode.tasks;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.sensors.MRColorSensor;
import org.firstinspires.ftc.teamcode.taskutil.Task;

public class TurnUntilLineTask extends Task {
    public TurnUntilLineTask(Object e) {
        super(e);
    }

    @Override
    public void init() {

    }

    @Override
    public void run() throws InterruptedException {
        MRColorSensor checkingColorSensor = Robot.leftLineColor;
        int leftFactor = -1;
        if (Robot.currentAlliance == Alliance.RED) {
            checkingColorSensor = Robot.rightLineColor;
            leftFactor = -1;
        } else if (Robot.currentAlliance == Alliance.BLUE) {
            checkingColorSensor = Robot.leftLineColor;
            leftFactor = 1;
        }

        int whiteReading = (int)checkingColorSensor.whiteReading() - (int)6;
        while ((whiteReading < 5)) {
            Robot.leftMotors(0.35f * leftFactor);
            Robot.rightMotors(-0.35f * leftFactor);
            whiteReading = (int)checkingColorSensor.whiteReading() - (int)6;
        }

        Robot.leftMotors(0.0f);
        Robot.rightMotors(0.0f);
    }
}

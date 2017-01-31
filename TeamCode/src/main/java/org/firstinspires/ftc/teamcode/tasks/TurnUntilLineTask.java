package org.firstinspires.ftc.teamcode.tasks;

import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

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
        OpticalDistanceSensor sensorToTest = Robot.rightLineLight;
        int leftFactor = 1;
        if (Robot.currentAlliance == Alliance.RED) {
            sensorToTest = Robot.leftLineLight;
            leftFactor = -1;
        }

        while (sensorToTest.getLightDetected() <= Robot.ODS_BLACK_VALUE) {
            Robot.leftMotors(0.35f * leftFactor);
            Robot.rightMotors(-0.35f * leftFactor);

            Robot.update();
            Robot.idle();
        }

        Robot.leftMotors(0.0f);
        Robot.rightMotors(0.0f);
    }
}

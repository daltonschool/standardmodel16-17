package org.firstinspires.ftc.teamcode.tasks;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.taskutil.Task;

public class FlywheelEngageTask extends Task {
    public FlywheelEngageTask(Object e) {
        super(e);
    }

    @Override
    public void init() {

    }

    @Override
    public void run() throws InterruptedException {
        float flywheelPower = 0.3f;

        if (Robot.voltageSensor.getVoltage() < 13) {
            flywheelPower = 0.4f;
        } else if (Robot.voltageSensor.getVoltage() < 13.5) {
            flywheelPower = 0.35f;
        } else {
            flywheelPower = 0.3f;
        }

        Robot.flywheelLeft.setPower(flywheelPower);
        Robot.flywheelRight.setPower(flywheelPower);
    }
}

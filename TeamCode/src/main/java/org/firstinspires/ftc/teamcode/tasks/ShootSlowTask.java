package org.firstinspires.ftc.teamcode.tasks;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.Blackbox;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.taskutil.Task;

public class ShootSlowTask extends Task {
    public ShootSlowTask(Object e) {
        super(e);
    }

    @Override
    public void init() {

    }

    @Override
    public void run() throws InterruptedException {
        float flywheelPower = 0.3f;

        if (Robot.voltageSensor.getVoltage() < 13) {
            flywheelPower = 0.5f;
        } else if (Robot.voltageSensor.getVoltage() < 13.5) {
            flywheelPower = 0.45f;
        } else {
            flywheelPower = 0.4f;
        }

        Robot.flywheelLeft.setPower(flywheelPower);
        Robot.flywheelRight.setPower(flywheelPower);

        Thread.sleep(1500);

        Robot.conveyor.setPower(1.0f);
        Thread.sleep(500);
        Robot.conveyor.setPower(0.0f);
        Thread.sleep(500);
        Robot.conveyor.setPower(1.0f);

        Thread.sleep(3000);

        Robot.conveyor.setPower(0.0f);
        Robot.flywheelLeft.setPower(0.0f);
        Robot.flywheelRight.setPower(0.0f);
        Robot.nom.setPower(0.0f);
    }
}
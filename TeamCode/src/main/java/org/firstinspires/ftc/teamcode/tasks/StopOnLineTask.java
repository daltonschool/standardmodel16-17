package org.firstinspires.ftc.teamcode.tasks;

import org.firstinspires.ftc.teamcode.PIDController;
import org.firstinspires.ftc.teamcode.Robot;

import org.firstinspires.ftc.teamcode.sensors.ColorSensors;
import org.firstinspires.ftc.teamcode.taskutil.Task;

/**
 * Created by student on 11/22/16.
 */
public class StopOnLineTask extends Task {
    public StopOnLineTask(Object e) {
        super(e);
    }
    ColorSensors csValues = new ColorSensors();
    PIDController pidObject =  new PIDController(.1, .1, .1);
    double colorVal = csValues.getColorSensorVal("ls1");;
    double speed = .8;
    double errorR;

    @Override
    public void init() {
        Robot.turnToHeading(270, .7);
    }

    @Override
    public void run() throws InterruptedException {
        while (colorVal < 20) {
            errorR = colorVal - 25;
            colorVal = csValues.getColorSensorVal("ls1");
            Robot.leftMotors(speed);
            Robot.rightMotors(speed);
            speed = pidObject.Step(errorR);
            Robot.telemetry.addData("rightSensorValue: ", Robot.leftLineColor.blue());
            Robot.telemetry.update();
            Robot.update();
            Robot.idle();
        }
        Robot.leftMotors(0.0f);
        Robot.rightMotors(0.0f);
    }
}

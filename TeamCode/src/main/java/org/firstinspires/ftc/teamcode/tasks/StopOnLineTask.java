package org.firstinspires.ftc.teamcode.tasks;

import org.firstinspires.ftc.teamcode.PIDController;
import org.firstinspires.ftc.teamcode.Robot;

import org.firstinspires.ftc.teamcode.sensors.ColorSensors;
import org.firstinspires.ftc.teamcode.sensors.MRColorSensor;
import org.firstinspires.ftc.teamcode.taskutil.Task;

/*
public class StopOnLineTask extends Task {
    public StopOnLineTask(Object e) {
        super(e);
    }
<<<<<<< HEAD
    ColorSensors csValues = new ColorSensors();
    PIDController pidObject =  new PIDController();

    double colorVal = csValues.getColorSensorVal("right line color");;
=======
    //MRColorSensor rightColorSensor = new ColorSensors();
    PIDController pidObject =  new PIDController();
    byte colorVal;
>>>>>>> 06d748271e8eace2fa7efcd25494ba1bb216c1a0
    double speed = .8;
    double errorR;
    double startPos = Robot.leftMotor.getCurrentPosition();

    @Override
    public void init() {
        //Robot.turnToHeading(270, .7);
    }

    @Override
    public void run() throws InterruptedException {
<<<<<<< HEAD
        pidObject.setTunings();
=======
        pidObject.SetTunings(.1,.1,.1);
        pidObject.SetSampleTime(50);
        colorVal = Robot.rightLineColor.whiteReading();
>>>>>>> 06d748271e8eace2fa7efcd25494ba1bb216c1a0
        while (colorVal < 20) {
            errorR = colorVal - 10;
            colorVal = Robot.rightLineColor.whiteReading();
            Robot.leftMotors(speed);
            Robot.rightMotors(speed);
<<<<<<< HEAD
//            speed = pidObject.Step(errorR);
            Robot.telemetry.addData("rightSensorValue: ", Robot.leftLineColor.blue());
=======
            pidObject.Compute(5200 - (Robot.leftMotor.getCurrentPosition() - startPos));
            pidObject.SetOutputLimits(-1,1);
            speed = pidObject.getOutput();
            Robot.telemetry.addData("rightSensorValue: ", Robot.rightLineColor.whiteReading());
>>>>>>> 06d748271e8eace2fa7efcd25494ba1bb216c1a0
            Robot.telemetry.update();
            Robot.update();
            Robot.idle();
        }
        Robot.leftMotors(0.0f);
        Robot.rightMotors(0.0f);
    }
}
*/
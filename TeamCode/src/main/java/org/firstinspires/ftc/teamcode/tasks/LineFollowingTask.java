package org.firstinspires.ftc.teamcode.tasks;
import com.qualcomm.robotcore.hardware.ColorSensor;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.sensors.ColorSensors;
import org.firstinspires.ftc.teamcode.taskutil.Task;

import static java.lang.Thread.sleep;
import static org.firstinspires.ftc.teamcode.Robot.idle;
import static org.firstinspires.ftc.teamcode.Robot.leftMotors;
import static org.firstinspires.ftc.teamcode.Robot.rightMotors;

public class LineFollowingTask extends Task {
    public LineFollowingTask(Object e) {
        super(e);
    }
    ColorSensors obj1 = new ColorSensors();
    //PIDController pidObject = new PIDController(.1,.1,.15);
    double leftPower = .2;
    double rightPower = .2;
    double errorR = 0.0;
    //double errorL = 0.0;
    double Kp = (1 - leftPower) / (25);

    @Override
    public void init() {

    }

    @Override
    public void run() throws InterruptedException {
        rightMotors(0);
        leftMotors(0);
        sleep(50);
        while(true) {
            Robot.update();
            Robot.telemetry.addData("rightSensorValue: ", obj1.getAlphaVal("right line color"));
            Robot.telemetry.update();
            leftPower = .2;
            rightPower= .2;
            errorR = obj1.getAlphaVal("right line color") - 25;
            if (obj1.getAlphaVal("right line color") > 30) {
                leftPower = leftPower * (1 + (Kp * errorR));
                rightPower = rightPower * (1 - (Kp * errorR));
            }
            else if (obj1.getAlphaVal("right line color") < 20) {
                rightPower = rightPower * (1 - (Kp * errorR));
                leftPower = leftPower * (1 + (Kp * errorR));

            }
            else {
                leftPower = .2;
                rightPower = .2;
            }
            leftMotors(leftPower);
            rightMotors(rightPower);
            sleep(50);
            if (Robot.vuforia.hasLocation() && Robot.vuforia.getLocation().get(0) < -1300) {
                Robot.leftMotors(0.0f);
                Robot.rightMotors(0.0f);
                Thread.sleep(2000);
                return;
            }
            Robot.telemetry.update();
            idle();
        }
    }
}

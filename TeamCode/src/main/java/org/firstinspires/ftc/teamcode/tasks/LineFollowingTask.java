package org.firstinspires.ftc.teamcode.tasks;

import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Utils;
import org.firstinspires.ftc.teamcode.taskutil.Task;

import static org.firstinspires.ftc.teamcode.Robot.leftMotors;
import static org.firstinspires.ftc.teamcode.Robot.rightMotors;

/**
 * Created by student on 11/21/16.
 */
public class LineFollowingTask extends Task {
    public LineFollowingTask(Object e) {
        super(e);
    }
    public static ColorSensor lineSensor = null;
    double lastError = 0;
    double rightBaseSpeed = .5;
    double leftBaseSpeed = .5;
    double rightMaxSpeed = 1;
    double leftMaxSpeed = 1;
    double Kp = 0;
    double Kd = 0;

    @Override
    public void init() {
    }

    @Override
    public void run() throws InterruptedException {
        linefollow:
        while (true) { //change to condition to check if it close enough to read the beacon color
            double colorVal = lineSensor.green() + lineSensor.blue() + lineSensor.red();
            boolean done = false;
            double whiteLineIntensityValue = 23.6; //Replace with value that we get
            double blackMatIntensityValue = 1.24;  //Also do that here
            double targetIntensity = whiteLineIntensityValue + blackMatIntensityValue / 2; //Get tarrget value


           // int position = qtrrc.readLine(sensors); // get calibrated readings along with the line position, refer to the QTR Sensors Arduino Library for more details on line position.
            double error = colorVal;

            double motorSpeed = Kp * error + Kd * (error - lastError);
            lastError = error;

            double rightMotorSpeed = rightBaseSpeed + motorSpeed;
            double leftMotorSpeed = leftBaseSpeed - motorSpeed;

            if (rightMotorSpeed > rightMaxSpeed )
                rightMotorSpeed = rightMaxSpeed; // prevent the motor from going beyond max speed
            if (leftMotorSpeed > leftMaxSpeed )
                leftMotorSpeed = leftMaxSpeed; // prevent the motor from going beyond max speed
            if (rightMotorSpeed < 0)
                rightMotorSpeed = 0; // keep the motor speed positive
            if (leftMotorSpeed < 0)
                leftMotorSpeed = 0; // keep the motor speed positive

            rightMotors(rightMotorSpeed);
            leftMotors(leftMotorSpeed);

            if (done) {
                rightMotors(0);
                leftMotors(0);
                break linefollow;
            }
        }

    }
}

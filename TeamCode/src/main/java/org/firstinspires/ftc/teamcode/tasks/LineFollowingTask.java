package org.firstinspires.ftc.teamcode.tasks;

import android.graphics.Color;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.PIDController;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.sensors.ColorSensors;
import org.firstinspires.ftc.teamcode.taskutil.Task;
import org.firstinspires.ftc.teamcode.Robot;
import static org.firstinspires.ftc.teamcode.Robot.leftMotors;
import static org.firstinspires.ftc.teamcode.Robot.rightMotors;
import static org.firstinspires.ftc.teamcode.Robot.vuforia;

public class LineFollowingTask extends Task {
    public LineFollowingTask(Object e) {
        super(e);
    }
    ColorSensors obj1 = new ColorSensors();
    PIDController obj2 = new PIDController(.1,.1,.1);
    double errorR;

    @Override
    public void init() {

    }

    @Override
    public void run() throws InterruptedException {
        ColorSensor rightLine = Robot.rightLineColor;
        ColorSensor leftLine = Robot.leftLineColor;

        double whiteLineIntensityValue = 50.0; //REPLACE WITH CORRECT VALUE
        double blackMatIntensityValue = 0.0;
        double sensorVal_1 = rightLine.alpha();
//        double sensorVal_2 = obj1.getAlphaVal("ls2");
        errorR = sensorVal_1 - ((whiteLineIntensityValue+blackMatIntensityValue)/2);
        double leftPower = .2;
        double rightPower = .2;

        //actual line following code
        while (!(Robot.vuforia.hasLocation() && Robot.vuforia.getLocation().get(0) < -1300) ){
            //gets values of the 2 sensors
            sensorVal_1 = rightLine.alpha();
//            sensorVal_2 = obj1.getAlphaVal("left line color");

            if(sensorVal_1 > 30) {
                //the right sensor is seeing white it needs
                leftPower = obj2.Step(errorR);
                rightPower = obj2.Step(errorR);
            }
            else if(sensorVal_1 < 20){ //if needs to turn left (white and black)
                //rightMotors(.8);
                //leftMotors(.8);
            }
            else {
                //both are in between 20 and 30 so both are seeing the line
                leftPower = .2;
                rightPower = .2;
            }
            rightMotors(rightPower);
            leftMotors(-leftPower);
            Robot.idle();
        }
        rightMotors(0);
        leftMotors(0);
    }
}

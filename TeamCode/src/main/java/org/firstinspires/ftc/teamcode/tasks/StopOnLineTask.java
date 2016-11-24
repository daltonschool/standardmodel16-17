package org.firstinspires.ftc.teamcode.tasks;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.teamcode.Blackbox;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Utils;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.sensors.ColorSensors;
import org.firstinspires.ftc.teamcode.taskutil.Task;
import com.qualcomm.robotcore.hardware.ColorSensor;
/**
 * Created by student on 11/22/16.
 */
public class StopOnLineTask extends Task {
    public StopOnLineTask(Object e) {
        super(e);
    }
    ColorSensors obj1 = new ColorSensors();
    double colorVal;

    @Override
    public void init() {
        Robot.turnToHeading(270, .7);
    }

    @Override
    public void run() throws InterruptedException {
        Robot.telemetry.addLine("CHECKING VALUE");
        Robot.telemetry.update();
        Thread.sleep(1000);
        boolean oL = false;
        while (!oL) {
            Robot.leftMotors(0.2f);
            Robot.rightMotors(0.2f);
            Robot.update();
            Robot.idle();
            colorVal = obj1.getColorSensorVal("ls1");

            if (colorVal>240) {
                oL = true;
            }
        }
        Robot.leftMotors(0.0f);
        Robot.rightMotors(0.0f);
    }
}

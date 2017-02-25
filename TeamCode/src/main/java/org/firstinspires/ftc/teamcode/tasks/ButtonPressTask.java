package org.firstinspires.ftc.teamcode.tasks;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.Blackbox;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Utils;
import org.firstinspires.ftc.teamcode.taskutil.Task;

public class ButtonPressTask extends Task {
    public ButtonPressTask(Object e) {
        super(e);
    }

    @Override
    public void init() {

    }

    public Alliance getBeaconColor() {
        Alliance rightColor;
        if (Robot.leftBeaconColor.red() == Robot.leftBeaconColor.blue()) {
            rightColor = Alliance.UNKNOWN;
        } else if (Robot.leftBeaconColor.red() > Robot.leftBeaconColor.blue()) {
            rightColor = Alliance.RED;
        } else {
            rightColor = Alliance.BLUE;
        }
        return rightColor;
    }

    public void extendArmBasedOnLeftColor(Alliance leftColor) throws InterruptedException {
        if (leftColor == Robot.currentAlliance) {
            Blackbox.log("INFO", "Pressing LEFT");
            Robot.telemetry.addLine("PRESSING LEFT");
            Robot.extendLeft();
        } else {
            Blackbox.log("INFO", "Pressing RIGHT");
            Robot.telemetry.addLine("PRESSING RIGHT");
            Robot.extendRight();
        }
        Robot.telemetry.update();
        Thread.sleep(500);
    }

    @Override
    public void run() throws InterruptedException {
        /*Robot.beaconLeft.setPosition(0.0);
        Robot.beaconRight.setPosition(0.0);
        Robot.telemetry.addLine("WAITING FOR SERVO POS");
        Robot.telemetry.update();
        Thread.sleep(500);*/

        // find the right color
        Alliance leftColor = getBeaconColor();

        Blackbox.log("INFO", "Beacon color sensor: r: " + Robot.leftBeaconColor.red() + ", g: " + Robot.leftBeaconColor.green() + ", b: " + Robot.leftBeaconColor.blue());

        // check if we found it
        if (leftColor == Alliance.UNKNOWN) {
            // ahh!
            Blackbox.log("INFO", "Could not find color!");
            Robot.telemetry.addLine("Could not find color!");
            Robot.telemetry.update();

            // back up
            /*Robot.leftMotors(-0.4f);
            Robot.rightMotors(-0.4f);
            Thread.sleep(500);
            Robot.leftMotors(0.0f);
            Robot.rightMotors(0.0f);*/

            return;
        }

        // extend correct arm
        extendArmBasedOnLeftColor(leftColor);

        // move forwards
        Robot.leftMotors(0.5f);
        Robot.rightMotors(0.5f);
        Thread.sleep(1000);
        Robot.leftMotors(0.0f);
        Robot.rightMotors(0.0f);

        // retreat
        Robot.leftMotors(-0.5f);
        Robot.rightMotors(-0.5f);
        Thread.sleep(700);
        Robot.leftMotors(0.0f);
        Robot.rightMotors(0.0f);

        // reset arm
        Robot.extendBoth();
        Thread.sleep(500);

        leftColor = getBeaconColor();
        int timesTried = 0;
        while (leftColor != Robot.currentAlliance) {
            extendArmBasedOnLeftColor(leftColor);

            // move forwards
            Robot.leftMotors(0.5f);
            Robot.rightMotors(0.5f);
            Thread.sleep(1500);
            Robot.leftMotors(0.0f);
            Robot.rightMotors(0.0f);

            // retreat
            Robot.leftMotors(-0.5f);
            Robot.rightMotors(-0.5f);
            Thread.sleep(700);
            Robot.leftMotors(0.0f);
            Robot.rightMotors(0.0f);;

            // reset arm
            Robot.extendBoth();
            Thread.sleep(500);

            leftColor = getBeaconColor(); // check new value
            timesTried++;

            if (timesTried > 2) {
                // stop
                Blackbox.log("BTN", "Tried too many times, giving up :(");
                Robot.telemetry.addLine("Tried too many times, giving up :(");
                Robot.telemetry.update();
                return;
            }
        }

        // reset
        Robot.extendBoth();
    }
}

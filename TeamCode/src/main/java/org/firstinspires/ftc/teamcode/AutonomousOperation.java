package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;

public abstract class AutonomousOperation extends LinearOpMode
{
    public static ColorSensor lineSensor = null;
    private ElapsedTime runtime = new ElapsedTime();

    public abstract Alliance getCurrentAlliance();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Starting...");
        telemetry.update();

        Blackbox.init();
        Blackbox.log("INFO", "Current alliance: " + (getCurrentAlliance() == Alliance.RED ? "red" : "blue"));

        Robot.init(this);

        // ready to go!
        Blackbox.log("INFO", "READY TO GO!");
        telemetry.addData("Status", "READY TO GO!");
        telemetry.update();

        Blackbox.log("vuf", "~~~~~~~~~~!");
        Blackbox.log("vuf", Robot.vuforia.gears.getLocation().formatAsTransform());
        Blackbox.log("vuf", Robot.vuforia.legos.getLocation().formatAsTransform());
        Blackbox.log("vuf", "~~~~~~~~~~");

        waitForStart();

        Robot.start();

        Blackbox.log("INFO", "Started!");

        while (true) {
            telemetry.addData("Status", "Running: " + runtime.toString());

            Robot.update();

            Robot.beaconLeft.setPosition(0.0);
            Robot.beaconRight.setPosition(0.0);

            Robot.nomMiddle.setPower(1.0f);
            Blackbox.log("INFO", "Servos reset, nom ON");

            Robot.moveForward_encoder(2300, 0.6f);
            Blackbox.log("INFO", "Move 1 done");
            telemetry.addLine("MOVE 1 DONE!");
            telemetry.update();
            Thread.sleep(101);

            Robot.flywheelLeft.setPower(0.4f);
            Robot.flywheelRight.setPower(0.4f);
            Blackbox.log("INFO", "Flywheels ON");

            Thread.sleep(500);

            Robot.conveyor.setPower(1.0f);
            Blackbox.log("INFO", "Conveyor ON");
            telemetry.addLine("CONVEYOR!");
            telemetry.update();
            Thread.sleep(3000);

            Robot.conveyor.setPower(0.0f);
            Robot.flywheelLeft.setPower(0.0f);
            Robot.flywheelRight.setPower(0.0f);
            Robot.nomMiddle.setPower(0.0f);
            Blackbox.log("INFO", "Flywheel, conveyor, and nom OFF");

            Robot.turnToHeading((getCurrentAlliance() == Alliance.RED ? 40 : -40), 0.6f);
            Blackbox.log("INFO", "Turn 1 done");
            telemetry.addLine("TURN 1 DONE");
            telemetry.update();
            Thread.sleep(100);

            Robot.moveForward_encoder(3450, 0.5f);
            Blackbox.log("INFO", "Move 2 done");
            telemetry.addLine("MOVE 2 DONE");
            telemetry.update();
            Thread.sleep(100);

            Robot.turnToHeading((getCurrentAlliance() == Alliance.RED ? 90 : -90), 0.6f);
            Blackbox.log("INFO", "Turn 2 done");
            telemetry.addLine("TURN 2 DONE");
            telemetry.update();
            Thread.sleep(100);

            // align to gears
            Blackbox.log("INFO", "Starting Vuforia alignment...");
            telemetry.addLine("TRYING TO ALIGN...");
            telemetry.update();
            alignTest((getCurrentAlliance() == Alliance.RED ? Robot.vuforia.gears : Robot.vuforia.legos));
            Thread.sleep(250);

            /*Robot.turnToHeading(90, 0.8f);
            telemetry.addLine("ALIGN TURN DONE");
            telemetry.update();
            Thread.sleep(100);*/

            for (int i = 0; i < 2000; i++) {
                telemetry.addData("r", Robot.beaconColor.red());
                telemetry.addData("g", Robot.beaconColor.green());
                telemetry.addData("b", Robot.beaconColor.blue());
                telemetry.update();
                Thread.sleep(1);
            }

            Blackbox.log("INFO", "Beacon color sensor: r: " + Robot.beaconColor.red() + ", g: " + Robot.beaconColor.green() + ", b: " + Robot.beaconColor.blue());

            // determine beacon color
            Alliance rightColor = Robot.getBeaconRightColor();
            if (rightColor == getCurrentAlliance()) {
                Blackbox.log("INFO", "Pressing RIGHT");
                telemetry.addLine("PRESSING RIGHT");
                Robot.beaconLeft.setPosition(0.0);
                Robot.beaconRight.setPosition(0.0);
            } else {
                Blackbox.log("INFO", "Pressing LEFT");
                telemetry.addLine("PRESSING LEFT");
                Robot.beaconLeft.setPosition(1.0);
                Robot.beaconRight.setPosition(1.0);
            }
            telemetry.update();
            Thread.sleep(250);

            Blackbox.log("INFO", "Pressing button");
            telemetry.addLine("PRESSING BUTTON");
            telemetry.update();
            Robot.moveForward_encoder(400, 0.5f);
            Thread.sleep(250);

            /*if (rightColor != currentAlliance) {
                Thread.sleep(250);
                Robot.beaconServo.setPosition(0.65);
            }*/

            Blackbox.log("INFO", "Retreating");
            telemetry.addLine("RETREAT");
            telemetry.update();
            Robot.leftMotors(-0.3f);
            Robot.rightMotors(-0.3f);
            Thread.sleep(500);
            Robot.leftMotors(0.0f);
            Robot.rightMotors(0.0f);

            telemetry.addLine("THE END OF PART 1");
            telemetry.update();

            /*//turn 90 degrees to the right
            Robot.turnToHeading(270, .7);

            //go forward until the the line is sensed
            while(!(lineSensor.green() > 240 &&  lineSensor.blue() > 240 && lineSensor.red() > 240)) {
                Robot.leftMotors(0.2f);
                Robot.rightMotors(0.2f);
                Robot.update();
                Robot.idle();
            }*/
            Robot.leftMotors(0.0f);
            Robot.rightMotors(0.0f);

            Blackbox.log("INFO", "Complete!");

            // press button
            requestOpModeStop();

            telemetry.update();
            Robot.idle();
        }
    }

    /*public void alignTo(VuforiaTrackable trackable) {
        trackable.getLocation()
    }*/
    public boolean onLine() throws InterruptedException{
        telemetry.addLine("CHECKING VALUE");
        telemetry.update();
        Thread.sleep(1000);
        boolean oL = false;
        if (lineSensor.green() > 240 &&  lineSensor.blue() > 240 && lineSensor.red() > 240) {
            oL = true;
        }
        return oL;
    }
    public void alignToTrackable(VuforiaTrackable trackable) {
        while (true) {
            if (!Robot.vuforia.hasLocation()) {
                Robot.vuforia.update();
                continue;
            }

        }
    }

    public void alignTest(VuforiaTrackable trackable) throws InterruptedException {
        while (true) {
            if (!Robot.vuforia.hasLocation()) {
                Robot.vuforia.update();
                continue;
            }
            VectorF translation = Robot.vuforia.getLocation();
            Orientation orientation = Robot.vuforia.getOrientation();
            OpenGLMatrix trackableLoc = trackable.getLocation();
            Orientation trackableOrientation = Orientation.getOrientation(trackableLoc, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
            float targetAngle = trackableOrientation.thirdAngle - 90;
            boolean orientationGood = ((orientation.thirdAngle > (targetAngle - 5) && orientation.thirdAngle < targetAngle) ||
                                        (orientation.thirdAngle > targetAngle && orientation.thirdAngle < (targetAngle + 5)));
            int targetPosition = 0;
            if (trackable.getName() == "Gears") {
                targetPosition = -1150;
            } else {
                targetPosition = -400;
            }

            if (translation.get(0) < targetPosition) {
                // we're too close, just stop
                Blackbox.log("WARN", "We're too close, cancelling alignment!");
                telemetry.addLine("ALIGNTEST IS DONE");
                Robot.leftMotors(0.0f);
                Robot.rightMotors(0.0f);
                return;
            }


            if (!orientationGood && orientation.thirdAngle < 0) {
                Robot.leftMotors(0.1f);
                Robot.rightMotors(0.6f);
            } else if (!orientationGood && orientation.thirdAngle > 0) {
                Robot.leftMotors(0.6f);
                Robot.rightMotors(0.1f);
            } else if (translation.get(0) > targetPosition) {
                Robot.leftMotors(0.5f);
                Robot.rightMotors(0.5f);
            } else {
                Blackbox.log("INFO", "Alignment complete!");
                telemetry.addLine("ALIGNTEST IS DONE");
                Robot.leftMotors(0.0f);
                Robot.rightMotors(0.0f);
                return;
            }
            Robot.update();

            telemetry.addData("Pos", Robot.vuforia.getLocationAsString());
            telemetry.addData("Ptest", translation.get(0));
            telemetry.addData("Still going", (translation.get(0) > targetPosition));
            telemetry.addData("Test", Robot.leftMotor.getCurrentPosition());
            telemetry.addData("Second Angle", orientation.secondAngle);
            telemetry.update();
            Robot.idle();
        }
    }

}

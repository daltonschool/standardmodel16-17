package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.CameraDevice;

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
    public abstract boolean onlyShoots();

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

        CameraDevice.getInstance().setFlashTorchMode(true);

        Blackbox.log("INFO", "Started!");

        while (true) {
            telemetry.addData("Status", "Running: " + runtime.toString());

            Robot.update();

            if (onlyShoots()) {
                Thread.sleep(10000);
            }

            Robot.beaconLeft.setPosition(0.0);
            Robot.beaconRight.setPosition(0.0);

            Robot.nomMiddle.setPower(1.0f);
            Blackbox.log("INFO", "Servos reset, nom ON");

            if (onlyShoots()) {
                Robot.moveForward_encoder(2800, 0.55f);
            } else {
                if (getCurrentAlliance() == Alliance.RED) {
                    Robot.moveForward_encoder(2250, 0.55f);
                } else {
                    Robot.moveForward_encoder(2500, 0.55f);
                }
            }
            Blackbox.log("INFO", "Move 1 done");
            telemetry.addLine("MOVE 1 DONE!");
            telemetry.update();
            Thread.sleep(101);

            Robot.flywheelLeft.setPower(0.35f);
            Robot.flywheelRight.setPower(0.35f);
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

            if (onlyShoots()) {
                // we're done
                telemetry.addLine("SHOTS DONE");
                telemetry.update();
                Blackbox.log("INFO", "Shots complete - we are done.");
                requestOpModeStop();
                return;
            }

            Robot.turnToHeading((getCurrentAlliance() == Alliance.RED ? 40 : -40), 0.6f);
            Blackbox.log("INFO", "Turn 1 done");
            telemetry.addLine("TURN 1 DONE");
            telemetry.update();
            Thread.sleep(100);

            if (getCurrentAlliance() == Alliance.RED) {
                Robot.moveForward_encoder(3450, 0.5f);
            } else {
                Robot.moveForward_encoder(3450, 0.5f);
            }
            Blackbox.log("INFO", "Move 2 done");
            telemetry.addLine("MOVE 2 DONE");
            telemetry.update();
            Thread.sleep(100);

            Robot.turnToHeading((getCurrentAlliance() == Alliance.RED ? 90 : -81), 0.6f);
            Blackbox.log("INFO", "Turn 2 done");
            telemetry.addLine("TURN 2 DONE");
            telemetry.update();
            Thread.sleep(100);

            Thread.sleep(1000);

            // align to gears
            Blackbox.log("INFO", "Starting Vuforia alignment...");
            telemetry.addLine("TRYING TO ALIGN...");
            telemetry.update();
            alignTest((getCurrentAlliance() == Alliance.RED ? Robot.vuforia.gears : Robot.vuforia.wheels));
            Thread.sleep(250);

            if (getCurrentAlliance() == Alliance.BLUE) {
                // HACK
                telemetry.addLine("HACKY THING");
                telemetry.update();
                Robot.moveForward_encoder(1000, 0.5f);
                Thread.sleep(500);
            } else {
                // HACK
                telemetry.addLine("HACKY THING");
                telemetry.update();
                Robot.moveForward_encoder(450, 0.5f);
                Thread.sleep(500);
            }

            /*Robot.turnToHeading(90, 0.8f);
            telemetry.addLine("ALIGN TURN DONE");
            telemetry.update();
            Thread.sleep(100);*/

            for (int i = 0; i < 1000; i++) {
                telemetry.addData("r", Robot.beaconColor.red());
                telemetry.addData("g", Robot.beaconColor.green());
                telemetry.addData("b", Robot.beaconColor.blue());
                telemetry.update();
                Thread.sleep(1);
            }

            Blackbox.log("INFO", "Beacon color sensor: r: " + Robot.beaconColor.red() + ", g: " + Robot.beaconColor.green() + ", b: " + Robot.beaconColor.blue());

            // determine beacon color
            Alliance rightColor = Robot.getBeaconRightColor();
            if (rightColor == Alliance.UNKNOWN) {
                telemetry.addLine("ABANDONING BEACON!");
                telemetry.update();
                Blackbox.log("CRIT", "COULD NOT DETERMINE BEACON COLOR!!!");
                Blackbox.log("CRIT", "ABANDONING BEACON!!!");
                Robot.leftMotors(-0.4f);
                Robot.rightMotors(-0.4f);
                Thread.sleep(1000);
                Robot.leftMotors(0.0f);
                Robot.rightMotors(0.0f);
                veryEnd();
                requestOpModeStop();
                return;
            }
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
            Robot.leftMotors(0.5f);
            Robot.rightMotors(0.5f);
            Thread.sleep(1500);
            Robot.leftMotors(0.0f);
            Robot.rightMotors(0.0f);

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

            Blackbox.log("INFO", "Pressing button 2");
            telemetry.addLine("PRESSING BUTTON 2");
            telemetry.update();
            Robot.leftMotors(0.5f);
            Robot.rightMotors(0.5f);
            Thread.sleep(1500);
            Robot.leftMotors(0.0f);
            Robot.rightMotors(0.0f);

            Blackbox.log("INFO", "Retreating 2");
            telemetry.addLine("RETREAT 2");
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

            veryEnd();

            // press button
            requestOpModeStop();

            telemetry.update();
            Robot.idle();

            return;
        }
    }

    public void veryEnd() throws InterruptedException {
        if (getCurrentAlliance() == Alliance.RED) {
            Robot.leftMotors(-0.6f);
            Robot.rightMotors(-0.5f);
        } else {
            Robot.leftMotors(-0.5f);
            Robot.rightMotors(-0.6f);
        }
        Thread.sleep(1500);
        Robot.leftMotors(0.0f);
        Robot.rightMotors(0.0f);

        Thread.sleep(1500);

        Robot.leftMotors(-0.5f);
        Robot.rightMotors(0.5f);
        Thread.sleep(1500);
        Robot.leftMotors(0.0f);
        Robot.rightMotors(0.0f);

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
        Thread.sleep(200);
        Robot.vuforia.update();
        Thread.sleep(200);
        Robot.vuforia.update();
        Thread.sleep(200);
        Robot.vuforia.update();
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
                targetPosition = -600;
            }

            if (translation.get(0) < targetPosition) {
                // we're too close, just stop
                Blackbox.log("WARN", "We're too close, cancelling alignment!");
                telemetry.addLine("ALIGNTEST IS DONE");
                telemetry.addData("stat", "DONE2");
                Robot.leftMotors(0.0f);
                Robot.rightMotors(0.0f);
                return;
            }


            if (!orientationGood && orientation.thirdAngle < targetAngle) {
                telemetry.addData("stat", "RIGHT");
                Robot.leftMotors(0.1f);
                Robot.rightMotors(0.4f);
            } else if (!orientationGood && orientation.thirdAngle > targetAngle) {
                telemetry.addData("stat", "LEFT");
                Robot.leftMotors(0.4f);
                Robot.rightMotors(0.1f);
            } else if (translation.get(0) < targetPosition) {
                telemetry.addData("stat", "" +
                        "'[]FORWARD");
                Robot.leftMotors(0.5f);
                Robot.rightMotors(0.5f);
            } else {
                telemetry.addData("stat", "DONE2");
                Blackbox.log("INFO", "Alignment complete!");
                telemetry.addLine("ALIGNTEST IS DONE");
                Robot.leftMotors(0.0f);
                Robot.rightMotors(0.0f);
                return;
            }

            telemetry.addData("Pos", Robot.vuforia.getLocationAsString());
            telemetry.addData("Ptest", translation.get(0));
            telemetry.addData("Target", targetPosition);
            telemetry.addData("Still going", (translation.get(0) < targetPosition));
            telemetry.addData("Test", Robot.leftMotor.getCurrentPosition());
            telemetry.addData("Orientation good?", orientationGood);
            telemetry.addData("Target angle", targetAngle);
            telemetry.update();
            Robot.update();
            Robot.idle();
        }
    }

}

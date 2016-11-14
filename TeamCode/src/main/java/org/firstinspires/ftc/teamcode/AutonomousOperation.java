package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;

@Autonomous(name="Autonomous Operation")
public class AutonomousOperation extends LinearOpMode
{
    public static ColorSensor lineSensor = null;
    private ElapsedTime runtime = new ElapsedTime();
    private Alliance currentAlliance = Alliance.RED; // this is hardcoded for now, but should be set somehow (two different opmodes? how is this normally done?)

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Starting...");
        telemetry.update();

        Robot.init(this);

        // ready to go!
        telemetry.addData("Status", "READY TO GO");
        telemetry.update();
        waitForStart();

        Robot.start();

        while (true) {
            telemetry.addData("Status", "Running: " + runtime.toString());

            Robot.update();

            Robot.beaconLeft.setPosition(0.0);
            Robot.beaconRight.setPosition(0.0);

            Robot.nomMiddle.setPower(1.0f);

            Robot.moveForward_encoder(2100, 0.6f);
            telemetry.addLine("MOVE 1 DONE!");
            telemetry.update();
            Thread.sleep(101);

            Robot.flywheelLeft.setPower(0.4f);
            Robot.flywheelRight.setPower(0.4f);

            Thread.sleep(500);

            Robot.conveyor.setPower(1.0f);
            telemetry.addLine("CONVEYOR!");
            telemetry.update();
            Thread.sleep(3000);
            Robot.conveyor.setPower(0.0f);
            Robot.flywheelLeft.setPower(0.0f);
            Robot.flywheelRight.setPower(0.0f);
            Robot.nomMiddle.setPower(0.0f);

            Robot.turnToHeading(40, 0.6f);
            telemetry.addLine("TURN 1 DONE");
            telemetry.update();
            Thread.sleep(100);

            Robot.moveForward_encoder(3500, 0.5f);
            telemetry.addLine("MOVE 2 DONE");
            telemetry.update();
            Thread.sleep(100);

            Robot.turnToHeading(90, 0.5f);
            telemetry.addLine("TURN 2 DONE");
            telemetry.update();
            Thread.sleep(100);

            // align to gears
            telemetry.addLine("TRYING TO ALIGN...");
            telemetry.update();
            alignTest();
            Thread.sleep(250);

            /*Robot.turnToHeading(90, 0.8f);
            telemetry.addLine("ALIGN TURN DONE");
            telemetry.update();
            Thread.sleep(100);*/

            for (int i = 0; i < 5000; i++) {
                telemetry.addData("r", Robot.beaconColor.red());
                telemetry.addData("g", Robot.beaconColor.green());
                telemetry.addData("b", Robot.beaconColor.blue());
                telemetry.update();
                Thread.sleep(1);
            }

            // determine beacon color
            Alliance rightColor = Robot.getBeaconRightColor();
            if (rightColor == currentAlliance) {
                telemetry.addLine("PRESSING RIGHT");
                Robot.beaconLeft.setPosition(0.0);
                Robot.beaconRight.setPosition(0.0);
            } else {
                telemetry.addLine("PRESSING LEFT");
                Robot.beaconLeft.setPosition(1.0);
                Robot.beaconRight.setPosition(1.0);
            }
            telemetry.update();
            Thread.sleep(250);

            telemetry.addLine("PRESSY PRESS");
            telemetry.update();
            Robot.moveForward_encoder(400, 0.5f);
            Thread.sleep(250);

            /*if (rightColor != currentAlliance) {
                Thread.sleep(250);
                Robot.beaconServo.setPosition(0.65);
            }*/

            telemetry.addLine("RETREAT");
            telemetry.update();
            //moveForward_encoder(-200, -0.5f);
            Thread.sleep(500);

            telemetry.addLine("THE END OF PART 1");
            telemetry.update();

            /*//turn 90 degrees to the right
            Robot.turnToHeading(270, .7);

            //go forward until the the line is sensed
            while(onLine()){
                Robot.moveForward_encoder(20, 0.3f);
                Robot.idle();
            }*/

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

    public void alignTest() throws InterruptedException { // only works for gears!!!
        while (true) {
            if (!Robot.vuforia.hasLocation()) {
                Robot.vuforia.update();
                continue;
            }
            VectorF translation = Robot.vuforia.getLocation();
            Orientation orientation = Robot.vuforia.getOrientation();
            boolean orientationGood = ((orientation.thirdAngle > -5 && orientation.thirdAngle < 0) ||
                                        (orientation.thirdAngle > 0 && orientation.thirdAngle < 5));

            if (translation.get(0) < -1000) {
                // we're too close, just stop
                telemetry.addLine("ALIGNTEST IS DONE");
                Robot.leftMotors(0.0f);
                Robot.rightMotors(0.0f);
                return;
            }


            if (!orientationGood && orientation.thirdAngle < 0) {
                Robot.leftMotors(0.2f);
                Robot.rightMotors(0.5f);
            } else if (!orientationGood && orientation.thirdAngle > 0) {
                Robot.leftMotors(0.5f);
                Robot.rightMotors(0.2f);
            } else if (translation.get(0) > -1000) {
                Robot.leftMotors(0.5f);
                Robot.rightMotors(0.5f);
            } else {
                telemetry.addLine("ALIGNTEST IS DONE");
                Robot.leftMotors(0.0f);
                Robot.rightMotors(0.0f);
                return;
            }
            Robot.update();

            telemetry.addData("Pos", Robot.vuforia.getLocationAsString());
            telemetry.addData("Ptest", translation.get(0));
            telemetry.addData("Still going", (translation.get(0) > -1400));
            telemetry.addData("Test", Robot.leftMotor.getCurrentPosition());
            telemetry.addData("Second Angle", orientation.secondAngle);
            telemetry.update();
            Robot.idle();
        }
    }

}

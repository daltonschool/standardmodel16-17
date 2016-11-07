package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;

@Autonomous(name="Autonomous Operation")
public class AutonomousOperation extends LinearOpMode
{
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

            Robot.moveForward_encoder(2000, 0.3f);
            telemetry.addLine("MOVE 1 DONE");
            telemetry.update();
            Thread.sleep(100);

            Robot.turnToHeading(55, 0.2f);
            telemetry.addLine("TURN 1 DONE");
            telemetry.update();
            Thread.sleep(100);

            Robot.moveForward_encoder(4000, 0.3f);
            telemetry.addLine("MOVE 2 DONE");
            telemetry.update();
            Thread.sleep(100);

            Robot.turnToHeading(75, 0.3f);
            telemetry.addLine("TURN 2 DONE");
            telemetry.update();
            Thread.sleep(100);

            /*turnToHeading(90, 0.2f);
            telemetry.addLine("TURN 2 DONE");
            telemetry.update();
            Thread.sleep(1000);*/

            // align to gears
            telemetry.addLine("TRYING TO ALIGN...");
            telemetry.update();
            alignTest();
            Thread.sleep(250);

            /*for (int i = 0; i < 500; i++) {
                telemetry.addData("r", beaconColor.red());
                telemetry.addData("g", beaconColor.green());
                telemetry.addData("b", beaconColor.blue());
                telemetry.update();
                Thread.sleep(1);
            }*/

            // determine beacon color
            Alliance rightColor = Robot.getBeaconRightColor();
            if (rightColor == currentAlliance) {
                telemetry.addLine("PRESSING RIGHT");
                Robot.beaconServo.setPosition(0.0);
            } else {
                telemetry.addLine("PRESSING LEFT");
                Robot.beaconServo.setPosition(0.6274509804);
            }
            telemetry.update();
            Thread.sleep(250);

            telemetry.addLine("PRESSY PRESS");
            telemetry.update();
            Robot.moveForward_encoder(400, 0.5f);
            Thread.sleep(250);

            if (rightColor != currentAlliance) {
                Thread.sleep(250);
                Robot.beaconServo.setPosition(0.65);
            }

            telemetry.addLine("RETREAT");
            telemetry.update();
            //moveForward_encoder(-200, -0.5f);
            Thread.sleep(500);

            telemetry.addLine("THE END");
            telemetry.update();

            // press button
            requestOpModeStop();

            telemetry.update();
            Robot.idle();
        }
    }

    /*public void alignTo(VuforiaTrackable trackable) {
        trackable.getLocation()
    }*/

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
            boolean orientationGood = ((orientation.thirdAngle > -175 && orientation.thirdAngle < -180) ||
                                        (orientation.thirdAngle > 175 && orientation.thirdAngle < 180));

            if (translation.get(0) < -1400) {
                // we're too close, just stop
                telemetry.addLine("ALIGNTEST IS DONE");
                Robot.leftMotors(0.0f);
                Robot.rightMotors(0.0f);
                return;
            }

            if (!orientationGood && orientation.thirdAngle < 0) {
                Robot.leftMotors(0.1f);
                Robot.rightMotors(0.05f);
            } else if (!orientationGood && orientation.thirdAngle > 0) {
                Robot.leftMotors(0.05f);
                Robot.rightMotors(0.1f);
            } else if (translation.get(0) > -1400) {
                //moveForward(Math.abs((-1100 - translation.get(0)) / 4), 0.2f);
                Robot.leftMotors(0.1f);
                Robot.rightMotors(0.1f);
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
            telemetry.update();
            Robot.idle();
        }
    }

}

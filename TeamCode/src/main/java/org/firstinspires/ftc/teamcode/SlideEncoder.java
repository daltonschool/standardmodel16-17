package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "SlideEncoder", group = "TeleOp")
public class SlideEncoder extends OpMode {

    DcMotor liftl;
    DcMotor liftr;
    double powerMultiplier = 0.5;
    double maxLiftHeight = 3200;
    double startPosL;
    double startPosR;

    int leftState = 0;
    int rightState = 0;

    public void init() {
        liftl = hardwareMap.dcMotor.get("lifttest2");
        liftr = hardwareMap.dcMotor.get("lifttest1");

        startPosL = liftl.getCurrentPosition();
        startPosR = liftr.getCurrentPosition();
    }

    public void loop() {
        double leftEncoder = liftl.getCurrentPosition() - startPosL;
        double rightEncoder = liftr.getCurrentPosition() - startPosR;

        double leftMultiplier = 1;
        if (leftState != 0) {
            double leftTarget = (leftState == 1 ? maxLiftHeight : 300);
            double leftDist = Math.abs(leftTarget - leftEncoder);
            leftMultiplier = Math.min(leftDist * 0.005, 1);
        }

        double rightMultiplier = 1;
        if (rightState != 0) {
            double rightTarget = (rightState == 1 ? maxLiftHeight : 300);
            double rightDist = Math.abs(rightTarget - ((rightEncoder) * -1));
            rightMultiplier = Math.min(rightDist * 0.005, 1);
        }

        if (gamepad1.a) {
            leftState = 1;
            rightState = 1;
        } else if (gamepad1.b) {
            leftState = -1;
            rightState = -1;
        }

        if (leftState == 1 && leftEncoder > maxLiftHeight) {
            leftState = 0;
            liftl.setPower(0);
        } else if (leftState == -1 && leftEncoder < 300) {
            leftState = 0;
            liftl.setPower(0);
        }

        if (rightState == 1 && rightEncoder < -maxLiftHeight) {
            rightState = 0;
            liftr.setPower(0);
        } else if (rightState == -1 && rightEncoder > -300) {
            rightState = 0;
            liftr.setPower(0);
        }

        liftl.setPower(leftState * powerMultiplier * leftMultiplier);
        liftr.setPower(-1 * rightState * powerMultiplier * rightMultiplier);

        telemetry.addData("Left Encoder Val:", leftEncoder);
        telemetry.addData("Right Encoder Val:", rightEncoder);
        telemetry.addData("Left state", leftState);
        telemetry.addData("Right state", rightState);

        //completion
        telemetry.update();

    }
}

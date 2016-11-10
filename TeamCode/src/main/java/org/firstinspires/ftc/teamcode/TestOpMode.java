package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Test OpMode", group="Tests")
public class TestOpMode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Utils.showToast("This is a test.", 0);
        waitForStart();
        while (opModeIsActive()) {
            Utils.showToast("Hello!", 0);

            requestOpModeStop();

            telemetry.update();
            idle();
        }
    }
}

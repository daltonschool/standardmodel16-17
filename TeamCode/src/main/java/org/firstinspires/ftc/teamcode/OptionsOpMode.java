package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.internal.network.NetworkConnectionHandler;
import org.firstinspires.ftc.teamcode.options.OptionsHTTPD;

import java.io.IOException;

@Autonomous(name="Options")
@Disabled
public class OptionsOpMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        try {
            OptionsHTTPD o = new OptionsHTTPD();
        } catch (IOException e) {
            telemetry.addLine("Error loading optionshttpd!");
            telemetry.update();
            e.printStackTrace();
            requestOpModeStop();
            return;
        }
        while (opModeIsActive()) {
            telemetry.addLine("Listening on port 9372!");
            telemetry.update();
            idle();
        }
    }
}
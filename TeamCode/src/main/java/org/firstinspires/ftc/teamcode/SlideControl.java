package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by student on 1/25/17.
 */

@TeleOp(name = "SlideControl", group = "TeleOp")
public class SlideControl extends OpMode {

    DcMotor liftl;
    DcMotor liftr;
    double powerMultiplier = 1;
    double targetliftheight;
    boolean justpressed1 = false;
    boolean justpressed2 = false;

    //init
    public void init() {
        liftl = hardwareMap.dcMotor.get("lifttest2");
        liftr = hardwareMap.dcMotor.get("lifttest1");
    }

    public void loop() {

        //woot lift code
        if (gamepad1.a == true) {
            liftl.setPower(powerMultiplier);
            liftr.setPower(-powerMultiplier);
            justpressed1 = true;
        } else if (gamepad1.b == true) {
            liftl.setPower(-powerMultiplier);
            liftr.setPower(powerMultiplier);
            justpressed2 = true;

        //acceleration dampening
        } else {
            liftl.setPower(0);
            liftr.setPower(0);
            if (justpressed1 == true) {
                for (double a = powerMultiplier; a > 0; a -= 0.1) {
                    liftl.setPower(a);
                    liftr.setPower(-a);
                    try {
                        Thread.sleep(40);
                    } catch (InterruptedException e) { }
                }
                justpressed1 = false;
            }
            if (justpressed2 == true) {
                for (double a = powerMultiplier; a > 0; a -= 0.1) {
                    liftl.setPower(-a);
                    liftr.setPower(a);
                    try {
                        Thread.sleep(40);
                    } catch (InterruptedException e) { }
                }
                justpressed2 = false;
            }
        }
        //// TODO: add encoder stopping --> This should run into new Delta teleop
        liftl.getCurrentPosition();
    }
}
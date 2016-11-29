package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * Demonstrates empty OpMode
 */
@TeleOp(name = "GammaDrive3Test", group = "TeleOp")
public class GammaDrive3Test extends OpMode {


    DcMotor launchLeft;
    DcMotor launchRight;

    DcMotor leftback;
    DcMotor rightback;

    DcMotor inside_nom;
    DcMotor outside_nom;
    DcMotor lift;

    Servo bpleft;
    Servo bpright;

    double powerleft;
    double powerright;

    double launchpower;


    boolean upprevstatelaunchspeed;
    boolean downprevstatelaunchspeed;

    double lastuppresslaunchspeed;
    double lastdownpresslaunchspeed;

    boolean prevstatelaunchtoggle;

    double lastpresslaunchtoggle;

    boolean launching;

    //for smoothing driving
    double lastpowerright;
    double lastpowerleft;

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        //
        launchLeft = hardwareMap.dcMotor.get("launch_left");
        launchRight = hardwareMap.dcMotor.get("launch_right");

        leftback = hardwareMap.dcMotor.get("drive_left");
        rightback = hardwareMap.dcMotor.get("drive_right");
        inside_nom = hardwareMap.dcMotor.get("inside_nom");
        outside_nom = hardwareMap.dcMotor.get("outside_nom");
        lift = hardwareMap.dcMotor.get("lift");

        hardwareMap.dcMotorController.get("Motor Controller 1").setMotorMode(1, DcMotor.RunMode.RUN_USING_ENCODER);
        hardwareMap.dcMotorController.get("Motor Controller 1").setMotorMode(2, DcMotor.RunMode.RUN_USING_ENCODER);

        launchpower = .4;

        bpright = hardwareMap.servo.get("leftBeacon");
        bpleft = hardwareMap.servo.get("rightBeacon");

        upprevstatelaunchspeed = false;
        downprevstatelaunchspeed = false;

        lastuppresslaunchspeed = 0;
        lastdownpresslaunchspeed = 0;

        lastpowerright = 0;
        lastpowerleft = 0;

    }

    /*
       * Code to run when the op mode is first enabled goes here
       * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
       */
    @Override
    public void init_loop() {
    }

    /*
     * This method will be called ONCE when start is pressed
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * This method will be called repeatedly in a loop
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
     */
    @Override
    public void loop() {
        telemetry.addData("Status", "Run Time: " + runtime.toString());

        telemetry.addData("Launch Power:", launchpower);

        //Launch
        if (gamepad2.dpad_up == true && upprevstatelaunchspeed == false && runtime.time() - lastuppresslaunchspeed > .5) {
            launchpower += .05;
            upprevstatelaunchspeed = true;
            lastuppresslaunchspeed = runtime.time();
        } else {
            upprevstatelaunchspeed = false;
        }
        if (gamepad2.dpad_down ==  true && downprevstatelaunchspeed == false && runtime.time() - lastdownpresslaunchspeed > .5) {
            launchpower -= .05;
            downprevstatelaunchspeed = true;
            lastdownpresslaunchspeed = runtime.time();
        } else {
            downprevstatelaunchspeed = false;
        }

        if (gamepad1.right_bumper ==  true && prevstatelaunchtoggle == false && runtime.time() - lastpresslaunchtoggle > .5) {
            launching = !launching;
            prevstatelaunchtoggle = true;
            lastpresslaunchtoggle = runtime.time();
        } else {
            prevstatelaunchtoggle = false;
        }


        if (launching == true && gamepad1.left_bumper == false) {
            launchRight.setPower(-launchpower);
            launchLeft.setPower(launchpower);
        } else if (launching == false && gamepad1.left_bumper == true){
            launchRight.setPower(.2);
            launchLeft.setPower(-.2);
        } else {
            launchRight.setPower(0);
            launchLeft.setPower(0);
        }



        launchpower = trim(launchpower);

        //Drive

        powerright = gamepad1.left_stick_y;
        powerleft = gamepad1.left_stick_y;

        powerright += gamepad1.right_stick_x;
        powerleft -= gamepad1.right_stick_x;


        // Anti-Jerkiness (hopefully)
        if (Math.abs(powerright - lastpowerright) > .7) {
            powerright = Math.abs(powerright + powerleft)/2;
        }

        if (Math.abs(powerleft - lastpowerleft) > .7) {
            powerleft = Math.abs(powerleft + powerleft)/2;
        }

        leftback.setPower(-trim(powerleft));
        rightback.setPower(trim(powerright));


        if (gamepad2.a == true) {
            inside_nom.setPower(1);
        } else if (gamepad2.y == true) {
            inside_nom.setPower(-1);
        } else {
            inside_nom.setPower(0);
        }

        if (gamepad2.b == true) {
            outside_nom.setPower(1);
        } else if (gamepad2.x == true) {
            outside_nom.setPower(-1);
        } else {
            outside_nom.setPower(0);
        }

        //Lift
        if (gamepad2.right_trigger != 0 && gamepad2.left_trigger == 0) {
            bpright.setPosition(0);
        } else {
            bpright.setPosition(1);
        }

        if (gamepad2.left_trigger != 0 && gamepad2.right_trigger == 0) {
            bpleft.setPosition(1);
        } else {
            bpleft.setPosition(0);
        }

        if (gamepad2.left_bumper == true && gamepad2.right_bumper == false) {
            lift.setPower(1);
        } else if (gamepad2.left_bumper == false && gamepad2.right_bumper == true) {
            lift.setPower(-1);
        } else {
            lift.setPower(0);
        }

        //

//        if (gamepad1.dpad_down == true) {
//            nom.setPower(.5);
//        }
//        if (gamepad1.dpad_right == true) {
//            nom.setPower(.5);
////            feeder.setPower(.3);
//        }
//        if (gamepad1.dpad_up == true) {
////            feeder.setPower(.3);
//        }
    }

    public double trim (double number) {
        if (number > 1) {
            number = 1;
        } else if (number < -1) {
            number = -1;
        }
        return number;
    }
}

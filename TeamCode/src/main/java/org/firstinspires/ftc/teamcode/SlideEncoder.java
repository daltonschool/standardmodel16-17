package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by student on 1/25/17.
 */

@TeleOp(name = "SlideEncoder", group = "TeleOp")
public class SlideEncoder extends OpMode {

    DcMotor liftl;
    DcMotor liftr;
    double powerMultiplier = 1;
    double liftmaxheight;
    double decellerationdampen = 0;
    //boolean justpressed1 = false;
    //boolean justpressed2 = false;
    double startPosL;
    double startPosR;

    public void init(){
        liftl = hardwareMap.dcMotor.get("lifttest2");
        liftr = hardwareMap.dcMotor.get("lifttest1");

        startPosL = liftl.getCurrentPosition();
        startPosR = liftr.getCurrentPosition();
    }
    public void loop(){
//// TODO: 1/25/17 implement acceleration for encoder limiting

        //code for liftl
        if(liftl.getCurrentPosition()- startPosL < (liftmaxheight-decellerationdampen)){
            if(gamepad1.a == true){
                liftl.setPower(powerMultiplier);
            }
            else if(gamepad1.b == true){
                liftl.setPower(-powerMultiplier);

            }
        }
        telemetry.addData("Left Encoder Val:", liftl.getCurrentPosition()-startPosL);
        //code for liftr
        if(liftr.getCurrentPosition()- startPosR < (liftmaxheight-decellerationdampen)){
            if(gamepad1.a == true){
                liftl.setPower(-powerMultiplier);
            }
            else if(gamepad1.b == true){
                liftl.setPower(powerMultiplier);

            }
        }
        telemetry.addData("Right Encoder Val:", liftr.getCurrentPosition()-startPosR);

        //completion
        telemetry.update();
    }
}

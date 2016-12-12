package org.firstinspires.ftc.teamcode.tasks;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.taskutil.Task;

public class LineFollowingTask extends Task {
    public LineFollowingTask(Object e) {
        super(e);
    }
    //PIDController pidObject = new PIDController(.1,.1,.15);
    double leftPower = .2;
    double rightPower = .2;
    double errorR = 0.0;
    //double errorL = 0.0;
    double Kp = (1 - leftPower) / (25);

    @Override
    public void init() {

    }

    @Override
    public void run() throws InterruptedException {
        Robot.rightMotors(0);
        Robot.leftMotors(0);
        Thread.sleep(50);
        while(true) {
            Robot.update();
            Robot.telemetry.addData("rightSensorValue: ", Robot.rightLineColor.whiteReading());
            Robot.telemetry.update();
            leftPower = .2;
            rightPower = .2;
            errorR = Robot.rightLineColor.whiteReading() - 15;
            if (Robot.rightLineColor.whiteReading() > 20) {
                leftPower = leftPower * (1 + (Kp * errorR));
                rightPower = rightPower * (1 - (Kp * errorR));
            }
            else if (Robot.rightLineColor.whiteReading() < 10) {
                rightPower = rightPower * (1 - (Kp * errorR));
                leftPower = leftPower * (1 + (Kp * errorR));
            }
            else {
                leftPower = .2;
                rightPower = .2;
            }
            Robot.leftMotors(leftPower);
            Robot.rightMotors(rightPower);
            Thread.sleep(50);
            if (Robot.vuforia.hasLocation() && Robot.vuforia.getLocation().get(0) < -1300) {
                Robot.leftMotors(0.0f);
                Robot.rightMotors(0.0f);
                Thread.sleep(2000);
                return;
            }
            Robot.telemetry.update();
            Robot.idle();
        }
    }
}

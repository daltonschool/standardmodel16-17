package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Autonomous Operation - BLUE - SO")
public class AutonomousBlueShotsOnly extends AutonomousOperation {
    public Alliance getCurrentAlliance() {
        return Alliance.BLUE;
    }
    public boolean onlyShoots() {
        return true;
    }
}

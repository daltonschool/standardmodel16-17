package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Autonomous Operation - RED")
public class AutonomousRed extends AutonomousOperation {
    public Alliance getCurrentAlliance() { return Alliance.RED; }
    public boolean onlyShoots() {
        return false;
    }
}

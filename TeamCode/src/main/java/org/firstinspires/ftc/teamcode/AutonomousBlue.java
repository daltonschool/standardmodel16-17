package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Autonomous Operation - BLUE")
public class AutonomousBlue extends AutonomousOperation {
    public Alliance getCurrentAlliance() {
        return Alliance.BLUE;
    }
}

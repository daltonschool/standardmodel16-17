package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name="Autonomous Operation - BLUE")
@Disabled
public class AutonomousBlue extends AutonomousOperation {
    public Alliance getCurrentAlliance() {
        return Alliance.BLUE;
    }
}

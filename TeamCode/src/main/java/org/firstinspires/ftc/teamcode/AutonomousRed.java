package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name="Autonomous Operation - RED")
@Disabled
public class AutonomousRed extends AutonomousOperation {
    public Alliance getCurrentAlliance() { return Alliance.RED; }
}

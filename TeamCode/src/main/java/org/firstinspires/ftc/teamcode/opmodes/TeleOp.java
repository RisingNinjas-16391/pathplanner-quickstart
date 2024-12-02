package org.firstinspires.ftc.teamcode.opmodes;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotContainer;

import edu.wpi.first.wpilibj.ftclib.opmode.CommandOpMode;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = "Teleop")
public class TeleOp extends CommandOpMode {
    private Telemetry m_telemetry;

    @Override
    public void robotInit() {
        m_telemetry = telemetry;
        new RobotContainer(hardwareMap, m_telemetry, gamepad1, gamepad2, 0); //Uses heavily modified untested hardware
    }

    @Override
    public void robotPeriodic() {
        super.robotPeriodic();
        m_telemetry.update();

    }

}
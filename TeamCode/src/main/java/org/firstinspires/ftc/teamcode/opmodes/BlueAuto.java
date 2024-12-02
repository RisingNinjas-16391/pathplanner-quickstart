package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.lib.ftclib.opmode.CommandOpMode;

@Autonomous(name = "BlueAuto", group = "Auto")
public class BlueAuto extends CommandOpMode {
    private Telemetry m_telemetry;
    private RobotContainer m_robotContainer;

    @Override
    public void robotInit() {
        m_telemetry = telemetry;
        m_robotContainer = new RobotContainer(hardwareMap, m_telemetry, gamepad1, gamepad2, 1); //Uses heavily modified untested hardware
    }

    @Override
    public void robotPeriodic() {
        super.robotPeriodic();
        m_telemetry.update();
    }

    @Override
    public void enabledInit() {
        m_robotContainer.getAutoCommand(1).schedule();
    }

}
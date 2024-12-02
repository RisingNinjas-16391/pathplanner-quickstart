package edu.wpi.first.wpilibj.ftclib.opmode;

import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class CommandOpMode extends TimedRobotOpMode {

    @Override
    public void robotInit() {
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void enabledInit() {

    }

    @Override
    public void enabledPeriodic() {

    }

    @Override
    public void disabledInit() {
        CommandScheduler.getInstance().reset();
    }
}

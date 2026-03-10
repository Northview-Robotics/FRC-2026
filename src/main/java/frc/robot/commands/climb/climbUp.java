package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climb;

public class climbUp extends Command {
    private final Climb m_climb;

    public climbUp(Climb climbSubsystem) {
        this.m_climb = climbSubsystem;
        addRequirements(m_climb);
    }

    @Override
    public void execute() {
        m_climb.setArmPos(true);
    }
}

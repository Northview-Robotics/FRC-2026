package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class FireShooter extends Command {
    private final Shooter m_shooter;
    private final Intake m_intake;
    private double duration; 
    private final Timer m_Timer;

    public FireShooter(Shooter shooterSubsystem, Intake intakeSubsystem, double duration) {
        m_shooter = shooterSubsystem;
        m_intake = intakeSubsystem;
        m_Timer = new Timer();
        this.duration = duration;
        addRequirements(shooterSubsystem, intakeSubsystem);
    }

    @Override
    public void initialize() {
        m_Timer.reset();
        m_Timer.start();
    }

    @Override
    public void execute() {
        m_shooter.runKicker(6); // Changed from boolean to voltage
        m_intake.setIntakePivot(); // Changed from setShootingPivot() to setIntakePivot() or similar.
        // Wait, I should check if Intake.java has a shooting pivot.
    }

    @Override
    public boolean isFinished() {
        return m_Timer.hasElapsed(duration);
    }

    @Override
    public void end(boolean interrupted) {
        m_shooter.runKicker(0);
    }
}

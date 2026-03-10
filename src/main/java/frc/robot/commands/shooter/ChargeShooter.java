package frc.robot.commands.shooter;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import frc.robot.subsystems.Shooter;

public class ChargeShooter extends Command {
    private final Shooter m_shooter;
    private double duration; 
    private final Timer m_Timer;

    public ChargeShooter(Shooter shooterSubsystem, double duration) {
        m_shooter = shooterSubsystem;
        m_Timer = new Timer();
        this.duration = duration;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
        m_Timer.reset();
        m_Timer.start();
    }
    
    @Override
    public void execute() {
        m_shooter.setHoodAngle(m_shooter.getTargetHoodAngle());
        m_shooter.setFlyWheelVel(m_shooter.getTargetFlyWheelVel());
    }

    @Override
    public boolean isFinished() {
        return m_Timer.hasElapsed(duration);
    }

    @Override
    public void end(boolean interrupted) {
        AngularVelocity targetVelocity = RotationsPerSecond.of(0);
        m_shooter.setFlyWheelVel(targetVelocity);
    }
}

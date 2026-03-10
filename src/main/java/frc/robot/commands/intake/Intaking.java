package frc.robot.commands.intake;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;


public class Intaking extends Command {
    private final Intake m_intake;
    private double duration; 
    private final Timer m_Timer;

    public Intaking(Intake intakeSubsystem, double duration) {
        this.m_intake = intakeSubsystem;
        m_Timer = new Timer();
        this.duration = duration;
        addRequirements(m_intake);
    }

    @Override
    public void initialize() {
        m_Timer.reset();
        m_Timer.start();
        m_intake.setIntakeActive(true);
    }

    @Override
    public void execute() {
        // Intake logic is now partially in the handler, but for commands we can set it active
    }

    @Override
    public boolean isFinished() {
        return m_Timer.hasElapsed(duration);
    }

    @Override
    public void end(boolean interrupted) {
        m_intake.setIntakeActive(false);
    }
}

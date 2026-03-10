package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.constants.idConstants;

import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class Indexer extends SubsystemBase {
    private static Indexer instance = null;

    private TalonFX indexingMotor;
    private final SmartMotorControllerConfig falconConfig;
    private SmartMotorController indexer;

    private Indexer() {
        indexingMotor = new TalonFX(idConstants.falcon500_I1); 

        falconConfig = new SmartMotorControllerConfig(this)
                .withClosedLoopController(0.00016541, 0, 0, RPM.of(5000), RotationsPerSecondPerSecond.of(2500))
                .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
                .withIdleMode(MotorMode.COAST)
                .withTelemetry("IndexerMotor", TelemetryVerbosity.HIGH)
                .withStatorCurrentLimit(Amps.of(20))
                .withMotorInverted(true)
                .withClosedLoopRampRate(Seconds.of(0.25))
                .withOpenLoopRampRate(Seconds.of(0.25))
                .withFeedforward(new SimpleMotorFeedforward(0.27937, 0.089836, 0.014557))
                .withSimFeedforward(new SimpleMotorFeedforward(0.27937, 0.089836, 0.014557))
                .withControlMode(ControlMode.CLOSED_LOOP);

        indexer = new TalonFXWrapper(indexingMotor, DCMotor.getFalcon500(1), falconConfig);
    }

    public void runIndexer(double voltage) {
        indexer.setVoltage(Volts.of(voltage));
    }

    public void indexerInputHandler(boolean shooterReady, boolean outtakeButton) {
        if (outtakeButton) {
            runIndexer(-6);
        } else if (shooterReady) {
            runIndexer(6);
        } else {
            runIndexer(0);
        }
    }

    public static Indexer getInstance() {
        if (instance == null) {
            instance = new Indexer();
        }
        return instance;
    }
}

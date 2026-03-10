package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.idConstants;
import frc.robot.constants.velocityMap;
import frc.robot.subsystems.automations.AutoAlign;
import frc.robot.constants.Constants;
import frc.robot.constants.angleMap;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.positional.Arm;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class Shooter extends SubsystemBase {
    private static Shooter instance = null;

    private TalonFX Lshooter1;
    private TalonFX Lshooter2;
    private TalonFX Rshooter1;
    private TalonFX Rshooter2;
    private TalonFX kickerMotor;
    private SparkMax hoodMotor;

    private SparkMaxConfig hoodMotorConfig;
    Follower followLFShooter;
    Follower followRFShooter;
    Follower followLShooter;

    private final SmartMotorControllerConfig krakenConfig;
    private final SmartMotorControllerConfig falconConfig;
    private final SmartMotorControllerConfig neo550Config;

    private SmartMotorController LShootingSystem;
    private SmartMotorController RShootingSystem;
    private SmartMotorController kickingSystem;
    private SmartMotorController hoodSystem;

    private final FlyWheelConfig shooterConfig;
    private final ArmConfig hoodConfig;
    private final Arm hood;
    private final FlyWheel mainShooter;

    private LEDS leds = LEDS.getInstance();
    private AutoAlign align = AutoAlign.getInstance();
    private Telemetry telemetry;
    private angleMap shootingAMap;
    private velocityMap shootingVMap;

    private Shooter() {
        Lshooter1 = new TalonFX(idConstants.krakenx60_S1);
        Lshooter2 = new TalonFX(idConstants.krakenx60_S2);
        Rshooter1 = new TalonFX(idConstants.krakenx60_S3);
        Rshooter2 = new TalonFX(idConstants.krakenx60_S4);
        kickerMotor = new TalonFX(idConstants.falcon500_S5);
        hoodMotor = new SparkMax(idConstants.neo550_S6, MotorType.kBrushless);

        hoodMotorConfig = new SparkMaxConfig();
        hoodMotor.configure(hoodMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        followLFShooter = new Follower(Lshooter1.getDeviceID(), MotorAlignmentValue.Aligned);
        followRFShooter = new Follower(Rshooter1.getDeviceID(), MotorAlignmentValue.Aligned);
        followLShooter = new Follower(Lshooter1.getDeviceID(), MotorAlignmentValue.Opposed);
        Lshooter2.setControl(followLFShooter);
        Rshooter1.setControl(followLShooter);
        Rshooter2.setControl(followRFShooter);

        krakenConfig = new SmartMotorControllerConfig(this)
                .withClosedLoopController(0.00016541, 0, 0, RPM.of(6000), RotationsPerSecondPerSecond.of(2500))
                .withGearing(new MechanismGearing(GearBox.fromReductionStages(1)))
                .withIdleMode(MotorMode.COAST)
                .withTelemetry("ShooterMotor", TelemetryVerbosity.HIGH)
                .withStatorCurrentLimit(Amps.of(40))
                .withMotorInverted(false)
                .withClosedLoopRampRate(Seconds.of(0.25))
                .withOpenLoopRampRate(Seconds.of(0.25))
                .withFeedforward(new SimpleMotorFeedforward(0.27937, 0.52, 0.14))
                .withSimFeedforward(new SimpleMotorFeedforward(0.27937, 0.52, 0.14))
                .withControlMode(ControlMode.CLOSED_LOOP);

        falconConfig = new SmartMotorControllerConfig(this)
                .withClosedLoopController(0.00016541, 0, 0, RPM.of(6300), RotationsPerSecondPerSecond.of(2500))
                .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
                .withIdleMode(MotorMode.COAST)
                .withTelemetry("KickerMotor", TelemetryVerbosity.HIGH)
                .withStatorCurrentLimit(Amps.of(40))
                .withMotorInverted(true)
                .withClosedLoopRampRate(Seconds.of(0.25))
                .withOpenLoopRampRate(Seconds.of(0.25))
                .withFeedforward(new SimpleMotorFeedforward(0.27937, 0.089836, 0.014557))
                .withSimFeedforward(new SimpleMotorFeedforward(0.27937, 0.089836, 0.014557))
                .withControlMode(ControlMode.CLOSED_LOOP);

        neo550Config = new SmartMotorControllerConfig(this)
                .withClosedLoopController(0.00016541, 0, 0, RPM.of(11000), RotationsPerSecondPerSecond.of(2500))
                .withGearing(new MechanismGearing(GearBox.fromReductionStages(100, 1)))
                .withIdleMode(MotorMode.BRAKE)
                .withTelemetry("HoodMotor", TelemetryVerbosity.HIGH)
                .withStatorCurrentLimit(Amps.of(40))
                .withMotorInverted(true)
                .withClosedLoopRampRate(Seconds.of(0.25))
                .withOpenLoopRampRate(Seconds.of(0.25))
                .withFeedforward(new SimpleMotorFeedforward(0.27937, 0.089836, 0.014557))
                .withSimFeedforward(new SimpleMotorFeedforward(0.27937, 0.089836, 0.014557))
                .withControlMode(ControlMode.CLOSED_LOOP);

        LShootingSystem = new TalonFXWrapper(Lshooter1, DCMotor.getKrakenX60(2), krakenConfig);
        RShootingSystem = new TalonFXWrapper(Rshooter1, DCMotor.getKrakenX60(2), krakenConfig);
        kickingSystem = new TalonFXWrapper(kickerMotor, DCMotor.getFalcon500(1), falconConfig);
        hoodSystem = new SparkWrapper(hoodMotor, DCMotor.getNeo550(1), neo550Config);

        shooterConfig = new FlyWheelConfig(LShootingSystem)
                .withDiameter(Inches.of(4))
                .withMass(Pounds.of(1.5))
                .withTelemetry("ShooterWheel", TelemetryVerbosity.HIGH)
                .withSoftLimit(RPM.of(-2892), RPM.of(2892))
                .withSpeedometerSimulation(RPM.of(2892));

        hoodConfig = new ArmConfig(hoodSystem)
                .withTelemetry("Hood", TelemetryVerbosity.HIGH)
                .withSoftLimits(Degrees.of(20), Degrees.of(30))
                .withHardLimit(Degrees.of(20), Degrees.of(30))
                .withLength(Constants.hoodArmLength)
                .withStartingPosition(Degrees.of(Constants.startingHoodAngle))
                .withMass(Constants.hoodMass);

        mainShooter = new FlyWheel(shooterConfig);
        hood = new Arm(hoodConfig);
    }

    public void runKicker(double voltage) {
        kickingSystem.setVoltage(Volts.of(voltage));
    }

    public AngularVelocity currentVelocityLog() {
        return mainShooter.getSpeed();
    }

    private double getVirtualTarget(Distance hubDistance) {
        telemetry = Telemetry.getInstance();
        ChassisSpeeds chassisVel = telemetry.currentVelocity;
        if (chassisVel == null) return hubDistance.in(Meters);
        double xVel = chassisVel.vxMetersPerSecond;
        double yVel = chassisVel.vyMetersPerSecond;
        double flightTime = hubDistance.in(Meters) * 0.4;

        double virtualTargetX = hubDistance.in(Meters) - (xVel * flightTime);
        double virtualTargetY = 0 - (yVel * flightTime);
        return Math.hypot(virtualTargetX, virtualTargetY);
    }

    public AngularVelocity getTargetFlyWheelVel() {
        double targetDist = getVirtualTarget(align.getHubDist());
        shootingVMap = velocityMap.getInstance();
        return RotationsPerSecond.of(shootingVMap.mainMap.get(targetDist));
    }

    public Angle getTargetHoodAngle() {
        double targetDist = getVirtualTarget(align.getHubDist());
        shootingAMap = angleMap.getInstance();
        return Degrees.of(shootingAMap.mainMap.get(targetDist));
    }

    public void setHoodAngle(Angle target) {
        hood.setAngle(target).schedule();
    }

    public void setFlyWheelVel(AngularVelocity speed) {
        mainShooter.setSpeed(speed).schedule();
    }

    public boolean isAtTargetSpeed() {
        AngularVelocity current = mainShooter.getSpeed();
        AngularVelocity target = getTargetFlyWheelVel();
        return Math.abs(current.in(RPM) - target.in(RPM)) < 100;
    }

    public void shooterInputManager(double rightTrigger) {
        if (rightTrigger > 0.3) {
            setHoodAngle(getTargetHoodAngle());
            setFlyWheelVel(getTargetFlyWheelVel());
            leds.shooterChargingWave();

            if (isAtTargetSpeed()) {
                runKicker(6); 
                leds.shooterReadyBlink();
            } else {
                runKicker(0);
            }
        } else {
            setFlyWheelVel(RotationsPerSecond.of(0));
            runKicker(0);
        }
    }

    public void flyWheelSysId(double voltage, double step, double duration) {
        mainShooter.sysId(Volts.of(voltage), Volts.of(step).per(Second), Seconds.of(duration)).schedule();
    }

    public void hoodSysId(double voltage, double step, double duration) {
        hood.sysId(Volts.of(voltage), Volts.of(step).per(Second), Seconds.of(duration)).schedule();
    }

    public static Shooter getInstance() {
        if (instance == null) {
            instance = new Shooter();
        }
        return instance;
    }
}

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.constants.Constants;
import frc.robot.subsystems.automations.Vision;
import frc.robot.subsystems.automations.AutoAlign;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.io.File;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import swervelib.SwerveDrive;
import swervelib.SwerveInputStream;
import swervelib.SwerveModule;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

public class Drive extends SubsystemBase {
    private static Drive instance = null;

    private File directory = new File(Filesystem.getDeployDirectory(), "swerve2");
    public SwerveDrive swerveDrive;
    private SwerveInputStream driveVel;

    private StructPublisher<Pose3d> publisher3d;
    private StructPublisher<ChassisSpeeds> publisherSpeed;
    private Pose2d currentPose2d;
    private Pose3d currentPose3d;

    private RobotConfig config;

    public int selectModule;

    SysIdRoutine driveSysID;
    SysIdRoutine angleSysID;
    Direction currentDir = Direction.kForward;
    private SwerveModule[] modules;

    private Drive() {
        try {
            SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
            swerveDrive = new SwerveParser(directory).createSwerveDrive(Constants.maxDriveSpeed, Constants.startPose);
        } catch (Exception e) {
            throw new RuntimeException(e);
        }

        publisherSpeed = NetworkTableInstance.getDefault().getStructTopic("MyChassisSpeed", ChassisSpeeds.struct).publish();
        publisher3d = NetworkTableInstance.getDefault().getStructTopic("/AdvantageScope/Robot/Pose", Pose3d.struct).publish();
        modules = swerveDrive.getModules();

        driveSysID = new SysIdRoutine(new SysIdRoutine.Config(),
                new SysIdRoutine.Mechanism(
                        (voltage) -> { modules[0].getDriveMotor().setVoltage(voltage.in(Volts)); },
                        (log) -> {
                            log.motor("drive1").voltage(Volts.of(modules[0].getDriveMotor().getVoltage()))
                                    .linearVelocity(MetersPerSecond.of(modules[0].getDriveMotor().getVelocity() / (Constants.secondsPerMinute * Constants.driveCircumferenceMeters)))
                                    .linearPosition(Meters.of(modules[0].getDriveMotor().getPosition() * Constants.driveCircumferenceMeters));
                        },
                        this
                )
        );

        angleSysID = new SysIdRoutine(new SysIdRoutine.Config(),
                new SysIdRoutine.Mechanism(
                        (voltage) -> { modules[0].getAngleMotor().setVoltage(voltage.in(Volts)); },
                        (log) -> {
                            log.motor("angle1").voltage(Volts.of(modules[0].getAngleMotor().getVoltage()))
                                    .angularVelocity(RotationsPerSecond.of(modules[0].getAngleMotor().getVelocity() / Constants.secondsPerMinute))
                                    .angularPosition(Rotations.of(modules[0].getAngleMotor().getPosition()));
                        },
                        this
                )
        );

        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
        }

        AutoBuilder.configure(
                this::getRobotPose,
                this::resetRobotPose,
                this::getRobotSpeed,
                (speeds, feedforwards) -> drive(speeds),
                new PPHolonomicDriveController(
                        new PIDConstants(5.0, 0.0, 0.0),
                        new PIDConstants(5.0, 0.0, 0.0)
                ),
                config,
                () -> {
                    var alliance = DriverStation.getAlliance();
                    return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
                },
                this
        );
    }

    public void driveInputHandler(double x, double y, double theta, boolean autoAlignHold) {
        if (autoAlignHold) {
            rotateToYaw(x, y);
        } else {
            swerveSupplier(x, y, theta);
        }
    }

    public void swerveSupplier(double x, double y, double theta) {
        SwerveInputStream angularVelStream = SwerveInputStream.of(
                swerveDrive,
                () -> x,
                () -> y
        ).withControllerRotationAxis(() -> theta).scaleRotation(0.8).deadband(Constants.deadband).allianceRelativeControl(true);

        driveVel = angularVelStream.copy().withControllerHeadingAxis(() -> x, () -> y);
        driveField(driveVel.get());

        publisherSpeed.set(swerveDrive.getFieldVelocity());
    }

    private void rotateToYaw(double x, double y) {
        AutoAlign alignBot = AutoAlign.getInstance();
        Rotation2d targetAngle = alignBot.getHubYaw();
        swerveDrive.driveFieldOriented(
                swerveDrive.swerveController.getTargetSpeeds(
                        x,
                        y,
                        targetAngle.getRadians(),
                        swerveDrive.getYaw().getRadians(),
                        swerveDrive.getMaximumChassisVelocity()
                )
        );
    }

    private void drive(ChassisSpeeds vel) {
        swerveDrive.drive(vel);
    }

    private void driveField(ChassisSpeeds vel) {
        swerveDrive.driveFieldOriented(vel);
    }

    public ChassisSpeeds getRobotSpeed() {
        return swerveDrive.getRobotVelocity();
    }

    public Pose2d getRobotPose() {
        return swerveDrive.getPose();
    }

    public void resetRobotPose(Pose2d pose) {
        swerveDrive.resetOdometry(pose);
    }

    public void updatePoseEstimator() {
        Vision.getInstance().updateVision();
        swerveDrive.updateOdometry();
    }

    @Override
    public void periodic() {
        Rotation2d heading = swerveDrive.getYaw();
        currentPose2d = swerveDrive.getPose();
        currentPose3d = new Pose3d(currentPose2d.getX(), currentPose2d.getY(), 0, new Rotation3d(0, 0, heading.getRadians()));
        publisher3d.set(currentPose3d);
    }

    public static Drive getInstance() {
        if (instance == null) {
            instance = new Drive();
        }
        return instance;
    }
}

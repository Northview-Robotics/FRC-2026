package frc.robot.subsystems.automations;

import static edu.wpi.first.units.Units.Meters;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.autoConstants;
import frc.robot.subsystems.Drive;

public class AutoAlign extends SubsystemBase{
    private static AutoAlign align = null;
    private Drive drivetrain = Drive.getInstance();
    Map<Pose2d, Pose2d> intermediatePoseMap = new HashMap<>();
    Map<Integer, Pose2d> tagMap = new HashMap<>();

    //Commented out because all of the auto align paths haven't been made yet

    private AutoAlign(){
        intermediatePoseMap.put(autoConstants.BlueTrenchLeft, autoConstants.BlueTrenchLeftI);
        intermediatePoseMap.put(autoConstants.RedTrenchLeft, autoConstants.RedTrenchLeftI);
        intermediatePoseMap.put(autoConstants.RedTrenchRight, autoConstants.RedTrenchRightI);
        intermediatePoseMap.put(autoConstants.BlueTrenchRight, autoConstants.BlueTrenchRightI);
    }

    public Rotation2d getHubYaw(){
        Pose2d botPos = drivetrain.getRobotPose();
        Pose2d hubPos = getHubPos();

        double targetRadians = Math.atan2(
            hubPos.getY() - botPos.getY(),
            hubPos.getX() - botPos.getX()
        );

        return new Rotation2d(targetRadians);
    }

    public Distance getHubDist(){
        Pose2d botPos = drivetrain.getRobotPose();
        Pose2d hubPos = getHubPos();

        Distance hubDist = Meters.of(
            Math.hypot((hubPos.getX() - botPos.getX()), (hubPos.getY() - botPos.getY()))
        );
        return hubDist;
    }

    private Pose2d getHubPos(){
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        Pose2d hubPos = (alliance == Alliance.Blue) ? autoConstants.HubB : autoConstants.HubR;
        return hubPos;
    }

    //Path finding
    public void followGeneratedPath(Pose2d intermediate, Pose2d target) {
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(drivetrain.getRobotPose(), intermediate, target);
        PathConstraints constraints = new PathConstraints(4.0, 3.0, 2 * Math.PI, 2 * Math.PI);
        PathPlannerPath path = new PathPlannerPath(
            waypoints,
            constraints,
            null,
            new GoalEndState(0.0, target.getRotation())
        );
        path.preventFlipping = true;
        AutoBuilder.followPath(path).schedule();  //Follow path
    }

    public void travelToTower(boolean confirmAlign){
        if(confirmAlign){
            Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
            Pose2d towerPos = (alliance == Alliance.Blue) ? autoConstants.TowerB : autoConstants.TowerR;
            Pose2d towerIPos = (alliance == Alliance.Blue) ? autoConstants.TowerBI : autoConstants.TowerRI;
            followGeneratedPath(towerIPos, towerPos);
        }
    }

    public void alignToTrench(boolean confirmAlign){
        Optional<Pose2d> trenchPose = getTrenchToAlign();

        if(trenchPose.isPresent() && confirmAlign){
            Pose2d targetTrench = trenchPose.get();
            Pose2d targetTrenchI = intermediatePoseMap.get(targetTrench);
            followGeneratedPath(targetTrench, targetTrenchI);
        }
    }

    public Optional<Pose2d> getTrenchToAlign() {
        Pose2d botPose = drivetrain.getRobotPose();
        if (botPose.getTranslation().getDistance(autoConstants.BlueTrenchLeft.getTranslation()) < 1) {
            return Optional.of(autoConstants.BlueTrenchLeft);
        }
        if (botPose.getTranslation().getDistance(autoConstants.BlueTrenchRight.getTranslation()) < 1) {
            return Optional.of(autoConstants.BlueTrenchRight);
        }

        if (botPose.getTranslation().getDistance(autoConstants.RedTrenchLeft.getTranslation()) < 1) {
            return Optional.of(autoConstants.RedTrenchLeft);
        }

        if (botPose.getTranslation().getDistance(autoConstants.RedTrenchRight.getTranslation()) < 1) {
            return Optional.of(autoConstants.RedTrenchRight);
        }

        return Optional.empty();

        // Pose2d botPose = drivetrain.getRobotPose();
        // return botPose.getTranslation().getDistance(autoConstants.BlueTrenchLeft.getTranslation()) < 1 ||
        //        botPose.getTranslation().getDistance(autoConstants.BlueTrenchRight.getTranslation()) < 1 ||
        //        botPose.getTranslation().getDistance(autoConstants.RedTrenchRight.getTranslation()) < 1 ||
        //        botPose.getTranslation().getDistance(autoConstants.RedTrenchLeft.getTranslation()) < 1;
    }

    public static AutoAlign getInstance(){
        if (align == null){
            align = new AutoAlign();
        }
        return align;
    }
}
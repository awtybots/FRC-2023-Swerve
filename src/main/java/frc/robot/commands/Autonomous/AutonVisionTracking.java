package frc.robot.commands.Autonomous;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.Swerve.Swerve;

public class AutonVisionTracking extends SequentialCommandGroup {
    public AutonVisionTracking(Swerve s_Swerve, LimelightSubsystem s_Limelight) {

        double beta = Math.toRadians(s_Limelight.getHorizontalOffset());
        double alpha = Math.toRadians(s_Limelight.getHorizontalRotation());
        double initial_distance = s_Limelight.getDistance();

        double hypotenuse = Math.sqrt(2 * Math.pow(initial_distance, 2) * (1 - Math.cos(alpha + beta)));
        double theta = Math.asin((initial_distance * Math.sin(alpha + beta)) / hypotenuse);
        double delta = Math.PI / 2 - beta - theta;

        double displacementX = hypotenuse * Math.sin(delta);
        double displacementY = hypotenuse * Math.cos(delta);

        PathPlannerTrajectory trajectory1 =
                PathPlanner.generatePath(
                        new PathConstraints(
                                Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared),
                        new PathPoint(new Translation2d(0, 0), Rotation2d.fromDegrees(0)),
                        new PathPoint(
                                new Translation2d(displacementX, displacementY), Rotation2d.fromDegrees(-alpha)));
        // ? Why trajectory 2?
        PathPlannerTrajectory trajectory2 =
                PathPlanner.generatePath(
                        new PathConstraints(
                                Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared),
                        new PathPoint(new Translation2d(0, 0), Rotation2d.fromDegrees(0)),
                        new PathPoint(
                                new Translation2d(
                                        s_Limelight.getDistance() - Constants.LimeLightConstants.distanceToTarget, 0),
                                Rotation2d.fromDegrees(0)));

        // PathPlannerTrajectory exampleTrajectory =
        //         PathPlanner.generatePath(
        //                 new PathConstraints(
        //                         Constants.AutoConstants.kMaxSpeedMetersPerSecond,
        //                         Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared),
        //                 new PathPoint(new Translation2d(0, 0), Rotation2d.fromDegrees(0)),
        //                 new PathPoint(new Translation2d(0, -1), Rotation2d.fromDegrees(0)));

        // Trajectory exampleTrajectory =
        //         TrajectoryGenerator.generateTrajectory(
        //                 // Start at the origin facing the +X direction
        //                 new Pose2d(0, 0, new Rotation2d(0)),
        //                 // List.of(new Translation2d(1, 0)),
        //                 List.of(new Translation2d(0, firstYTranslation/2)),
        //                 // End 1 meters straight ahead of where we started, facing forward
        //                 new Pose2d(0, firstYTranslation, new Rotation2d(Math.toRadians(90))),
        //                 config);

        PIDController xController = new PIDController(Constants.AutoConstants.kPXController, 0, 0);
        PIDController yController = new PIDController(Constants.AutoConstants.kPYController, 0, 0);
        PIDController thetaController =
                new PIDController(Constants.AutoConstants.kPThetaController, 0, 0);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        PPSwerveControllerCommand swerveControllerCommand1 =
                new PPSwerveControllerCommand(
                        trajectory1,
                        s_Swerve::getPose,
                        Constants.DriveConstants.kDriveKinematics,
                        xController,
                        yController,
                        thetaController,
                        s_Swerve::setModuleStates,
                        s_Swerve);

        PPSwerveControllerCommand swerveControllerCommand2 =
                new PPSwerveControllerCommand(
                        trajectory1,
                        s_Swerve::getPose,
                        Constants.DriveConstants.kDriveKinematics,
                        xController,
                        yController,
                        thetaController,
                        s_Swerve::setModuleStates,
                        s_Swerve);

        addCommands(
                new InstantCommand(() -> s_Swerve.resetOdometry(trajectory1.getInitialHolonomicPose())),
                swerveControllerCommand1);
        // new InstantCommand(() -> s_Swerve.resetOdometry(trajectory2.getInitialHolonomicPose())),
        // swerveControllerCommand2);
    }
}

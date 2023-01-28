package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

// Possibly for a holonomic implementation + events? idk
// import java.util.HashMap;
// import com.pathplanner.lib.commands.FollowPathWithEvents;
// import com.pathplanner.lib.controllers.PPHolonomicDriveController;

public class PathPlannerAuto extends SequentialCommandGroup {
    String trajectoryJSON = "pathplanner/generatedJSON/New_Path.wpilib.json";
    PathPlannerTrajectory trajectory = new PathPlannerTrajectory();


    public PathPlannerAuto(Swerve s_Swerve){
        String trajectoryPath = Filesystem.getDeployDirectory().toPath().toString();
        PathPlannerTrajectory trajectory = PathPlanner.loadPath(trajectoryPath, new PathConstraints(6, 4));

        var thetaController =
            new PIDController(
                Constants.AutoConstants.kPThetaController, 0, 0);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        PPSwerveControllerCommand swerveControllerCommand =
            new PPSwerveControllerCommand(
                trajectory,
                s_Swerve::getPose,
                Constants.DriveConstants.kDriveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);


        addCommands(
            new InstantCommand(() -> s_Swerve.resetOdometry(trajectory.getInitialPose())),
            swerveControllerCommand
        );
    }
}

package frc.robot.autos;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
// import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Positions.StowPosition;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.Swerve;
import java.util.HashMap;

public class PathPlannerAuto extends SequentialCommandGroup {
  String trajectoryJSON = "Test1";
  PathPlannerTrajectory trajectory = new PathPlannerTrajectory();
  PIDController thetaController;

  public PathPlannerAuto(
    Swerve s_Swerve,
    ElevatorSubsystem s_elevatorSubsystem,
    ArmSubsystem s_arArmSubsystem,
    ClawSubsystem s_ClawSubsystem) {
// String trajectoryPath = Filesystem.getDeployDirectory().toPath().toString();
    PathPlannerTrajectory trajectory =
        PathPlanner.loadPath(trajectoryJSON, new PathConstraints(6, 4));

    thetaController = new PIDController(Constants.AutoConstants.kPThetaController, 0, 0);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    HashMap<String, Command> eventMap = new HashMap<>();
    eventMap.put("event", new StowPosition(s_elevatorSubsystem, s_arArmSubsystem, s_ClawSubsystem));
    // eventMap.put("intakeDown", new IntakeDown()); - example on the library

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

    FollowPathWithEvents command =
        new FollowPathWithEvents(swerveControllerCommand, trajectory.getMarkers(), eventMap);

    addCommands(
        new InstantCommand(() -> s_Swerve.resetOdometry(trajectory.getInitialPose())), command);
  }
}

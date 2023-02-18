package frc.robot.autos;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
// import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.MechanicalParts.ArmSubsystem;
import frc.robot.subsystems.MechanicalParts.ClawSubsystem;
import frc.robot.subsystems.MechanicalParts.ElevatorSubsystem;
import frc.robot.subsystems.Swerve.Swerve;
import java.util.HashMap;

public class PathPlannerAuto extends SequentialCommandGroup {
    PathPlannerTrajectory trajectory = new PathPlannerTrajectory();
    PIDController thetaController;

    /**
     * Sequential command group that (for now only) run the {@code Test1} path using the PathPlanner
     * library. TODO: PathPlanner Auto | Make it so you can pass the HashMap of events and the
     * trajectoryJSON name.
     *
     * @param Swerve Swerve subsystem.
     * @param ElevatorSubsystem Elevator subsystem.
     * @param ArmSubsystem Arm subsystem.
     * @param ClawSubsystem Claw subsystem.
     */
    public PathPlannerAuto(
            PathPlannerTrajectory trajectory,
            Swerve s_Swerve,
            HashMap<String, Command> eventMap) {

        thetaController = new PIDController(Constants.AutoConstants.kPThetaController, 0, 0);
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

        FollowPathWithEvents command =
                new FollowPathWithEvents(swerveControllerCommand, trajectory.getMarkers(), eventMap);

        addCommands(
                new InstantCommand(() -> s_Swerve.resetOdometry(trajectory.getInitialPose())), command);
    }
    
    public PathPlannerAuto(
            PathPlannerTrajectory trajectory,
            Swerve s_Swerve) {

        thetaController = new PIDController(Constants.AutoConstants.kPThetaController, 0, 0);
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
                new InstantCommand(() -> s_Swerve.resetOdometry(trajectory.getInitialPose())), swerveControllerCommand);
    }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.Autonomous.*;
import frc.robot.commands.Autonomous.Balance.*;
import frc.robot.commands.Autonomous.ShootPiece.*;
import frc.robot.commands.DriveParts.*;
import frc.robot.commands.Positions.Intake.*;
import frc.robot.commands.Positions.Nodes.HighNodePosition.HighNodePosition;
import frc.robot.commands.Positions.Nodes.MidNodePosition;
import frc.robot.commands.Positions.StowPosition;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.MechanicalParts.*;
import frc.robot.subsystems.Swerve.Swerve;
import frc.util.AutonManager;
import frc.util.Controller;
import java.util.HashMap;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    private final AutonManager autonManager;

    private final Swerve s_Swerve = new Swerve();
    private LedSubsystem s_Led = new LedSubsystem(0, 72);

    private final LimelightSubsystem Limelight = new LimelightSubsystem();

    private final ElevatorSubsystem s_Elevator = new ElevatorSubsystem();
    private final ArmElevatorSubsystem s_ArmElevator = new ArmElevatorSubsystem();
    private final ClawSubsystem s_Claw = new ClawSubsystem();
    private final IntakeSubsystem s_Intake = new IntakeSubsystem(s_Led);

    private static boolean isCone = Constants.DefaultConfig.isCone;
    private static boolean resetPosMode = false;
    private static double angleOffset = 0;

    public enum State {
        Stow,

        IntakeFromGround,
        IntakeFromSlidingHumanPlayer,

        Shooting,
        HighNode,
        MidNode,

        Balance
    }

    private static State currentState = State.Stow;

    private final Controller driverController = new Controller(0);
    private final Controller operatorController = new Controller(1);

    private final HashMap<String, Command> eventMap = new HashMap<>();

    private final String[] autonChoices =
            new String[] {
                "GyroTest",
                "LeftPlacePickup",
                "LeftPlacePickupPlace",
                "LeftPlacePickupShootBalance",
                "MiddlePlace",
                "MiddlePlaceBalance",
                "MiddlePlaceExitBalance",
                "RightPlacePickup",
                "RightPlacePickupPlace",
                "RightPlacePickupShootBalance",
            };

    public final SwerveAutoBuilder autoBuilder =
            new SwerveAutoBuilder(
                    s_Swerve::getPose,
                    s_Swerve::resetOdometry,
                    Constants.Drivetrain.kDriveKinematics,
                    new PIDConstants(Constants.Auton.kPXYController, 0.0, 0.0),
                    new PIDConstants(Constants.Auton.kPThetaController, 0.0, 0.0),
                    s_Swerve::setModuleStates,
                    eventMap,
                    true,
                    s_Swerve);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        autonManager = new AutonManager(Limelight);
        eventAssignment();
        addAutonomousChoices();
        autonManager.displayChoices();
        configureButtonBindings();
    }

    /**
     * Use this method to define the command or command groups to be run at each event marker key. New
     * event markers can be created in PathPlanner.
     */
    private void eventAssignment() {
        eventMap.put(
                "PreparePickup", new PreparePickup(false, s_Claw, s_ArmElevator, s_Elevator, s_Intake));
        eventMap.put("Pickup", new Pickup(false, s_Claw, s_ArmElevator, s_Elevator, s_Intake));
        eventMap.put(
                "PlaceCubeMid",
                Place.Cube(0, s_Swerve, Limelight, s_Claw, s_ArmElevator, s_Elevator, s_Intake));
        eventMap.put(
                "PlaceConeMid",
                Place.Cone(0, s_Swerve, Limelight, s_Claw, s_ArmElevator, s_Elevator, s_Intake));
        eventMap.put(
                "PlaceCubeHigh",
                Place.Cube(1, s_Swerve, Limelight, s_Claw, s_ArmElevator, s_Elevator, s_Intake));
        eventMap.put("PlaceLow", new AutonIntakeNoCurrentLimit(s_Intake).withTimeout(0.3));
        eventMap.put("Balance", new Balance(s_Swerve, s_Led));
        eventMap.put(
                "BalanceWithShoot",
                new BalanceWithShoot(s_Swerve, s_Led, s_Claw, s_ArmElevator, s_Elevator, s_Intake));
    }

    // The RightPlacePickupPlaceBalance is : 1 foot from DriverStation blue line (x: 2.16), 6 inches
    // from Right wall (y: 0.76).
    /** Use this method to add Autonomous paths, displayed with {@link AutonManager} */
    private void addAutonomousChoices() {
        autonManager.addDefaultOption("Do Nothing.", new InstantCommand());

        for (String auton : autonChoices) {
            var trajectories = PathPlanner.loadPathGroup(auton, Constants.Auton.kPathConstr);
            Command autoPath = autoBuilder.fullAuto(trajectories);
            autonManager.addOption(auton, autoPath);
        }
    }

    public static boolean getIsCone() {
        return isCone;
    }

    public static void setIsCone(boolean value) {
        isCone = value;
    }

    public static boolean resetPosMode() {
        return resetPosMode;
    }

    public static void enableResetPos() {
        resetPosMode = true;
    }

    public static void disableResetPos() {
        resetPosMode = false;
    }

    public static double getAngleOffset() {
        return angleOffset;
    }

    public static void setAngleOffset(double offset) {
        angleOffset = offset;
    }

    public static State getCurrentState() {
        return currentState;
    }

    public static void setCurrentState(State state) {
        currentState = state;
    }

    public void autonResetGyro() {
        s_Swerve.zeroGyro(180);
    }

    private void configureButtonBindings() {
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.

        final int translationAxis = XboxController.Axis.kLeftY.value;
        final int strafeAxis = XboxController.Axis.kLeftX.value;
        final int rotationAxis = XboxController.Axis.kRightX.value;

        s_Swerve.setDefaultCommand(
                new TeleopSwerve(s_Swerve, driverController, translationAxis, strafeAxis, rotationAxis));

        driverController.buttonA.onTrue(new InstantCommand(() -> s_Swerve.toggleSwerveMode()));
        driverController.buttonY.onTrue(new InstantCommand(s_Swerve::zeroGyro));

        // April Tag Mode
        driverController.leftTrigger.onTrue(
                new InstantCommand(
                        () -> {
                            Limelight.setMode(1);
                            isCone = false;
                        }));

        // Reflective Tape Mode
        driverController.rightTrigger.onTrue(
                new InstantCommand(
                        () -> {
                            Limelight.setMode(3);
                            isCone = true;
                        }));

        driverController.buttonStart.onTrue(
                new InstantCommand(
                        () -> {
                            s_Led.setAnimation("VIVELAFRANCE", true);
                        }));
        driverController.buttonBack.onTrue(
                new InstantCommand(
                        () -> {
                            s_Led.setAnimation("VIVELAFRANCE", false);
                        }));

        s_Elevator.setDefaultCommand(new DriveElevator(operatorController, s_Elevator));
        s_ArmElevator.setDefaultCommand(new DriveArmElevator(operatorController, s_ArmElevator));
        s_Claw.setDefaultCommand(new DriveClaw(operatorController, s_Claw));
        s_Intake.setDefaultCommand(new TeleopIntake(operatorController, s_Intake));

        operatorController.buttonA.onTrue(new StowPosition(s_Elevator, s_ArmElevator, s_Claw));
        driverController.rightBumper.onTrue(new StowPosition(s_Elevator, s_ArmElevator, s_Claw));
        operatorController.buttonB.onTrue(new MidNodePosition(s_Elevator, s_ArmElevator, s_Claw));
        operatorController.buttonY.onTrue(new HighNodePosition(s_Elevator, s_ArmElevator, s_Claw));
        operatorController.buttonX.onTrue(new Position(s_ArmElevator, s_Elevator, s_Claw));

        operatorController.buttonBack.onTrue(
                new InstantCommand(
                        () -> {
                            if (resetPosMode()) {
                                s_Elevator.zeroHeightEncoder();
                                s_ArmElevator.resetEncoderValue();
                                s_Claw.resetEncoderValue();
                            }
                        }));

        operatorController.buttonStart.onTrue(new InstantCommand(RobotContainer::enableResetPos));
        operatorController.buttonStart.onFalse(new InstantCommand(RobotContainer::disableResetPos));

        operatorController.dPadDown.onTrue(
                new IntakeFromGroundPosition(s_Elevator, s_ArmElevator, s_Claw));
        operatorController.dPadUp.onTrue(
                new IntakeFromHumanPlayerPosition(s_Elevator, s_ArmElevator, s_Claw));
        operatorController.dPadRight.onTrue(
                new IntakeFromSlidingHumanPlayerPosition(s_Elevator, s_ArmElevator, s_Claw));
        operatorController.dPadLeft.onTrue(new ShootPiece(s_Intake, s_Elevator, s_ArmElevator, s_Claw));
    }

    public Command getAutonomousCommand() {
        isCone = false;
        return autonManager.getSelected();
    }

    public void idleLimelight() {
        Limelight.setPipeline(7);
        Limelight.setMode(0); // pipeline default
    }
}

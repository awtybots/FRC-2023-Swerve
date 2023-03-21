// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class DefaultConfig {
        public static final boolean isCone = true;

        public static final double rampRate = 2;
        public static final double stickDeadband = 0.1;

        public static final double lowSpeedMultiplier = 0.2;

        public static final boolean fieldRelative = true;

        public static final int LEDPort = 0;

        public static final boolean VisionTrackingStrafe = true;
    }

    public static final class Balance { // TODO move to balance command
        public static final double BEAM_BALANCED_GOAL_DEGREES = 0;
        public static final double BEAM_BALANACED_DRIVE_KP = 1;
        public static final double BACKWARDS_BALANCING_EXTRA_POWER_MULTIPLIER = 3;
        public static final double BEAM_BALANCED_ANGLE_TRESHOLD_DEGREES = 1;
    }

    public static final class Presets {

        public static final int ElevatorThreshold = 5000;
        public static final int ArmThreshold = (int) (10 * Constants.Arm.armConversion);
        public static final int ClawThreshold = 10;

        public static final class Stow {
            public static final double ElevatorPosition = 1.19377;
            public static final int ArmPosition = 0;
            public static final int ClawPosition = 0;
        }

        public static final class Nodes {
            public static final class Cube {
                public static final class MidNode {
                    public static final double ElevatorPosition = 1.1938;
                    public static final double ArmPosition = 18.691;
                    public static final double ClawPosition = 28.9285;
                    // public static final int ElevatorPosition = 23700;
                    // public static final double ArmPosition = 16.07 * Constants.Arm.armConversion;
                    // public static final double ClawPosition = 11.714 * Constants.Claw.clawConversion;
                }

                public static final class HighNode {
                    public static final double TransitionArmRotation = 46.667;
                    public static final double ElevatorPosition = 14.9222;
                    public static final double ArmPosition = 78.8;
                    public static final double ClawPosition = 44.28;
                }
            }

            public static final class Cone {
                public static final class MidNode {
                    public static final double ElevatorPosition = 38.1040;
                    public static final double ArmPosition = 45.844;
                    public static final double ClawPosition = -33.4285;
                    // public static final int ElevatorPosition = 126270;
                    // public static final double ArmPosition = 23.69;
                    // public static final double ClawPosition = -6.214;
                }

                public static final class HighNode {
                    public static final double TransitionArmRotation = 46.6666666;
                    public static final double ElevatorPosition = 57.89822997;
                    public static final double ArmPosition = 77.966666666;
                    public static final double ClawPosition = -10.89;
                    // public static final int TransitionArmRotation = 28;
                    // public static final int ElevatorPosition = 194000;
                    // public static final double ArmPosition = 46.78;
                    // public static final double ClawPosition = -1.21;
                }
            }
        }

        public static final class Intake {
            public static final class IntakeFromGroundLow {
                public static final double ElevatorPosition = 1.1928;
                public static final double ArmPosition = 117.62; // 77.86
                public static final double ClawPosition = -9.00;
            }

            public static final class Cube {
                public static final class IntakeFromGround {
                    public static final double ElevatorPosition = 23.8756; // 1.206 inch on dash
                    public static final double ArmPosition = 166.79; // 25.266 degrees on dash
                    public static final double ClawPosition = -45.99; // 38.286 degrees on dash
                    // public static final int ElevatorPosition = 80000;
                    // public static final double ArmPosition = 100.074;
                    // public static final double ClawPosition = -5.11;
                }

                public static final class IntakeFromSlidingHumanPlayer {
                    public static final double ElevatorPosition = 2.8033;
                    public static final double ArmPosition = 29.3167;
                    public static final double ClawPosition = 160.38;
                    // public static final double ClawPosition = 13.2857; // TODO THIS
                }
            }

            public static final class Cone {
                public static final class IntakeFromGround {
                    public static final double ElevatorPosition = 34.321;
                    public static final double ArmPosition = 167.300;
                    public static final double ClawPosition = -49.050;
                }

                public static final class IntakeFromHumanPlayer {
                    public static final double ElevatorPosition = 25.875;
                    public static final double ArmPosition = 71.609;
                    public static final double ClawPosition = 14.76;
                }

                public static final class IntakeFromSlidingHumanPlayer {
                    public static final double ElevatorPosition = 2.2413;
                    public static final double ArmPosition = 28.0167;
                    public static final double ClawPosition = 17.82;
                }
            }
        }
    }

    public static final class Elevator {
        public static final int kLeftElevatorMotorId = 20; // !
        public static final int kRightElevatorMotorId = 21; // !
        public static final int kElevatorEncoderId = 22; // !

        public static final double kMaxPercentOutput = 0.75;
        public static final double kRamp =
                0.08; // seconds from zero to full throttle on elevator motors
        public static final double kWinchDiameter =
                1.751; // (inches) pitch diameter of #25 chain 22T sprocket
        public static final double kGearRatio =
                1.0 / 9; // output over input (9:1 reduction please double check this)

        // Heights
        public static final int initialHeight = 4000;
        public static final int minimumHeight = 4000;
        public static final int maximumHeight = 195000;
        public static final double ElevatorOffset = 0.05;

        // PID
        public static final double kP = 0.25; // = 0.025;
        public static final double kI = 0; // = 0.00;
        public static final double kD = 0.1; // 0.015;
        public static final double kF = 0;

        public static final double arbitraryFeedforwardRate = 0.02; // 0.0425;
    }

    public static final class Arm {
        public static final double armConversion = 216.0 / 184.0;

        public static final int kLeftArmMotorId = 7;
        public static final int kRightArmMotorId = 12;

        public static final int kCurrentLimit = 40;

        // Heights
        public static final int initialHeight = 0;
        public static final int minimumHeight = 0;
        public static final int maximumHeight = (int) (70 * armConversion);

        // PID
        public static final double kP = 0.04; // 0.002
        public static final double kI = 0; // 0.0001
        public static final double kD = 0.000;
        public static final double kF = 0.00;

        public static final double armLength = 0.54; // 0.54 m
        public static final double startingAngle = 144;

        public static final double rampRate = 0.6;
        public static final double arbitraryFeedFowardRate = 0.1;
    }

    public static final class Claw {
        public static final double clawConversion = 40.0 / 91.67;

        public static final int kPivotMotorId = 6;
        public static final int kIntakeMotorId = 15; // !

        public static final double kMaxPercentOutput = 0.75;

        public static final int kClawCurrentLimit = 40;
        public static final int kIntakeCurrentLimit = 20;

        public static final int initialHeight = 0;
        public static final int minimumHeight = (int) (-15 * Constants.Claw.clawConversion);
        public static final int maximumHeight = (int) (35 * Constants.Claw.clawConversion);

        public static final double kP = 0.10;
        public static final double kI = 0.00; // !
        public static final double kD = 0.00;

        public static final double startingAngle =
                64; // ! ONLY FROM TOP OF WRIST NOT INDICATIVE OF MASS

        public static final double arbitraryFeedFowardRate = 0.0;
    }

    public static final class Piston {
        public static final int kTraverseSolenoidF = 0;
        public static final int kTraverseSolenoidR = 1;
    }

    public static final class LimeLight {
        public static final double AprilTagHeight = 0.50;
        public static final double LimelightHeight = 0.46;
        public static final double LimelightAngle = 14;
    }

    public static final class Drivetrain {
        // Driving Parameters - Note that these are not the maximum capable speeds of
        // the robot, rather the allowed maximum speeds
        public static final double kMaxSpeedMetersPerSecond = 4.8;
        public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

        // Chassis configuration
        public static final double kTrackWidth = Units.inchesToMeters(26.5);
        // Distance between centers of right and left wheels on robot
        public static final double kWheelBase = Units.inchesToMeters(26.5);
        // Distance between front and back wheels on robot
        public static final SwerveDriveKinematics kDriveKinematics =
                new SwerveDriveKinematics(
                        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

        // Angular offsets of the modules relative to the chassis in radians
        // public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
        // public static final double kFrontRightChassisAngularOffset = 0;
        // public static final double kBackLeftChassisAngularOffset = Math.PI;
        // public static final double kBackRightChassisAngularOffset = Math.PI / 2;

        public static final double kFrontLeftChassisAngularOffset = Math.PI;
        public static final double kFrontRightChassisAngularOffset = 0;
        public static final double kBackLeftChassisAngularOffset = Math.PI;
        public static final double kBackRightChassisAngularOffset = 0;

        // SPARK MAX CAN IDs
        public static final int kFrontLeftDrivingCanId = 10;
        public static final int kRearLeftDrivingCanId = 8;
        public static final int kFrontRightDrivingCanId = 2;
        public static final int kRearRightDrivingCanId = 4;

        public static final int kFrontLeftTurningCanId = 11;
        public static final int kRearLeftTurningCanId = 9;
        public static final int kFrontRightTurningCanId = 3;
        public static final int kRearRightTurningCanId = 5;

        public static final boolean kGyroReversed = true;

        public static final double stallCurrentLimit =
                40; // If the current for any of the motors exceeds this limit, isSTall returns true
        // !! TODO check if the motor has been still for a set time as well. Currently it will think its
        // stalling when it starts accelerating.
    }

    public static final class SwerveModule {
        // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
        // This changes the drive speed of the module (a pinion gear with more teeth will result in a
        // robot that drives faster).
        public static final int kDrivingMotorPinionTeeth = 14;

        // Invert the turning encoder, since the output shaft rotates in the opposite direction of
        // the steering motor in the MAXSwerve Module.
        public static final boolean kTurningEncoderInverted = true;

        // Calculations required for driving motor conversion factors and feed forward
        public static final double kNeoMotorFreeRpm = 5676;
        public static final double kDrivingMotorFreeSpeedRps = kNeoMotorFreeRpm / 60;
        public static final double kWheelDiameterMeters = 0.0762;
        public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
        // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the
        // bevel pinion
        public static final double kDrivingMotorReduction =
                (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
        public static final double kDriveWheelFreeSpeedRps =
                (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters) / kDrivingMotorReduction;

        public static final double kDrivingEncoderPositionFactor =
                (kWheelDiameterMeters * Math.PI) / kDrivingMotorReduction; // meters
        public static final double kDrivingEncoderVelocityFactor =
                ((kWheelDiameterMeters * Math.PI) / kDrivingMotorReduction) / 60.0; // meters per second

        public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
        public static final double kTurningEncoderVelocityFactor =
                (2 * Math.PI) / 60.0; // radians per second

        public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
        public static final double kTurningEncoderPositionPIDMaxInput =
                kTurningEncoderPositionFactor; // radians

        public static final double kDrivingP = 0.04;
        public static final double kDrivingI = 0;
        public static final double kDrivingD = 0;
        public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
        public static final double kDrivingMinOutput = -1;
        public static final double kDrivingMaxOutput = 1;

        public static final double kTurningP = 0.4;
        public static final double kTurningI = 0;
        public static final double kTurningD = 0;
        public static final double kTurningFF = 0;
        public static final double kTurningMinOutput = -1;
        public static final double kTurningMaxOutput = 1;

        public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
        public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

        public static final int kDrivingMotorCurrentLimit = 50; // amps
        public static final int kTurningMotorCurrentLimit = 20; // amps
    }

    public static final class Auton {
        public static final double kMaxSpeedMetersPerSecond = 4.8; // 4
        public static final double kMaxAccelerationMetersPerSecondSquared = 3; // 2

        public static final double kPXYController = 1; // TODO: tune PID for autos
        public static final double kPThetaController = 1;
    }

    public static final class Intake { // TODO move into intake subsytem
        // Time in milliseconds that it should take to eject a cube / cone
        public static final long IntakeEjectionTime = 500;
        // Time in milliseconds that it should take to intake a cube / cone
        public static final long IntakeTime = 500;
    }
}

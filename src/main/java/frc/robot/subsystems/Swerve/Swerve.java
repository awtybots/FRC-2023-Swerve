// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Swerve;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class Swerve extends SubsystemBase {
    // Create MAXSwerveModules
    private final MAXSwerveModule m_frontLeft =
            new MAXSwerveModule(
                    DriveConstants.kFrontLeftDrivingCanId,
                    DriveConstants.kFrontLeftTurningCanId,
                    DriveConstants.kFrontLeftChassisAngularOffset);

    private final MAXSwerveModule m_frontRight =
            new MAXSwerveModule(
                    DriveConstants.kFrontRightDrivingCanId,
                    DriveConstants.kFrontRightTurningCanId,
                    DriveConstants.kFrontRightChassisAngularOffset);

    private final MAXSwerveModule m_rearLeft =
            new MAXSwerveModule(
                    DriveConstants.kRearLeftDrivingCanId,
                    DriveConstants.kRearLeftTurningCanId,
                    DriveConstants.kBackLeftChassisAngularOffset);

    private final MAXSwerveModule m_rearRight =
            new MAXSwerveModule(
                    DriveConstants.kRearRightDrivingCanId,
                    DriveConstants.kRearRightTurningCanId,
                    DriveConstants.kBackRightChassisAngularOffset);

    // The gyro sensor
    private final AHRS m_gyro;
    private final SwerveDriveOdometry m_odometry;

    public Boolean swerveHighSpeedMode;

    /** Creates a new DriveSubsystem. */
    public Swerve() {
        swerveHighSpeedMode = true;

        m_gyro = new AHRS(SPI.Port.kMXP);

        zeroGyro();

        // Odometry class for tracking robot pose
        m_odometry =
                new SwerveDriveOdometry(
                        DriveConstants.kDriveKinematics,
                        Rotation2d.fromDegrees(m_gyro.getAngle()),
                        new SwerveModulePosition[] {
                            m_frontLeft.getPosition(),
                            m_frontRight.getPosition(),
                            m_rearLeft.getPosition(),
                            m_rearRight.getPosition()
                        });
    }

    @Override
    public void periodic() {
        // Update the odometry in the periodic block

        m_odometry.update(
                Rotation2d.fromDegrees(-m_gyro.getAngle()),
                new SwerveModulePosition[] {
                    m_frontLeft.getPosition(),
                    m_frontRight.getPosition(),
                    m_rearLeft.getPosition(),
                    m_rearRight.getPosition()
                });
    }

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    /**
     * Returns the currently-estimated yaw of the robot.
     *
     * @return The yaw.
     */
    public double getYaw() {
        return m_gyro.getYaw();
    }

    /**
     * Returns the currently-estimated roll of the robot.
     *
     * @return The roll.
     */
    public double getRoll() {
        return m_gyro.getRoll();
    }

    /**
     * Returns the currently-estimated pitch of the robot.
     *
     * @return The pitch.
     */
    public double getPitch() {
        return m_gyro.getPitch();
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
        m_odometry.resetPosition(
                Rotation2d.fromDegrees(-m_gyro.getAngle()),
                new SwerveModulePosition[] {
                    m_frontLeft.getPosition(),
                    m_frontRight.getPosition(),
                    m_rearLeft.getPosition(),
                    m_rearRight.getPosition()
                },
                pose);
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed Speed of the robot in the x direction (forward).
     * @param ySpeed Speed of the robot in the y direction (sideways).
     * @param rot Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the field.
     */
    public void drive(Translation2d translation, double rot, boolean fieldRelative) {

        double XVelocity = translation.getX();
        double YVelocity = translation.getY();

        var swerveModuleStates =
                DriveConstants.kDriveKinematics.toSwerveModuleStates(
                        fieldRelative
                                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                        XVelocity, YVelocity, rot, Rotation2d.fromDegrees(-m_gyro.getYaw()))
                                : new ChassisSpeeds(XVelocity, YVelocity, rot));
        SwerveDriveKinematics.desaturateWheelSpeeds(
                swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
        m_rearLeft.setDesiredState(swerveModuleStates[2]);
        m_rearRight.setDesiredState(swerveModuleStates[3]);
    }

    /** Sets the wheels into an X formation to prevent movement. */
    public void setX() {
        m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    }

    /**
     * Sets the swerve ModuleStates.
     *
     * @param desiredStates The desired SwerveModule states.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(
                desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
        m_frontLeft.setDesiredState(desiredStates[0]);
        m_frontRight.setDesiredState(desiredStates[1]);
        m_rearLeft.setDesiredState(desiredStates[2]);
        m_rearRight.setDesiredState(desiredStates[3]);
    }

    /** Resets the drive encoders to currently read a position of 0. */
    public void resetEncoders() {
        m_frontLeft.resetEncoders();
        m_rearLeft.resetEncoders();
        m_frontRight.resetEncoders();
        m_rearRight.resetEncoders();
    }

    /** Zeroes the heading of the robot. */
    public void zeroGyro() {
        m_gyro.zeroYaw();
    }

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from -180 to 180
     */
    public double getHeading() {
        return Rotation2d.fromDegrees(m_gyro.getAngle()).getDegrees();
    }

    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    public double getTurnRate() {
        return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
    }

    public void toggleSwerveMode() {
        swerveHighSpeedMode = !swerveHighSpeedMode;
    }
}

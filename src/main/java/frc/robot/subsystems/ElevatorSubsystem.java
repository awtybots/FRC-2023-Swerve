package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {


    public ElevatorSubsystem() {

    }

    // private void configAngleMotor(){
    //     mAngleMotor.configFactoryDefault();
    //     mAngleMotor.configAllSettings(Robot.ctreConfigs.swerveAngleFXConfig);
    //     mAngleMotor.setInverted(Constants.Swerve.angleMotorInvert);
    //     mAngleMotor.setNeutralMode(Constants.Swerve.angleNeutralMode);
    //     resetToAbsolute();
    // }

    // public Rotation2d getCanCoder(){
    //     return Rotation2d.fromDegrees(angleEncoder.getSelectedSensorPosition() * 360.0 / 4096.0);
    // }

    // public SwerveModuleState getState(){
    //     double velocity = Conversions.falconToMPS(mDriveMotor.getSelectedSensorVelocity(), Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio);
    //     Rotation2d angle = Rotation2d.fromDegrees(Conversions.falconToDegrees(mAngleMotor.getSelectedSensorPosition(), Constants.Swerve.angleGearRatio));
    //     return new SwerveModuleState(velocity, angle);
    // }
}
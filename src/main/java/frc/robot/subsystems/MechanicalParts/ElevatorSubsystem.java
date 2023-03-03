package frc.robot.subsystems.MechanicalParts;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {

    private final double kMaxPercentOutput;
    private final double kRamp;

    // ? Is this going to be used?

    private final WPI_TalonFX mLeftElevatorMotor;
    private final WPI_TalonFX mRightElevatorMotor;
    public final WPI_TalonFX[] motors;

    private final WPI_TalonSRX elevatorEncoder;

    public double elevatorTargetHeight = Constants.ElevatorConstants.initialHeight;

    public ElevatorSubsystem() {
        kMaxPercentOutput = Constants.ElevatorConstants.kMaxPercentOutput;
        kRamp = Constants.ElevatorConstants.kRamp;

        elevatorEncoder = new WPI_TalonSRX(Constants.ElevatorConstants.kElevatorEncoderId);
        configElevatorEncoder();

        mLeftElevatorMotor = new WPI_TalonFX(Constants.ElevatorConstants.kLeftElevatorMotorId);
        mRightElevatorMotor = new WPI_TalonFX(Constants.ElevatorConstants.kRightElevatorMotorId);
        motors = new WPI_TalonFX[] {mLeftElevatorMotor, mRightElevatorMotor};
        configMotors();
    }

    private void resetToAbsolute() {
        // double absolutePosition = TalonConversions.degreesToFalcon(getCanCoder().getDegrees(),
        // Constants.ElevatorConstants.kGearRatio);
        double absolutePosition = (getCanCoder());
        // mLeftElevatorMotor.setSelectedSensorPosition(absolutePosition);
        mRightElevatorMotor.setSelectedSensorPosition(absolutePosition);
    }

    public double getCanCoder() {
        return elevatorEncoder.getSelectedSensorPosition() * 5; // ? Why is there a <<* 5>> ?!? explain!
    }

    private void configElevatorEncoder() {
        elevatorEncoder.configFactoryDefault();
        elevatorEncoder.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);

        // elevatorEncoder.configAllSettings(Robot.ctreConfigs.swerveCanCoderConfig);
    }

    private void configMotors() {
        mLeftElevatorMotor.configFactoryDefault();
        mRightElevatorMotor.configFactoryDefault();
        mRightElevatorMotor.setInverted(TalonFXInvertType.Clockwise);

        // for (WPI_TalonFX motor : motors) {
        mRightElevatorMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        // mRightElevatorMotor.setSelectedSensorPosition(0.0);

        mLeftElevatorMotor.setNeutralMode(NeutralMode.Brake);
        mRightElevatorMotor.setNeutralMode(NeutralMode.Brake);

        mRightElevatorMotor.configOpenloopRamp(kRamp);
        mRightElevatorMotor.configClosedloopRamp(kRamp);
        mRightElevatorMotor.configPeakOutputForward(kMaxPercentOutput);
        mRightElevatorMotor.configPeakOutputReverse(-kMaxPercentOutput);
        mRightElevatorMotor.configClosedLoopPeakOutput(0, kMaxPercentOutput);
        // }

        resetToAbsolute();
        mLeftElevatorMotor.follow(mRightElevatorMotor);
    }

    public double getDistance() {
        return convertTalonToMeters(getCanCoder());
    }

    public double convertTalonToMeters(double talon) {
        return talon * 1.1 / 198000;
    }

    public double convertMetersToTalon(double meters) {
        return meters * 198000 / 1.1;
    }
    
    public boolean isFinished() {
        return Math.abs(motors[1].getSelectedSensorPosition() - elevatorTargetHeight) < Constants.Position.ElevatorThreshold;
    }

    public void setHeight(double value) {
        resetToAbsolute();
        elevatorTargetHeight = value;
        motors[1].set(ControlMode.Position, elevatorTargetHeight);
    }

    public void drive(double pct) {
        resetToAbsolute();
        elevatorTargetHeight += pct * 1000;
        elevatorTargetHeight =
                MathUtil.clamp(
                        elevatorTargetHeight,
                        Constants.ElevatorConstants.minimumHeight,
                        Constants.ElevatorConstants.maximumHeight);
    }

    public void stop() {
        // for (WPI_TalonFX motor : motors) motor.set(ControlMode.PercentOutput, 0.0);
        motors[1].set(ControlMode.PercentOutput, 0.0);
    }

    @Override
    public void periodic() {
        motors[1].set(ControlMode.Position, elevatorTargetHeight);

        SmartDashboard.putNumber(
                "Elevator Absolute Angle", elevatorEncoder.getSelectedSensorPosition());
        SmartDashboard.putNumber("Elivator position1 ", motors[0].getSelectedSensorPosition());
        SmartDashboard.putNumber("Elivator position2 ", motors[1].getSelectedSensorPosition());

        if (elevatorTargetHeight - motors[1].getSelectedSensorPosition() < 1000
                && motors[1].getSelectedSensorPosition() < 6000)
            motors[1].set(ControlMode.PercentOutput, 0);

        // 3868 -> 5828  =  1960
        // 15067 -> 24873   =  9806

        // for (WPI_TalonFX motor : motors){
        mRightElevatorMotor.config_kP(0, Constants.ElevatorConstants.kP);
        mRightElevatorMotor.config_kI(0, Constants.ElevatorConstants.kI);
        mRightElevatorMotor.config_kD(0, Constants.ElevatorConstants.kD);
        mRightElevatorMotor.config_kF(0, Constants.ElevatorConstants.kF);
        // }

    }
}

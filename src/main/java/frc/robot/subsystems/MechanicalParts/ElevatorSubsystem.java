package frc.robot.subsystems.MechanicalParts;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.util.math.Convert;

public class ElevatorSubsystem extends SubsystemBase {

    private final double kMaxPercentOutput;
    private final double kRamp;

    // ? Is this going to be used?

    private final WPI_TalonFX mLeftElevatorMotor;
    private final WPI_TalonFX mRightElevatorMotor;
    public final WPI_TalonFX[] motors;

    public double elevatorTargetHeight = Constants.ElevatorConstants.initialHeight;

    public ElevatorSubsystem() {
        kMaxPercentOutput = Constants.ElevatorConstants.kMaxPercentOutput;
        kRamp = Constants.ElevatorConstants.kRamp;

        mLeftElevatorMotor = new WPI_TalonFX(Constants.ElevatorConstants.kLeftElevatorMotorId);
        mRightElevatorMotor = new WPI_TalonFX(Constants.ElevatorConstants.kRightElevatorMotorId);
        motors = new WPI_TalonFX[] {mLeftElevatorMotor, mRightElevatorMotor};
        // motors = new WPI_TalonFX[] {mRightElevatorMotor, mLeftElevatorMotor};
        configMotors();
    }

    private void configMotors() {
        mLeftElevatorMotor.configFactoryDefault();
        mRightElevatorMotor.configFactoryDefault();
        mRightElevatorMotor.setInverted(TalonFXInvertType.Clockwise);

        // for (WPI_TalonFX motor : motors) {
        mRightElevatorMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        mRightElevatorMotor.setSelectedSensorPosition(4000);
        // mLeftElevatorMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        // mLeftElevatorMotor.setSelectedSensorPosition(4000);

        mLeftElevatorMotor.setNeutralMode(NeutralMode.Brake);
        mRightElevatorMotor.setNeutralMode(NeutralMode.Brake);

        mLeftElevatorMotor.configNeutralDeadband(0.0);
        mRightElevatorMotor.configNeutralDeadband(0);

        mRightElevatorMotor.configOpenloopRamp(kRamp);
        mRightElevatorMotor.configClosedloopRamp(kRamp);
        mRightElevatorMotor.configPeakOutputForward(kMaxPercentOutput);
        mRightElevatorMotor.configPeakOutputReverse(-kMaxPercentOutput);
        mRightElevatorMotor.configClosedLoopPeakOutput(0, kMaxPercentOutput);

        mLeftElevatorMotor.follow(mRightElevatorMotor);

        mRightElevatorMotor.config_kP(0, Constants.ElevatorConstants.kP);
        mRightElevatorMotor.config_kI(0, Constants.ElevatorConstants.kI);
        mRightElevatorMotor.config_kD(0, Constants.ElevatorConstants.kD);
        mRightElevatorMotor.config_kF(0, Constants.ElevatorConstants.kF);
    }
    // mLeftElevatorMotor.configOpenloopRamp(kRamp); // !
    // mLeftElevatorMotor.configClosedloopRamp(kRamp); // !
    // mLeftElevatorMotor.configPeakOutputForward(kMaxPercentOutput); // !
    // mLeftElevatorMotor.configPeakOutputReverse(-kMaxPercentOutput); // !
    // mLeftElevatorMotor.configClosedLoopPeakOutput(0, kMaxPercentOutput); // !

    // mRightElevatorMotor.follow(mLeftElevatorMotor); // !

    // public double getDistance() {
    //     return convertTalonToMeters(getCanCoder());
    // }

    public double convertTalonToInches(double talon) {
        return Convert.encoderPosToDistance(
                talon,
                Constants.ElevatorConstants.kGearRatio,
                Constants.ElevatorConstants.kWinchDiameter,
                Convert.Encoder.TalonFXIntegrated);
        // return talon * 1.1 / 198000;
    }

    public double convertMetersToTalon(double meters) {
        return meters * 198000 / 1.1;
    }

    public boolean isFinished() {
        return Math.abs(motors[1].getSelectedSensorPosition() - elevatorTargetHeight)
                < Constants.Position.ElevatorThreshold;
    }

    public void setHeight(double value) {
        elevatorTargetHeight = value;
    }

    public void resetEncoderValue() {
        elevatorTargetHeight = Constants.ElevatorConstants.initialHeight;
        motors[1].setSelectedSensorPosition(elevatorTargetHeight);
    }

    public void drive(double pct) {
        elevatorTargetHeight += pct * 1000;
        // if (!RobotContainer.getResetPosMode()) {
        elevatorTargetHeight =
                MathUtil.clamp(
                        elevatorTargetHeight,
                        Constants.ElevatorConstants.minimumHeight,
                        Constants.ElevatorConstants.maximumHeight);
        // }
    }

    public void stop() {
        // for (WPI_TalonFX motor : motors) motor.set(ControlMode.PercentOutput, 0.0);
        motors[1].set(ControlMode.PercentOutput, 0.0);
    }

    private double positionError() {
        return motors[1].getSelectedSensorPosition() - elevatorTargetHeight;
    }

    private double getHeightInches() {
        return convertTalonToInches(motors[1].getSelectedSensorPosition());
    }

    private double getErrorInches() {
        return convertTalonToInches(this.positionError());
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber(
                "Elevator Left", convertTalonToInches(mLeftElevatorMotor.getSelectedSensorPosition()));
        SmartDashboard.putNumber(
                "Elevator Right", convertTalonToInches(mRightElevatorMotor.getSelectedSensorPosition()));

        SmartDashboard.putNumber("Elevator Target Height", convertTalonToInches(elevatorTargetHeight));
        SmartDashboard.putNumber("Elevator Error", this.getErrorInches());

        motors[1].set(
                ControlMode.Position,
                elevatorTargetHeight,
                DemandType.ArbitraryFeedForward,
                Constants.ElevatorConstants.arbitraryFeedforwardRate);
        // motors[1].set(ControlMode.Position, elevatorTargetHeight);
        // Constants.ElevatorConstants.arbitraryFeedforwardRate);

        // if ((elevatorTargetHeight - motors[1].getSelectedSensorPosition()) < 3000
        //         && motors[1].getSelectedSensorPosition() < 6000
        //         && !RobotContainer.getResetPosMode()) motors[1].set(ControlMode.PercentOutput, 0);

        // 3868 -> 5828  =  1960
        // 15067 -> 24873   =  9806

        // for (WPI_TalonFX motor : motors){

        // mLeftElevatorMotor.config_kP(0, Constants.ElevatorConstants.kP);
        // mLeftElevatorMotor.config_kI(0, Constants.ElevatorConstants.kI);
        // mLeftElevatorMotor.config_kD(0, Constants.ElevatorConstants.kD);
        // mLeftElevatorMotor.config_kF(0, Constants.ElevatorConstants.kF);
    }
    // }

    public boolean atTargetHeight() {
        return Math.abs(this.positionError()) < Constants.Position.ElevatorThreshold;
    }
}

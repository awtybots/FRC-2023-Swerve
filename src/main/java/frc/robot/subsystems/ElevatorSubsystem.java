package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.robot.Constants;
import frc.util.math.Convert;
import frc.util.math.Convert.Encoder;

public class ElevatorSubsystem extends SubsystemBase {

    private final double kMaxPercentOutput;
    private final double kRamp;
    private final double kWinchDiameter;
    private final double kGearRatio;

    private WPI_TalonFX mLeftElevatorMotor;
    private WPI_TalonFX mRightElevatorMotor;
    private final WPI_TalonFX[] motors;

    public ElevatorSubsystem() {
        kMaxPercentOutput = Constants.ElevatorConstants.kMaxPercentOutput;
        kRamp = Constants.ElevatorConstants.kRamp;
        kWinchDiameter = Constants.ElevatorConstants.kWinchDiameter;
        kGearRatio = Constants.ElevatorConstants.kGearRatio;

        mLeftElevatorMotor = new WPI_TalonFX(Constants.ElevatorConstants.kLeftElevatorMotorId);
        mRightElevatorMotor = new WPI_TalonFX(Constants.ElevatorConstants.kRightElevatorMotorId);
        motors = new WPI_TalonFX[] {mLeftElevatorMotor, mRightElevatorMotor};
        configMotors();
    }

    private void configMotors() {
        mLeftElevatorMotor.configFactoryDefault();
        mRightElevatorMotor.configFactoryDefault();

        mLeftElevatorMotor.setInverted(TalonFXInvertType.Clockwise);

        for (WPI_TalonFX motor : motors) {
            motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
            motor.setSelectedSensorPosition(0.0);

            motor.setNeutralMode(NeutralMode.Brake);

            motor.configOpenloopRamp(kRamp);
            motor.configClosedloopRamp(kRamp);
            motor.configPeakOutputForward(kMaxPercentOutput);
            motor.configPeakOutputReverse(-kMaxPercentOutput);
            motor.configClosedLoopPeakOutput(0, kMaxPercentOutput);
        }
    }

    public void drive(double pct) {
        for (WPI_TalonFX motor : motors)
            motor.set(ControlMode.PercentOutput, pct * kMaxPercentOutput);
    }

    public void stop() {
        for (WPI_TalonFX motor : motors) motor.set(ControlMode.PercentOutput, 0.0);
    }

    private double getPosition() {
        double sum = 0.0;
        for (WPI_TalonFX motor : motors) {
            sum +=
                    Convert.encoderPosToDistance(
                            motor.getSelectedSensorPosition(),
                            kGearRatio,
                            kWinchDiameter,
                            Encoder.TalonFXIntegrated);
        }
        return sum / motors.length;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("position", getPosition());;
    }

}
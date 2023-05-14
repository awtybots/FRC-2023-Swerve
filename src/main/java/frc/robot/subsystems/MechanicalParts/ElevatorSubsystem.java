package frc.robot.subsystems.MechanicalParts;

import static frc.robot.Constants.Elevator;
import static frc.robot.Constants.Presets;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.util.math.Convert;
import frc.util.math.Convert.Encoder;

public class ElevatorSubsystem extends SubsystemBase implements ElevatorMech {

    private final double kMaxPercentOutput = Elevator.kMaxPercentOutput;
    private final double kRamp = Elevator.kRamp;

    private final WPI_TalonFX mLeftElevatorMotor;
    private final WPI_TalonFX mRightElevatorMotor;
    public final WPI_TalonFX[] motors;

    public double elevatorTargetHeight = Elevator.initialHeight;

    public ElevatorSubsystem() {
        mLeftElevatorMotor = new WPI_TalonFX(Elevator.kLeftMotorId);
        mRightElevatorMotor = new WPI_TalonFX(Elevator.kRightMotorId);
        motors = new WPI_TalonFX[] {mLeftElevatorMotor, mRightElevatorMotor};
        configMotors();
    }

    private void configMotors() {
        mLeftElevatorMotor.configFactoryDefault();
        mRightElevatorMotor.configFactoryDefault();
        mRightElevatorMotor.setInverted(TalonFXInvertType.Clockwise);

        mRightElevatorMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        mRightElevatorMotor.setSelectedSensorPosition(4000);

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

        mRightElevatorMotor.config_kP(0, Elevator.kP);
        mRightElevatorMotor.config_kI(0, Elevator.kI);
        mRightElevatorMotor.config_kD(0, Elevator.kD);
        mRightElevatorMotor.config_kF(0, Elevator.kF);
    }

    public double convertTalonToInches(double talon) {
        return Convert.encoderPosToDistance(
                talon, Elevator.kGearRatio, Elevator.kWinchDiameter, Convert.Encoder.TalonFXIntegrated);
    }

    public double convertMetersToTalon(double meters) {
        return meters * 198000 / 1.1;
    }

    public void setHeight(double inches) {
        double sensorUnits =
                Convert.distanceToEncoderPos(
                        inches, Elevator.kGearRatio, Elevator.kWinchDiameter, Encoder.TalonFXIntegrated);
        setHeight(sensorUnits);
    }

    public void zeroHeightEncoder() {
        elevatorTargetHeight = Elevator.initialHeight;
        motors[1].setSelectedSensorPosition(elevatorTargetHeight);
    }

    public void drive(double pct) {
        elevatorTargetHeight =
                MathUtil.clamp(elevatorTargetHeight, Elevator.minimumHeight, Elevator.maximumHeight);
        elevatorTargetHeight += pct * 1000;
    }

    public void stop() {
        motors[1].set(ControlMode.PercentOutput, 0.0);
    }

    public double distToSetpoint() {
        return motors[1].getSelectedSensorPosition() - elevatorTargetHeight;
    }

    public double heightInches() {
        return convertTalonToInches(motors[1].getSelectedSensorPosition());
    }

    private double getErrorInches() {
        return convertTalonToInches(this.distToSetpoint());
    }

    @Override
    public void periodic() {

        elevatorTargetHeight =
                MathUtil.clamp(elevatorTargetHeight, Elevator.minimumHeight, Elevator.maximumHeight);

        SmartDashboard.putNumber(
                "Elevator Right", convertTalonToInches(mRightElevatorMotor.getSelectedSensorPosition()));

        SmartDashboard.putNumber("Elevator Target Height", convertTalonToInches(elevatorTargetHeight));
        SmartDashboard.putNumber("Elevator Error", this.getErrorInches());
        SmartDashboard.putNumber("Elevator Target Height Ticks", elevatorTargetHeight);

        motors[1].set(
                ControlMode.Position,
                elevatorTargetHeight,
                DemandType.ArbitraryFeedForward,
                Elevator.arbitraryFeedforwardRate);
    }

    public boolean atTargetHeight() {
        return Math.abs(this.distToSetpoint()) < Presets.ElevatorThreshold;
    }
}

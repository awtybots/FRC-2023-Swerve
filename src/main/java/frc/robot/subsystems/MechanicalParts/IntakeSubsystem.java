package frc.robot.subsystems.MechanicalParts;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Claw;
import frc.robot.RobotContainer;
import frc.robot.subsystems.LedSubsystem;

public class IntakeSubsystem extends SubsystemBase {

    private WPI_TalonFX mIntakeMotor;
    private boolean kKeep;
    private double kIntakePct;
    private final LedSubsystem s_Led;

    private final Debouncer currentFilter = new Debouncer(0.2, DebounceType.kRising);

    // private double idlePct = 0.06;

    public IntakeSubsystem(LedSubsystem ledSubsystem) {
        s_Led = ledSubsystem;
        mIntakeMotor = new WPI_TalonFX(Claw.kIntakeMotorId);
        configMotors();
    }

    private void configMotors() {
        mIntakeMotor.configFactoryDefault();
        mIntakeMotor.configNeutralDeadband(0.0);
        mIntakeMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        mIntakeMotor.setNeutralMode(NeutralMode.Brake);
        mIntakeMotor.configPeakOutputForward(Claw.kMaxPercentOutput);
        mIntakeMotor.configPeakOutputReverse(-Claw.kMaxPercentOutput);
    }

    public void intake(double pct, boolean keep) {
        kKeep = keep;
        // if(pct == 0) {
        //     if(RobotContainer.getIsCone()) {
        //         intake(-idlePct, true);
        //     } else {
        //         intake(idlePct, true);
        //     }
        //     return;
        // }
        if (keep) {
            kIntakePct = pct * Claw.kMaxPercentOutput;
        } else {
            mIntakeMotor.set(pct * Claw.kMaxPercentOutput);
        }
    }

    public double getOutputCurrent() {
        return mIntakeMotor.getSupplyCurrent();
    }

    public void stopIntake() {
        mIntakeMotor.set(0);
    }

    @Override
    public void periodic() {
        if (currentFilter.calculate(getOutputCurrent() > 18)) {
            if (RobotContainer.getIsCone()) {
                s_Led.setHoldAnimation("IntakeCube", false);
                s_Led.setHoldAnimation("IntakeCone", true);
            } else {
                s_Led.setHoldAnimation("IntakeCube", true);
                s_Led.setHoldAnimation("IntakeCone", false);
            }
        } else {
            s_Led.setHoldAnimation("IntakeCube", false);
            s_Led.setHoldAnimation("IntakeCone", false);
        }
        // SmartDashboard.putNumber("Claw Current", getOutputCurrent());
        if (kKeep) {
            mIntakeMotor.set(kIntakePct);
        }
    }
}

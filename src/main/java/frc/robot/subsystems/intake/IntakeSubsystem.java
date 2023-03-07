
package frc.robot.subsystems.intake;

import com.hamosad1657.lib.motors.HaTalonSRX;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotMap;

public class IntakeSubsystem extends SubsystemBase {
	private static IntakeSubsystem instance;

	public static IntakeSubsystem getInstance() {
		if (instance == null) {
			instance = new IntakeSubsystem();
		}
		return instance;
	}

	private HaTalonSRX intakeMotor;
	private DigitalInput raiseLimit, bottomLimit;
	private boolean isIntakeOpen;

	private IntakeSubsystem() {
		this.intakeMotor = new HaTalonSRX(RobotMap.kIntakeMotorID);
		this.intakeMotor.setIdleMode(IdleMode.kBrake);
		this.isIntakeOpen = false;
		this.raiseLimit = new DigitalInput(RobotMap.kIntakeRaiseLimitPort);
		this.bottomLimit = new DigitalInput(RobotMap.kIntakeBottomLimitPort);
	}

	/**
	 * Toggles the intake's motors.
	 */
	public Command toggleIntake() {
		if (this.isIntakeOpen) {
			this.isIntakeOpen = false;
			return this.raiseIntakeCommand();
		} else {
			this.isIntakeOpen = true;
			return this.lowerIntakeCommand();
		}
	}

	public Command lowerIntakeCommand() {
		if (!this.bottomLimit.get()) {
			return new SequentialCommandGroup(
					new InstantCommand(() -> this.intakeMotor.set(-IntakeConstants.kDeafultSpeed), this),
					new WaitCommand(IntakeConstants.kLoweringWaitingTime), new InstantCommand(() -> {
						this.intakeMotor.set(0.0);
						this.isIntakeOpen = true;
					}, this));
		}
		return new InstantCommand();

	}

	public Command raiseIntakeCommand() {
		if (!this.raiseLimit.get()) {
			return new SequentialCommandGroup(
					new InstantCommand(() -> this.intakeMotor.set(IntakeConstants.kDeafultSpeed), this),
					new WaitCommand(IntakeConstants.kRaisingWaitingTime), new InstantCommand(() -> {
						this.intakeMotor.set(0.0);
						this.isIntakeOpen = false;
					}, this));
		}
		return new InstantCommand();
	}

	public Command keepIntakeUpCommand() {
		return new InstantCommand(
				() -> this.intakeMotor.set(this.isIntakeOpen ? 0.0 : IntakeConstants.kKeepInPlaceSpeed), this);
	}

	public void setIntakeMotor(double speedPercentOutput) {
		this.intakeMotor.set(speedPercentOutput);
	}
}

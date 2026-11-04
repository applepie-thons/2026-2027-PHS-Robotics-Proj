package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.core.CoreCANcoder;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConsts;
import frc.robot.Constants.ModuleConsts;

public class SwerveModule {
    private final TalonFX speedMotor;
	public final TalonFX directionMotor;
	// Tracks the current angle of motor. This retains its values over a powercycle,
	// unlike the TalonFX's builtin encoder.
	private final CoreCANcoder absoluteEncoder;

	private final boolean reverseSpeedMotor;
	private final boolean reverseDirectionMotor;
	private final boolean reverseAbsoluteEncoder;
	private final boolean reverseSpeedEncoder;

	private final PIDController directionController;
	private TalonFXConfiguration currentLimitConfig = new TalonFXConfiguration();

	// Used to tell modules apart in SmartDashboard logging.
	String moduleName;

    public SwerveModule(String moduleName, int speedMotorId, int directionMotorId,
			int absoluteEncoderId, boolean reverseSpeedMotor,
			boolean reverseDirectionMotor, boolean reverseAbsoluteEncoder,
			boolean reverseSpeedEncoder)
    {
        
        this.moduleName = moduleName;
        this.speedMotor = new TalonFX(speedMotorId);
        this.directionMotor = new TalonFX(directionMotorId);
        this.absoluteEncoder = new CoreCANcoder(absoluteEncoderId);
        this.reverseSpeedMotor = reverseSpeedMotor;
        this.reverseDirectionMotor = reverseDirectionMotor;
        this.reverseAbsoluteEncoder = reverseAbsoluteEncoder;
        this.reverseSpeedEncoder = reverseSpeedEncoder;

        this.directionController = new PIDController(ModuleConsts.directionP, 0, 0);
        this.directionController.enableContinuousInput(-Math.PI, Math.PI);

        this.speedMotor.setPosition(0);


        currentLimitConfig.CurrentLimits.StatorCurrentLimit = 100;
		currentLimitConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		currentLimitConfig.CurrentLimits.SupplyCurrentLimit = 55;
		currentLimitConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		currentLimitConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

		for (int i = 0; i < 5; i++) {
			speedMotor.getConfigurator().apply(currentLimitConfig);
			directionMotor.getConfigurator().apply(currentLimitConfig);
		}

    }

    public double getDrivePosition() {
		double motorRotations = this.speedMotor.getPosition().getValueAsDouble();

		// Do the following unit conversion:
		// (MotorRotation) * (Meters / MotorRotation) -> (Meters)
		double meters = motorRotations * ModuleConsts.speedMotorRotationToMeters;

		int reverseFactor = (this.reverseSpeedEncoder ? -1 : 1);
		return meters * reverseFactor;
	}

    public double getDriveVelocity() {
		double motorRotationsPerSec = this.speedMotor.getVelocity().getValueAsDouble();

		// Do the following unit conversion:
		// (MotorRotation / Sec) * (Meters / MotorRotation) -> (Meters / Sec)
		double metersPerSec = motorRotationsPerSec * ModuleConsts.speedMotorRotationToMeters;

		int reverseFactor = (this.reverseSpeedEncoder ? -1 : 1);
		return metersPerSec * reverseFactor;
	}

	public double getAbsoluteEncoderRad() {
		double angleRad = this.absoluteEncoder.getAbsolutePosition().getValue().baseUnitMagnitude();
		return (this.reverseAbsoluteEncoder ? -angleRad : angleRad);
	}

	public SwerveModuleState getModuleState() {
		return new SwerveModuleState(
				getDriveVelocity(), new Rotation2d(getAbsoluteEncoderRad()));
	}

	public SwerveModulePosition getModulePosition() {
		return new SwerveModulePosition(
				getDrivePosition(), new Rotation2d(getAbsoluteEncoderRad()));
	}

	public void setDesiredState(SwerveModuleState state, boolean ignoreLowSpeed) {
		// This "if" condition prevents the wheels from swinging back to their
		// neutral position when the joysticks are let go.
		if (ignoreLowSpeed && Math.abs(state.speedMetersPerSecond) < 0.02) {
			stop();
			return;
		}

		state.optimize(getModuleState().angle);

		// The SwerveDriveKinematics page of WPILib says that this improves the "skew"
		// of swerve drive when changing direction. It does this by reducing the speed of
		// a module when it's not pointing in the desired direction.
		state.speedMetersPerSecond *= state.angle.minus(getModuleState().angle).getCos();

		double newSpeedMotorVal = state.speedMetersPerSecond / DriveConsts.maxMetersPerSecToMotorSpeed;
		int speedReverse = reverseSpeedMotor ? -1 : 1;
		speedMotor.set(newSpeedMotorVal * speedReverse);

		double newDirectionMotorVal = directionController.calculate(
				getAbsoluteEncoderRad(), state.angle.getRadians());
		int directionReverse = reverseDirectionMotor ? -1 : 1;
		directionMotor.set(newDirectionMotorVal * directionReverse);
	}

	public void stop() {
		speedMotor.set(0);
		directionMotor.set(0);
	}

    // **************************************
    // Functions for Debugging
    // **************************************

    public void log() {

		SmartDashboard.putNumber( moduleName + " speed",
								  getDrivePosition() );
		SmartDashboard.putNumber( moduleName + " velocity",
								  getDriveVelocity() );
	}


}

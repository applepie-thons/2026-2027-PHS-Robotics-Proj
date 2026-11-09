package frc.robot.subsystems;
 
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ConfigConsts;
import frc.robot.Constants.DriveConsts;

public class SwerveDrive extends SubsystemBase
{
    
    public double kP = 5.0;
	public PIDController turn_pid = new PIDController(2.8, 5.0, 0);
	public PIDController turn_pid_d_pad = new PIDController(1.2, 0, 0);
	public PIDController turn_pid_for_moving = new PIDController(2.8, 5.0, 0);
	public PIDController move_pid = new PIDController(2.0, 5.0, 0);

    // Shortened names for convenience:
	// * lf: left-front
	// * rf: right-front
	// * lb: left-back
	// * rb: right-back
	public final SwerveModule lfModule = new SwerveModule(
			"lf",
			ConfigConsts.lfSpeedMotorId,
			ConfigConsts.lfDirectionMotorId,
			ConfigConsts.lfEncoderId,
			ConfigConsts.reverseLfSpeedMotor,
			ConfigConsts.reverseLfDirectionMotor,
			ConfigConsts.reverseLfEncoder,
			ConfigConsts.reverseLfSpeedEncoder);

	public final SwerveModule rfModule = new SwerveModule(
			"rf",
			ConfigConsts.rfSpeedMotorId,
			ConfigConsts.rfDirectionMotorId,
			ConfigConsts.rfEncoderId,
			ConfigConsts.reverseRfSpeedMotor,
			ConfigConsts.reverseRfDirectionMotor,
			ConfigConsts.reverseRfEncoder,
			ConfigConsts.reverseRfSpeedEncoder);

	public final SwerveModule lbModule = new SwerveModule(
			"lb",
			ConfigConsts.lbSpeedMotorId,
			ConfigConsts.lbDirectionMotorId,
			ConfigConsts.lbEncoderId,
			ConfigConsts.reverseLbSpeedMotor,
			ConfigConsts.reverseLbDirectionMotor,
			ConfigConsts.reverseLbEncoder,
			ConfigConsts.reverseLbSpeedEncoder);

	public final SwerveModule rbModule = new SwerveModule(
			"rb",
			ConfigConsts.rbSpeedMotorId,
			ConfigConsts.rbDirectionMotorId,
			ConfigConsts.rbEncoderId,
			ConfigConsts.reverseRbSpeedMotor,
			ConfigConsts.reverseRbDirectionMotor,
			ConfigConsts.reverseRbEncoder,
			ConfigConsts.reverseRbSpeedEncoder);

	public AHRS navxMxp = new AHRS(NavXComType.kMXP_SPI);

    public SwerveModulePosition[] getModulePositions() {
		// TODO_OLD: The ordering of the modules here is guessed based on how we need to pass
		// swerve module states in `setModules()`. Confirm that this is correct.

		return new SwerveModulePosition[]
		{
			rfModule.getModulePosition(),
			lfModule.getModulePosition(),
			rbModule.getModulePosition(),
			lbModule.getModulePosition()
		};
	}

    public SwerveDrive() 
    {
        ResetGyro();

		UpdateTurnPIDController(turn_pid, 0.035);
		UpdateTurnPIDController(turn_pid_for_moving, 0.035);
		UpdateTurnPIDController(turn_pid_d_pad, 2);

		move_pid.setIZone(0.2);
    };

	public void ResetGyro()
	{
		// Calibrate the the NavXMXP in a separate thread, so that it doesn't block other initialization.
		new Thread(() ->  {
			try 
			{
				Thread.sleep(1000);
				navxMxp.reset();

				//initialize after gyro is reset, because it needs the gyro position for initialization
				// odometry = new SwerveDriveOdometry(DriveConsts.driveKinematics, navxMxp.getRotation2d(), getModulePositions());
			} catch (Exception e) {}
		}).start();
	}

	public void UpdateTurnPIDController(PIDController pid, double errorTolerance)
	{
		pid.enableContinuousInput(-Math.PI, Math.PI);
		pid.setTolerance(errorTolerance);
		pid.setIZone(DegToRad(15));
	};

    public void SetKP(double newKP)
	{
		kP = newKP;
		move_pid.setI(kP);
	}

	public void SetModules(double xSpeed, double ySpeed, double turnSpeed)
	{
		/* Hysteresis code. Doesn't really improve the feel of driving, so commented out.
		   xSpeed = xHysteresis.nextValue(xSpeed);
		   ySpeed = yHysteresis.nextValue(ySpeed);
		   turnSpeed = rotHysteresis.nextValue(turnSpeed); */

		// NOTE: The angle that "getRotation2d()" returns is CCW positive.
		Rotation2d gyroAngle = navxMxp.getRotation2d();
		ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turnSpeed, gyroAngle);

		SmartDashboard.putNumber( "Rotation2D Degree", gyroAngle.getDegrees());

		SwerveModuleState[] moduleStates = DriveConsts.driveKinematics.toSwerveModuleStates(chassisSpeeds);
		SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, DriveConsts.maxMetersPerSecToMotorSpeed);

		boolean ignoreLowSpeed = true;
		rfModule.setDesiredState(moduleStates[0], ignoreLowSpeed);
		lfModule.setDesiredState(moduleStates[1], ignoreLowSpeed);
		rbModule.setDesiredState(moduleStates[2], ignoreLowSpeed);
		lbModule.setDesiredState(moduleStates[3], ignoreLowSpeed);

		// odometry.update(navxMxp.getRotation2d(), getModulePositions());
	}

	public void SetWheelsToAngle(double angleRadians)
	{
		double speed = 0;
		Rotation2d rot2d = new Rotation2d(angleRadians);
		SwerveModuleState swerveModuleState = new SwerveModuleState(speed, rot2d);

		boolean ignoreLowSpeed = false;
		lfModule.setDesiredState(swerveModuleState, ignoreLowSpeed);
		rfModule.setDesiredState(swerveModuleState, ignoreLowSpeed);
		lbModule.setDesiredState(swerveModuleState, ignoreLowSpeed);
		rbModule.setDesiredState(swerveModuleState, ignoreLowSpeed);

		// odometry.update(navxMxp.getRotation2d(), getModulePositions());
	}

	public void StopModules()
	{
		lfModule.stop();
		rfModule.stop();
		lbModule.stop();
		rbModule.stop();

		// odometry.update(navxMxp.getRotation2d(), getModulePositions());
	}

	public boolean TurnToDegree(double degree, double errorTolerance)
	{
		if (errorTolerance > 0) turn_pid.setTolerance(Math.toRadians(errorTolerance));

		double currentRadians = -navxMxp.getRotation2d().getRadians();
		double output = turn_pid.calculate(currentRadians, DegToRad(degree - 180));

		SmartDashboard.putNumber("PID Output", output);
		SmartDashboard.putNumber("PID Error", turn_pid.getError());

		if (turn_pid.atSetpoint())
		{
			SetModules(0.0, 0.0, 0.0);
			return true;
		}

		SetModules(0.0, 0.0, output);
		return false;
	}

    // ------- Radian Functions -------- //
	public double DegToRad(double deg) { return deg * (Math.PI/180); }
}

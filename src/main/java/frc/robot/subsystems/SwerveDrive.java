// copyright ME


package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ConfigConsts;
import frc.robot.Constants.DriveConsts;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;


public class SwerveDrive extends SubsystemBase {
    
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
		//
		// 3/22/2026: I don't think we have the need or time to do anything meaningful
		// odometry, which is the only user of this function.
		SwerveModulePosition[] modulePositions = {
			rfModule.getModulePosition(),
			lfModule.getModulePosition(),
			rbModule.getModulePosition(),
			lbModule.getModulePosition()
		};
		return modulePositions;
	}

	public void resetGyro() {
		// Calibrate the the NavXMXP in a separate thread, so that it doesn't block other initialization.
		new Thread(() -> {
			try {
				Thread.sleep(1000);
				navxMxp.reset();

				// initialize after gyro is reset, because it needs the gyro position for initialization
				/*
				odometry = new SwerveDriveOdometry(
					DriveConsts.driveKinematics,
					navxMxp.getRotation2d(),
					getModulePositions());
				*/

			} catch (Exception e) {
			}
		}).start();
	}

    public SwerveDrive() 
    {
        resetGyro();
		turn_pid.enableContinuousInput(-Math.PI, Math.PI);
		turn_pid.setTolerance(0.035);
		turn_pid.setIZone(DegToRad(15));

		turn_pid_for_moving.enableContinuousInput(-Math.PI, Math.PI);
		turn_pid_for_moving.setTolerance(0.035);
		turn_pid_for_moving.setIZone(DegToRad(15));

		turn_pid_d_pad.enableContinuousInput(-Math.PI, Math.PI);
		turn_pid_d_pad.setTolerance(2);
		turn_pid_d_pad.setIZone(DegToRad(15));

		move_pid.setIZone(0.2);
    };

    public void setKP(double newKP) {
		kP = newKP;
		move_pid.setI(kP);
	}

	public void setModules(double xSpeed, double ySpeed, double turnSpeed) {
		// Hysteresis code. Doesn't really improve the feel of driving, so commented out.
		// xSpeed = xHysteresis.nextValue(xSpeed);
		// ySpeed = yHysteresis.nextValue(ySpeed);
		// turnSpeed = rotHysteresis.nextValue(turnSpeed);

		// NOTE: The angle that "getRotation2d()" returns is CCW positive.
		ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
				xSpeed, ySpeed, turnSpeed, navxMxp.getRotation2d());

		SmartDashboard.putNumber( "rotation2d deg",  navxMxp.getRotation2d().getDegrees() );

		SwerveModuleState[] moduleStates = DriveConsts.driveKinematics.toSwerveModuleStates(chassisSpeeds);
		SwerveDriveKinematics.desaturateWheelSpeeds(
				moduleStates, DriveConsts.maxMetersPerSecToMotorSpeed);

		boolean ignoreLowSpeed = true;
		lfModule.setDesiredState(moduleStates[1], ignoreLowSpeed);
		rfModule.setDesiredState(moduleStates[0], ignoreLowSpeed);
		lbModule.setDesiredState(moduleStates[3], ignoreLowSpeed);
		rbModule.setDesiredState(moduleStates[2], ignoreLowSpeed);

		// odometry.update(navxMxp.getRotation2d(), getModulePositions());
	}

	public void setWheelsToAngle(double angleRadians) {
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

	public void stopModules() {
		lfModule.stop();
		rfModule.stop();
		lbModule.stop();
		rbModule.stop();

		// odometry.update(navxMxp.getRotation2d(), getModulePositions());
	}

	public boolean TurnToDegree(double degree, double errorTolerance) {
		double current_degree = -navxMxp.getRotation2d().getRadians();

		// Subtract 180 to put the 0-360 degree input in the -180 to 180 range. This is to match the gyro's radian reading which spans from -pi to pi.
		double result = turn_pid.calculate(current_degree, DegToRad(degree - 180));
		SmartDashboard.putNumber("pid output", result);
		double error = turn_pid.getError();
		SmartDashboard.putNumber("pid error", error);
		if(!turn_pid.atSetpoint()){
			setModules(0.0, 0.0, result);
			return(false);
		}
		else{
			setModules(0.0, 0.0, 0.0);
			return(true);
		}
	}

    // ------- radian functions -------- //
	public double DegToRad(double deg){
		return deg * (Math.PI/180);
	}

}

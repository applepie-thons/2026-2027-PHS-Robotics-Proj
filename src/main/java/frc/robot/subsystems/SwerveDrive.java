// copyright ME


package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ConfigConsts;

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

    // ------- radian functions -------- //
	public double DegToRad(double deg){
		return deg * (Math.PI/180);
	}

}

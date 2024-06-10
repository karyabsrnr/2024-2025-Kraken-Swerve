package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.Conversions;
import frc.robot.Robot;
import frc.robot.SwerveModuleConstants;

// Conversions.rotationsToMeters(mDriveMotor.getPosition().getValue(), Constants.Swerve.wheelCircumference), 
//             getCANcoder()
// Conversions.RPSToMPS(mDriveMotor.getVelocity().getValue(), Constants.Swerve.wheelCircumference), 
//             // getCANcoder()
//             Rotation2d.fromRotations(mAngleMotor.getPosition().getValue()),

public class SwerveModule {
  public int moduleNumber;
  private Rotation2d angleOffset;

  private TalonFX mAngleMotor;
  private TalonFX mDriveMotor;
  private CANcoder angleEncoder;
  //sofia added photonvision
  private CANcoder driveEncoder;

  private final SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);

  /* drive motor control requests */
  private final DutyCycleOut driveDutyCycle = new DutyCycleOut(0);
  private final VelocityVoltage driveVelocity = new VelocityVoltage(0);

  /* angle motor control requests */
  private final PositionVoltage anglePosition = new PositionVoltage(0);

  public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){
      this.moduleNumber = moduleNumber;
      this.angleOffset = moduleConstants.angleOffset;
      
      /* Angle Encoder Config */
      angleEncoder = new CANcoder(moduleConstants.cancoderID);
      angleEncoder.getConfigurator().apply(Robot.ctreConfigs.swerveCANcoderConfig);

      /* Angle Encoder Config  Sofia photonvision*/
      driveEncoder = new CANcoder(moduleConstants.cancoderID);
      driveEncoder.getConfigurator().apply(Robot.ctreConfigs.swerveCANcoderConfig);

      /* Angle Motor Config */
      mAngleMotor = new TalonFX(moduleConstants.angleMotorID);
      mAngleMotor.getConfigurator().apply(Robot.ctreConfigs.swerveAngleFXConfig);
      resetToAbsolute();

      /* Drive Motor Config */
      mDriveMotor = new TalonFX(moduleConstants.driveMotorID);
      mDriveMotor.getConfigurator().apply(Robot.ctreConfigs.swerveDriveFXConfig);
      mDriveMotor.getConfigurator().setPosition(0.0);
  }

  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
      desiredState = SwerveModuleState.optimize(desiredState, getState().angle); 
      mAngleMotor.setControl(anglePosition.withPosition(desiredState.angle.getRotations()));
      setSpeed(desiredState, isOpenLoop);
  }

  private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
      if(isOpenLoop){
          driveDutyCycle.Output = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
          mDriveMotor.setControl(driveDutyCycle);
      }
      else {
          driveVelocity.Velocity = Conversions.MPSToRPS(desiredState.speedMetersPerSecond, Constants.Swerve.wheelCircumference);
          driveVelocity.FeedForward = driveFeedForward.calculate(desiredState.speedMetersPerSecond);
          mDriveMotor.setControl(driveVelocity);
      }
  }

  public Rotation2d getCANcoder(){
      return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValue());
  }

  public void resetToAbsolute(){
      double absolutePosition = getCANcoder().getRotations() - angleOffset.getRotations();
      mAngleMotor.setPosition(absolutePosition);
  }

  public SwerveModuleState getState(){
      return new SwerveModuleState(
          Conversions.RPSToMPS(mDriveMotor.getVelocity().getValue(), Constants.Swerve.wheelCircumference), 
          Rotation2d.fromRotations(mAngleMotor.getPosition().getValue())
      );
  }

  public SwerveModulePosition getPosition(){
      return new SwerveModulePosition(
          Conversions.rotationsToMeters(mDriveMotor.getPosition().getValue(), Constants.Swerve.wheelCircumference), 
          Rotation2d.fromRotations(mAngleMotor.getPosition().getValue())
      );
  }

//   //sofia photonvision code copied from wpilib differentialdrive pose estimator
//     private final DifferentialDrivePoseEstimator m_poseEstimator =
//       new DifferentialDrivePoseEstimator(
//           m_kinematics,
//           m_gyro.getRotation2d(),
//           m_leftEncoder.getDistance(),
//           m_rightEncoder.getDistance(),
//           new Pose2d(),
//           VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
//           VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));
//     m_poseEstimator.update(
//         m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
    
//         // Compute the robot's field-relative position exclusively from vision measurements.
//     Pose3d visionMeasurement3d =
//         objectToRobotPose(m_objectInField, m_robotToCamera, m_cameraToObjectEntry);

//     // Convert robot pose from Pose3d to Pose2d needed to apply vision measurements.
//     Pose2d visionMeasurement2d = visionMeasurement3d.toPose2d();

//     // Apply vision measurements. For simulation purposes only, we don't input a latency delay -- on
//     // a real robot, this must be calculated based either on known latency or timestamps.
//     m_poseEstimator.addVisionMeasurement(visionMeasurement2d, Timer.getFPGATimestamp());
}


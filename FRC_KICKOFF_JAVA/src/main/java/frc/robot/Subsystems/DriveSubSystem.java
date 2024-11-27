package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.Constants;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveSubSystem extends SubsystemBase {

    private final Spark leftMotor;
    private final Spark rightMotor;

    private final Encoder leftEncoder;
    private final Encoder rightEncoder;

    private final PIDController leftController;
    private final PIDController rightController;

    private final SimpleMotorFeedforward leftfeFeedforward;
    private final SimpleMotorFeedforward rightfeFeedforward;

    private final ADIS16470_IMU imu;
    private final DifferentialDriveOdometry odometry;
    private final DifferentialDriveKinematics kinematics;

    private final SlewRateLimiter speedLimiter;
    private double leftOutput = 0;
    private double rightOutput = 0;

   

    public DriveSubSystem() {

        leftMotor = new Spark(Constants.LEFT_MOTOR_PORT);
        rightMotor = new Spark(Constants.RIGHT_MOTOR_PORT);
        leftMotor.setInverted(false);
        rightMotor.setInverted(true);

        leftEncoder = new Encoder(Constants.LEFT_ENCODER_A_PORT, Constants.LEFT_ENCODER_B_PORT);
        rightEncoder = new Encoder(Constants.RIGHT_ENCODER_A_PORT, Constants.RIGHT_ENCODER_B_PORT);
        leftEncoder.setDistancePerPulse(0.000234);
        leftEncoder.setReverseDirection(true);
        rightEncoder.setDistancePerPulse(0.000234);

        leftController = new PIDController(Constants.LEFT_KP, Constants.LEFT_KI, Constants.LEFT_KD);
        rightController = new PIDController(Constants.RIGHT_KP, Constants.RIGHT_KI, Constants.RIGHT_KD);

        leftfeFeedforward = new SimpleMotorFeedforward(0.1, 0.3);
        rightfeFeedforward = new SimpleMotorFeedforward(0.1, 0.3);
        imu = new ADIS16470_IMU();

        odometry = new DifferentialDriveOdometry(new Rotation2d(imu.getAngle()), 0, 0,
                new Pose2d(0, 0, new Rotation2d()));
        kinematics = new DifferentialDriveKinematics(1);
        speedLimiter = new SlewRateLimiter(3.0); // Limit speed change rate to 3 units per second

        AutoBuilder.configureLTV(
                this::getPose,
                this::resetPose,
                this::getRobotChassisSpeeds,
                this::setChassisSpeedsPID, 0.2,
                new ReplanningConfig(),
                () -> false,
                this);// Reference to this subsystem to set requirements
    }

    public void arcadeDrive(double forwardSpeed, double rotationSpeed) {
        // Apply speed limiter
        double setFwdSpeed = forwardSpeed;
        double setRotSpeed = rotationSpeed;
        forwardSpeed = speedLimiter.calculate(forwardSpeed);
        if ((Math.abs(forwardSpeed) + Math.abs(rotationSpeed) > 1)) {
            setFwdSpeed = forwardSpeed / (Math.abs(forwardSpeed) + Math.abs(rotationSpeed));
            setRotSpeed = rotationSpeed / (Math.abs(forwardSpeed) + Math.abs(rotationSpeed));
        }

        // Drive the robot
        this.leftOutput = setFwdSpeed + setRotSpeed;
        this.rightOutput = setFwdSpeed - setRotSpeed;
    }

    public void leftMotorSetPID(double setpoint) {

        leftController.setSetpoint(setpoint);
    }

    public void rightMotorSetPID(double setpoint) {

        rightController.setSetpoint(setpoint);
    }

    public void arcadeDrivePID(double forwardSpeed, double rotationSpeed) {
        double setFwdSpeed = MathUtil.applyDeadband(forwardSpeed, 0.1);
        double setRotSpeed = MathUtil.applyDeadband(rotationSpeed, 0.1);
        forwardSpeed = speedLimiter.calculate(setFwdSpeed);
        if ((Math.abs(forwardSpeed) + Math.abs(rotationSpeed) > 1)) {
            setFwdSpeed = forwardSpeed / (Math.abs(forwardSpeed) + Math.abs(rotationSpeed));
            setRotSpeed = rotationSpeed / (Math.abs(forwardSpeed) + Math.abs(rotationSpeed));
        }

        // Drive the robot
        double setVelLeft = (setFwdSpeed + setRotSpeed) * Constants.MAX_WHEEL_SPEED;
        double setVelRight = (setFwdSpeed - setRotSpeed) * Constants.MAX_WHEEL_SPEED;

        if (setVelLeft!=leftController.getSetpoint())
        {
        leftController.setSetpoint(setVelLeft);
        }
        if (setVelRight!=rightController.getSetpoint())
        {
        rightController.setSetpoint(setVelRight);
        }   
        leftOutput = leftController.calculate(leftEncoder.getRate()) + leftfeFeedforward.calculate(setVelLeft);
        rightOutput = rightController.calculate(rightEncoder.getRate()) + rightfeFeedforward.calculate(setVelRight);

        leftMotor.set(leftOutput);
        rightMotor.set(rightOutput);

    }

    public DifferentialDriveOdometry getOdometry() {
        return odometry;
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void resetPose(Pose2d pose) {
        odometry.resetPosition(new Rotation2d(imu.getAngle()),
                new DifferentialDriveWheelPositions(leftEncoder.getDistance(), rightEncoder.getDistance()), pose);

    }

    public ChassisSpeeds getRobotChassisSpeeds() {
        DifferentialDriveWheelSpeeds robotspeeds = new DifferentialDriveWheelSpeeds(rightEncoder.getRate(),
                leftEncoder.getRate());
        return kinematics.toChassisSpeeds(robotspeeds);
    }

    private void setChassisSpeeds(ChassisSpeeds speeds) {
        DifferentialDriveWheelSpeeds setspeeds = kinematics.toWheelSpeeds(speeds);
        if (Math.abs(setspeeds.leftMetersPerSecond) > 1.5 || Math.abs(setspeeds.rightMetersPerSecond) > 1.5) {
            leftMotor.set(0);
            rightMotor.set(0);
        } else
            leftMotor.set(setspeeds.leftMetersPerSecond / Constants.MAX_WHEEL_SPEED);
        rightMotor.set(setspeeds.rightMetersPerSecond / Constants.MAX_WHEEL_SPEED);

    }

    private void setChassisSpeedsPID(ChassisSpeeds speeds) {

        DifferentialDriveWheelSpeeds setspeeds = kinematics.toWheelSpeeds(speeds);
        if (Math.abs(setspeeds.leftMetersPerSecond) > 1.5 || Math.abs(setspeeds.rightMetersPerSecond) > 1.5) {
            leftMotorSetPID(0);
            rightMotorSetPID(0);
        }
        leftMotorSetPID(setspeeds.leftMetersPerSecond);
        rightMotorSetPID(setspeeds.rightMetersPerSecond);

        leftController.setSetpoint(setspeeds.leftMetersPerSecond);
        rightController.setSetpoint(setspeeds.rightMetersPerSecond);

        leftOutput = leftController.calculate(leftEncoder.getRate()) + leftfeFeedforward.calculate(setspeeds.leftMetersPerSecond/Constants.MAX_WHEEL_SPEED);
        rightOutput = rightController.calculate(rightEncoder.getRate()) + rightfeFeedforward.calculate(setspeeds.rightMetersPerSecond/Constants.MAX_WHEEL_SPEED);

        leftMotor.set(leftOutput);
        rightMotor.set(rightOutput);

    }

    @Override
    public void periodic() {

        // Update odometry based on encoder readings and IMU
        odometry.update(new Rotation2d(imu.getAngle()), leftEncoder.getDistance(), rightEncoder.getDistance());

        // Output the robot's pose to SmartDashboard for debugging
        Pose2d pose = odometry.getPoseMeters();
        SmartDashboard.putString("Robot Pose", pose.toString());
        SmartDashboard.putNumber("left encoder", leftEncoder.getDistance());
        SmartDashboard.putNumber("right encoder", rightEncoder.getDistance());
        SmartDashboard.putNumber("left encoder rate", leftEncoder.getRate());
        SmartDashboard.putNumber("right encoder rate ", rightEncoder.getRate());

        SmartDashboard.putNumber("left setpoint", leftController.getSetpoint());
        SmartDashboard.putNumber("right setpoint", rightController.getSetpoint());
        SmartDashboard.putNumber("left output", leftController.getPositionError());

    }

}

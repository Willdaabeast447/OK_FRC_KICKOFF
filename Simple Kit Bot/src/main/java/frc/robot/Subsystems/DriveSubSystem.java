package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.Command;
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

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;

public class DriveSubSystem extends SubsystemBase {

    private final Spark leftMotor;
    private final Spark rightMotor;

    private final Encoder leftEncoder;
    private final Encoder rightEncoder;

    private final ADIS16470_IMU imu;
    private final DifferentialDriveOdometry odometry;
    private final DifferentialDriveKinematics kinematics;

    private final SlewRateLimiter speedLimiter;

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

        imu = new ADIS16470_IMU();

        odometry = new DifferentialDriveOdometry(new Rotation2d(imu.getAngle()), 0, 0,
                new Pose2d(0, 0, new Rotation2d()));
        kinematics = new DifferentialDriveKinematics(1);
        speedLimiter = new SlewRateLimiter(3.0); // Limit speed change rate to 3 units per second

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
        leftMotor.set(setFwdSpeed + setRotSpeed);
        rightMotor.set(setFwdSpeed - setRotSpeed);
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

    @Override
    public void periodic() {

        // Update odometry based on encoder readings and IMU
        odometry.update(new Rotation2d(imu.getAngle()), leftEncoder.getDistance(), rightEncoder.getDistance());

    }

    public Command arcadeDriveCommand(DoubleSupplier forwardVel,DoubleSupplier rotDoubleSupplier)
    {
        return run(()->{this.arcadeDrive(forwardVel.getAsDouble(), rotDoubleSupplier.getAsDouble());});
    }

}

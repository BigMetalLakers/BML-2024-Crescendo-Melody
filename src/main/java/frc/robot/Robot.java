// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// import org.photonvision.PhotonCamera; //https://maven.photonvision.org/repository/internal/org/photonvision/PhotonLib-json/1.0/PhotonLib-json-1.0.json

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;//https://software-metadata.revrobotics.com/REVLib-2024.json

import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI; // required for ADIS IMUs
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  private final Spark m_rightDrive = new Spark(1);
  private final Spark m_leftDrive = new Spark(0);
  private final Spark intakeRollerMotor = new Spark(2);

  private CANSparkMax Shooter_R = new CANSparkMax(24, MotorType.kBrushless);
  private CANSparkMax Shooter_L = new CANSparkMax(23, MotorType.kBrushless);
  private CANSparkMax Loader_R = new CANSparkMax(22, MotorType.kBrushless);
  private CANSparkMax Loader_L = new CANSparkMax(21, MotorType.kBrushless);
  private CANSparkMax intakeNeo = new CANSparkMax(3, MotorType.kBrushless);
  private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftDrive, m_rightDrive);
  private final XboxController m_stick = new XboxController(0);
  private final Timer m_timer = new Timer();
  private final Encoder right_encoder = new Encoder(6, 7, true, CounterBase.EncodingType.k4X);
  private final Encoder left_encoder = new Encoder(8, 9, false, CounterBase.EncodingType.k4X);
  public static final ADIS16470_IMU imu = new ADIS16470_IMU(ADIS16470_IMU.IMUAxis.kY, ADIS16470_IMU.IMUAxis.kX, ADIS16470_IMU.IMUAxis.kZ, SPI.Port.kOnboardCS0, ADIS16470_IMU.CalibrationTime._1s);
  // private static final double kAngleSetPoint = 0.0;
  // private static final double kP = 0.05;
  // PhotonCamera noteCamera = new PhotonCamera("AVerMedia_PW315");
  // PhotonCamera frontCamera = new PhotonCamera("Microsoft_LifeCam_HD-3000");
  public static double speedForward, rotationClockwise, cYaw, targetYaw, tYaw;
  public static double sharpness = 0.3;
  public static double maxTurnEffort = 0.7;
  public static double intakeSpeed = 0;
  public static double MaxIntakeSpeed = 1;
  public static double loaderSpeed = 0;
  public static double shooterSpeed = 0;
  public static double roboTimer = 0;
  public static boolean hasTarget;

  @Override
  public void robotInit() {
    imu.calibrate();
    m_rightDrive.setInverted(true);
    right_encoder.setSamplesToAverage(5);
    left_encoder.setSamplesToAverage(5);
    right_encoder.setDistancePerPulse(10.0 * 0.3048 / 12928); // pulses averaged from l & r = 12928 of 10' (5 x 2' tiles)
    left_encoder.setDistancePerPulse(10.0 * 0.3048 / 12928); // 0.3048 converts feet into metres
    right_encoder.setMinRate(0.01);
    left_encoder.setMinRate(0.01);
    right_encoder.reset();
    left_encoder.reset();
    Loader_L.setIdleMode(IdleMode.kBrake);
    Loader_R.setIdleMode(IdleMode.kBrake);
    m_timer.reset();
    m_timer.start();
    // SmartDashboard.putNumber("sharpness", sharpness);
    // SmartDashboard.putNumber("maxTurnEffort", maxTurnEffort);
  }

  @Override
  public void robotPeriodic() {
    // cYaw = -imu.getAngle(ADIS16470_IMU.IMUAxis.kY) % 360;
    // var latestResult = noteCamera.getLatestResult();
    // hasTarget = latestResult.hasTargets();
    // SmartDashboard.putBoolean("hasTargets", hasTarget);
    // if (hasTarget) {
    //   targetYaw = latestResult.getBestTarget().getYaw();
    //   tYaw = cYaw - targetYaw;
    //   SmartDashboard.putNumber("targetYaw", targetYaw);
    //   SmartDashboard.putNumber("tYaw", tYaw);
    // }
    // sharpness = SmartDashboard.getNumber("sharpness", sharpness);
    // maxTurnEffort = SmartDashboard.getNumber("maxTurnEffort", maxTurnEffort);
    // SmartDashboard.putNumber("cYaw", cYaw);
    SmartDashboard.putNumber("speedForward", speedForward);
    SmartDashboard.putNumber("shooterSpeed", shooterSpeed);
    SmartDashboard.putNumber("loaderSpeed", loaderSpeed);
    SmartDashboard.putNumber("roboTimer", roboTimer);
    // SmartDashboard.putNumber("rotationClockwise", rotationClockwise);
    roboTimer = m_timer.get();
  }

  @Override
  public void autonomousInit() {
    right_encoder.reset();
    left_encoder.reset();
    m_timer.reset();
    m_timer.start();
    imu.reset();
    // is this needed?
    speedForward = 0;
    shooterSpeed = 0;
    loaderSpeed = 0;
  }

  @Override
  public void autonomousPeriodic() {
    // double turnValue = (kAngleSetPoint - imu.getAngle(null)) * kP;
    // turnValue = Math.copySign(turnValue, m_stick.getRightY());
    intakeNeo.set(0.25);
    intakeRollerMotor.set(-1);
    if (roboTimer <= 1.5) { // 1.5
      shooterSpeed = 1;
    } else if (roboTimer <= 2) { // 0.5
      loaderSpeed = 0.3;
      shooterSpeed = 1;
    } else if (roboTimer <= 3.8) { // 1.8
      speedForward = -0.7;
      loaderSpeed = 0;
      shooterSpeed = 0;
    } else if (roboTimer <= 4.8) { // 1
      speedForward = -0.3;
    } else if (roboTimer <= 7.5) { // 2.8
      speedForward = 0.68;
      shooterSpeed = 1;
    } else if (roboTimer <= 9) { // 1.5
      loaderSpeed = 0.3;
      shooterSpeed = 1;
    } else {
      loaderSpeed = 0;
      shooterSpeed = 0;
    }
    Loader_L.set(loaderSpeed);
    Loader_R.set(-loaderSpeed);
    Shooter_L.set(shooterSpeed);
    Shooter_R.set(-shooterSpeed);
    // m_robotDrive.arcadeDrive(speedForward, turnValue);
    m_robotDrive.arcadeDrive(speedForward, 0);
  }

  @Override
  public void teleopPeriodic() {
    if (m_stick.getRightBumper()) {
      right_encoder.reset();
      left_encoder.reset();
    }

    // forward & backwards movement
    speedForward = (m_stick.getRightTriggerAxis() - m_stick.getLeftTriggerAxis());

    // rotation
    rotationClockwise = m_stick.getLeftX() * -0.6;

    // intake forward & backwards
    intakeSpeed = m_stick.getAButton() ? MaxIntakeSpeed : 0;
    intakeSpeed = m_stick.getYButton() ? -MaxIntakeSpeed : intakeSpeed;

    // shoot
    loaderSpeed = Math.abs(m_stick.getRightY()) >= 0.5 ? m_stick.getRightY() * -1 : 0;
    shooterSpeed = m_stick.getLeftBumper() ? 0.5: 0; //nerfed to 1/2 speed for demo

    // set methods
    Loader_L.set(loaderSpeed);
    Loader_R.set(-loaderSpeed);
    Shooter_L.set(shooterSpeed);
    Shooter_R.set(-shooterSpeed);

    intakeNeo.set(intakeSpeed * 0.25);
    intakeRollerMotor.set(-intakeSpeed);

    // if (m_stick.getXButton()) {
    //   rotationClockwise = (kAngleSetPoint - imu.getAngle(n0ull)) * kP;
    //   rotationClockwise = Math.copySign(rotationClockwise, m_stick.getRightY());
    // }

    // // rotationClockwise = m_stick.getBButton()
    // //     ? (1 - Math.exp(-sharpness * Math.pow(tYaw, 2))) * Math.signum(tYaw) * maxTurnEffort
    // //     : rotationClockwise;
    // rotationClockwise = m_stick.getBButton()
    //     ? (1 - Math.exp(sharpness * Math.pow(targetYaw, 2))) * Math.signum(targetYaw) * maxTurnEffort
    //     : rotationClockwise;

    m_robotDrive.arcadeDrive(speedForward, rotationClockwise, false);
  }
}

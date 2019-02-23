/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class Drivetrain {

  private double ramp = 1.5;
  private double prevY = 0;

  private TalonSRX m_leftMaster, m_rightMaster;
  private VictorSPX m_leftFollowerOne, m_leftFollowerTwo, m_rightFollowerOne, m_rightFollowerTwo;
  private AnalogInput m_infaredBottom;
  private Solenoid m_liftGear;

  //#region Teleop Data

  private boolean liftgearExtended = false;

  //#endregion

  public Drivetrain() {
    m_leftMaster = new TalonSRX(RobotMap.Ports.leftTalon);
    m_rightMaster = new TalonSRX(RobotMap.Ports.rightTalon);
    m_leftFollowerOne = new VictorSPX(RobotMap.Ports.leftVictor1);
    m_leftFollowerTwo = new VictorSPX(RobotMap.Ports.leftVictor2);
    m_rightFollowerOne = new VictorSPX(RobotMap.Ports.rightVictor1);
    m_rightFollowerTwo = new VictorSPX(RobotMap.Ports.rightVictor2);

    m_leftFollowerOne.follow(m_leftMaster);
    m_leftFollowerTwo.follow(m_leftMaster);
    m_rightFollowerOne.follow(m_rightMaster);
    m_rightFollowerTwo.follow(m_rightMaster);

    m_leftMaster.setInverted(false);
    m_leftFollowerOne.setInverted(false);
    m_leftFollowerTwo.setInverted(false);

    m_rightMaster.setInverted(true);
    m_rightFollowerOne.setInverted(true);
    m_rightFollowerTwo.setInverted(true);

    m_leftMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 5);
    m_rightMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 5);

    m_leftMaster.configNominalOutputForward(0, 5);
    m_rightMaster.configNominalOutputReverse(0, 5);
    m_leftMaster.configPeakOutputForward(1, 5);
    m_rightMaster.configPeakOutputReverse(-1, 5);

    m_leftMaster.configPeakCurrentLimit(40, 5);
    m_leftMaster.configPeakCurrentDuration(100, 5);
    m_leftMaster.configContinuousCurrentLimit(30, 5);
    m_leftMaster.enableCurrentLimit(true);
    m_rightMaster.configPeakCurrentLimit(40, 5);
    m_rightMaster.configPeakCurrentDuration(100, 5);
    m_rightMaster.configContinuousCurrentLimit(30, 5);
    m_rightMaster.enableCurrentLimit(true);

    m_leftMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 40, 5);
    m_rightMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 40, 5);

    m_liftGear = new Solenoid(RobotMap.Ports.liftGearSolenoid); // Not sure if thats the right id

    m_infaredBottom = new AnalogInput(RobotMap.Ports.bellyPanInfared);
  }

  public void teleopPeriodic() {
    double arcadeY = Robot.kGamepad1.getRawAxis(1);
    double arcadeX = Robot.kGamepad1.getRawAxis(0);

    arcadeDrive(arcadeY, arcadeX);

    if (m_infaredBottom.getVoltage() < 0.4 && !liftgearExtended) {
      liftgearExtended = true;
    }

    if (liftgearExtended) {
      if (m_infaredBottom.getVoltage() > 0.95) {
        setLiftgear(false);
        liftgearExtended = false;
      }
    }

    if (Robot.kGamepad1.getRawButton(RobotMap.Buttons.buttonB) && !m_liftGear.get()) {
      setLiftgear(true);
    } else if (Robot.kGamepad1.getRawButton(RobotMap.Buttons.buttonY)) {
      setLiftgear(false);
    }
  }

  //#region Operator Functions

  /**
   * Set the forward and turn varibles for the drivetrain. By default it is limited to
   * to the power acceleration of [ramp] per seconds ^2
   * @param Y The forwards and backwards power
   * @param X The turn power. This varible isn't limited by power accel
   */
  public void arcadeDrive(double Y, double X) {

    double newY = Y;

    prevY = (m_leftMaster.getMotorOutputPercent() + m_rightMaster.getMotorOutputPercent()) / 2;

    double maxIncrement = Robot.kDeltaTime * ramp;

    if (Math.abs(Y - prevY) > maxIncrement){
      double sign = (Y - prevY) / Math.abs(Y - prevY);
      newY = (maxIncrement * sign) + prevY;
    }

    m_leftMaster.set(ControlMode.Current, bound(1, -1, newY + X));
    m_rightMaster.set(ControlMode.Current, bound(1, -1, newY - X));

    prevY = newY;
  }

  /**
   * Set the power for each side of the drivetrain individually.
   * @param left Power for the left side
   * @param right Power for the right side
   */
  public void tankDrive(double left, double right) {
    m_leftMaster.set(ControlMode.Current, bound(1, -1, left));
    m_rightMaster.set(ControlMode.Current, bound(1, -1, right));
  }

  /**
   * Toggles the liftgear's state
   * @return The new state of the liftgear
   */
  public boolean toggleLiftgear() {
    if (m_liftGear.get()) {
      m_liftGear.set(false);
    } else {
      m_liftGear.set(true);
    }

    return m_liftGear.get();
  }

  /**
   * Set the state of the liftgear
   * @param s True for engaged and False for disengaged
   */
  public void setLiftgear(boolean s) {
    m_liftGear.set(s);
  }

  //#endregion

  //#region Utility Functions

  public double getLeftEncoder() {
    return m_leftMaster.getSelectedSensorPosition(0);
  }

  public double getLeftVelocity() {
    return m_leftMaster.getSelectedSensorVelocity(0);
  }

  public double getRightEncoder() {
    return m_rightMaster.getSelectedSensorPosition(0);
  }

  public double getRightVelocity() {
    return m_rightMaster.getSelectedSensorVelocity(0);
  }

  public void setIdle(boolean brake) {
    if (brake) {
      m_leftMaster.setNeutralMode(NeutralMode.Brake);
      m_leftFollowerOne.setNeutralMode(NeutralMode.Brake);
      m_leftFollowerTwo.setNeutralMode(NeutralMode.Brake);
      m_rightMaster.setNeutralMode(NeutralMode.Brake);
      m_rightFollowerOne.setNeutralMode(NeutralMode.Brake);
      m_rightFollowerTwo.setNeutralMode(NeutralMode.Brake);
    } else {
      m_leftMaster.setNeutralMode(NeutralMode.Coast);
      m_leftFollowerOne.setNeutralMode(NeutralMode.Coast);
      m_leftFollowerTwo.setNeutralMode(NeutralMode.Coast);
      m_rightMaster.setNeutralMode(NeutralMode.Coast);
      m_rightFollowerOne.setNeutralMode(NeutralMode.Coast);
      m_rightFollowerTwo.setNeutralMode(NeutralMode.Coast);
    }
  }

  private double bound(double max, double min, double val) {
    if (val < min) {
      return min;
    } else if (val > max) {
      return max;
    } else {
      return val;
    }
  }

  //#endregion

  //#region Data Management

  public void updateSmartDashboard() {
    SmartDashboard.putNumber("Drivetrain Left Pos", getLeftEncoder());
    SmartDashboard.putNumber("Drivetrain Right Pos", getRightEncoder());
    SmartDashboard.putNumber("Drivetrain Left Velo", getLeftVelocity());
    SmartDashboard.putNumber("Drivetrain Right Velo", getRightVelocity());
  }

  //#endregion

}
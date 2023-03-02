
package frc.robot;

import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {

  TalonFX armMotor = new TalonFX(9);
  CANCoder enc = new CANCoder(19);
  DigitalInput limitUpper = new DigitalInput(0);
  DigitalInput limitLower = new DigitalInput(1);
  Joystick joy = new Joystick(0);
  double armkP = 1.7, armkD = 80, armkI = 0.0012, armkIZone=70, armkF = 0;
  double armkPdown = 0.15, armkDdown = 10, armkIdown = 2, armkIZonedown, armkFdown = 0;
  double armkG = 0.0721, armkS =-0.0138, armkV = 0, armkA = 0;
  double armkG2 = 0.05, armkS2 =-0.01, armkV2 = 0, armkA2 = 0;
  double armkG3 = 0.0721, armkS3 =-0.0138, armkV3 = 0, armkA3 = 0;
  double armPos1 = 1200, armPos2 = 1309, armPos3 = 2400;
  double armMMVel = 120, armMMAcc = 100;
  double armForwardSensorLim = 2600, armReverseSensorLim = 1140;
  double armMaxOutput = .125;

  double angle, angleCounts;
  double angleFromHorizontalDeg, angleFromHorizontalCounts, angleFromHorizontalRad;
  double cosAngleFromHorizontal;
  double angleOffsetDeg = 186.3;
  double angleOffsetCounts = angleOffsetDeg * 4096 / 360.;
  double angleRate, angleRatePrev = 0, angleRate2;
  double voltage;
  double time = 0, timePrev = 0;
  int button = 0;
  double stick;
  boolean logging = false, loggingPrev = false;
  boolean ls_upper = true, ls_lower = false, ls_upperActive = false;
  FileWriter writer;
  double voltageauto=0,autoTime;
  boolean autoFlagA=false,autoFlagB=false;

  public Robot() {
    super(0.02);
    LiveWindow.disableAllTelemetry();
  }

  @Override
  public void robotInit() {

    try {
      String fileName = "/home/lvuser/data" + new SimpleDateFormat("yyyyMMddhhmm'.txt'").format(new Date());
        writer = new FileWriter(fileName);
        writer.write("time,voltage,angleCounts,rate,rate2");
        writer.write("\n");
    } catch (IOException e) {
      System.out.println("Error opening log file");
      e.printStackTrace();
    }

    enc.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
    enc.setPositionToAbsolute(20);
    enc.configSensorDirection(true);
    enc.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 5, 20);

    armMotor.configFactoryDefault();
    

    // set voltage compensation
    armMotor.configVoltageCompSaturation(11.5);
    armMotor.enableVoltageCompensation(true);

    // Set encoder values as limit switch
    armMotor.configForwardSoftLimitThreshold(armForwardSensorLim);
    armMotor.configReverseSoftLimitThreshold(armReverseSensorLim);
    armMotor.configForwardSoftLimitEnable(true);
    armMotor.configReverseSoftLimitEnable(true);
    // armM

    // set to brake mode
    armMotor.setNeutralMode(NeutralMode.Brake);

    // set feedbak sensor to cancoder
    armMotor.configRemoteFeedbackFilter(enc, 0);
    armMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.RemoteSensor0, 0, 20);

    // set max closed loop output
    armMotor.configClosedLoopPeakOutput(0, armMaxOutput);
    armMotor.configPeakOutputForward(armMaxOutput);
    armMotor.configPeakOutputReverse(-armMaxOutput);

    // set current limit
    StatorCurrentLimitConfiguration currentConfig = 
        new StatorCurrentLimitConfiguration(true, 30, 
        32, .1);
    armMotor.configStatorCurrentLimit(currentConfig);

    armMotor.setStatusFramePeriod(StatusFrame.Status_10_Targets, 5);
    armMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 5);

    SmartDashboard.putNumber("arm kP", armkP);
    SmartDashboard.putNumber("arm kI", armkI);
    SmartDashboard.putNumber("arm kD", armkD);
    SmartDashboard.putNumber("arm kF", armkF);
    SmartDashboard.putNumber("arm kIzone", armkIZone);

    SmartDashboard.putNumber("arm kP down", armkPdown);
    SmartDashboard.putNumber("arm kI down ", armkIdown);
    SmartDashboard.putNumber("arm kD down", armkDdown);
    SmartDashboard.putNumber("arm kF down", armkFdown);
    SmartDashboard.putNumber("arm kIzone down", armkIZonedown);

    SmartDashboard.putNumber("arm kG", armkG);
    SmartDashboard.putNumber("arm kS", armkS);
    SmartDashboard.putNumber("arm kV", armkV);
    SmartDashboard.putNumber("arm kA", armkA);

    SmartDashboard.putNumber("arm kG2", armkG2);
    SmartDashboard.putNumber("arm kS2", armkS2);
    SmartDashboard.putNumber("arm kV2", armkV2);
    SmartDashboard.putNumber("arm kA2", armkA2);

    SmartDashboard.putNumber("arm kG3", armkG3);
    SmartDashboard.putNumber("arm kS3", armkS3);
    SmartDashboard.putNumber("arm kV3", armkV3);
    SmartDashboard.putNumber("arm kA3", armkA3);


    SmartDashboard.putNumber("arm pos1", armPos1);
    SmartDashboard.putNumber("arm pos2", armPos2);
    SmartDashboard.putNumber("arm pos3", armPos3);
    SmartDashboard.putNumber("arm MMvel", armMMVel);
    SmartDashboard.putNumber("arm MMacc", armMMAcc);
    SmartDashboard.putNumber("arm ForSensorLim", armForwardSensorLim);
    SmartDashboard.putNumber("arm RevSensorLim", armReverseSensorLim);
    SmartDashboard.putNumber("arm MaxOutput", armMaxOutput);

    SmartDashboard.putBoolean("logging", false);

    updateConstants();

  }

  @Override
  public void robotPeriodic() {
    ls_upper = limitUpper.get();
    ls_lower = limitLower.get();

    stick = joy.getRawAxis(0);
    if (Math.abs(stick) < 0.02)
      stick = 0;
    isUpperLimitActive(stick);

    time = Timer.getFPGATimestamp();

    voltage = armMotor.getMotorOutputVoltage();
    angle = enc.getPosition();
    angleCounts = armMotor.getSelectedSensorPosition();
    angleRate = armMotor.getSelectedSensorVelocity();

    angleFromHorizontalDeg = angle - angleOffsetDeg;
    angleFromHorizontalCounts = angleCounts - angleOffsetCounts;
    angleFromHorizontalRad = angleFromHorizontalDeg * Math.PI / 180.;
    cosAngleFromHorizontal = Math.cos(angleFromHorizontalDeg * Math.PI / 180.);

    angleRate2 = (angleRate - angleRatePrev) / (time - timePrev);

    //System.out.println(time+"  "+(time-timePrev)+"   "+angleRate+"   "+angleRatePrev);

    angleRatePrev = angleRate;
    timePrev = time;


    SmartDashboard.putNumber("arm Pos Counts", angleCounts);
    SmartDashboard.putNumber("arm Pos Deg", angle);
    SmartDashboard.putNumber("arm Cos(Pos)", cosAngleFromHorizontal);

    SmartDashboard.putNumber("arm angle from Horiz Deg", angleFromHorizontalDeg);
    SmartDashboard.putNumber("arm angle from Horiz Counts", angleFromHorizontalCounts);
    SmartDashboard.putNumber("arm angle rate", angleRate);
    SmartDashboard.putNumber("arm angle rate2", angleRate2);

    SmartDashboard.putNumber("arm voltage", voltage);
    SmartDashboard.putNumber("arm current", armMotor.getStatorCurrent());
    SmartDashboard.putNumber("arm CLE", armMotor.getClosedLoopError());
    SmartDashboard.putBoolean("arm Upper LS", ls_upper);
    SmartDashboard.putBoolean("arm Lower LS", ls_lower);
    logging = SmartDashboard.getBoolean("logging", false);

//    if (logging) logData();
  }


  @Override
  public void autonomousInit() {
      autoFlagA=false;autoFlagB=false;voltageauto=-0.04;
      armMotor.set(ControlMode.PercentOutput,voltageauto);
      Timer.delay(2);
    }

    @Override
    public void autonomousPeriodic() {
        
        if (angleCounts>2600) autoFlagA=true;
        if (angleCounts<1100) autoFlagB=true;

        if (angleRate==0 && angleRate2==0 && Timer.getFPGATimestamp()-autoTime>2){
          logData(); 
          if (!autoFlagA) voltageauto+=0.0005;
          if (autoFlagA && !autoFlagB) voltageauto -= 0.0005;
          if (autoFlagB) voltageauto=0;
          autoTime=Timer.getFPGATimestamp();          
        } 
//        System.out.println("FlagA = "+autoFlagA+"    FlagB = "+autoFlagB+"   voltAuto = "+voltageauto);
        armMotor.set(ControlMode.PercentOutput,voltageauto);
        SmartDashboard.putNumber("arm VoltAuto", voltageauto);
      }




  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    double arbff;
    if (angleFromHorizontalDeg < -69)
      arbff = +armkG2 * Math.cos(angleFromHorizontalRad) + armkS2;
    else if (angleFromHorizontalDeg >= -69 && angleFromHorizontalDeg<0)
      arbff = +armkG * Math.cos(angleFromHorizontalRad) + armkS;
    else
      arbff = +armkG3 * Math.cos(angleFromHorizontalRad) + armkS3;


    if (joy.getRawButton(1)) {
      if(button !=1){
        button=1;
        armMotor.setIntegralAccumulator(0, 0,20);
      }
//      armMotor.set(ControlMode.MotionMagic, armPos1, DemandType.ArbitraryFeedForward, arbff);
armMotor.set(ControlMode.Position, armPos1, DemandType.ArbitraryFeedForward, arbff);
    }

    else if (joy.getRawButton(2)) {
     if(button !=2){
      button=2;
      armMotor.setIntegralAccumulator(0, 0,20);
    }
//    armMotor.set(ControlMode.MotionMagic, armPos2, DemandType.ArbitraryFeedForward, arbff);
armMotor.set(ControlMode.Position, armPos2, DemandType.ArbitraryFeedForward, arbff);
    }

    else if (joy.getRawButton(3)) {       
      if(button !=3){
        button=3;
        armMotor.setIntegralAccumulator(0, 0,20);
      }
//      armMotor.set(ControlMode.MotionMagic, armPos3, DemandType.ArbitraryFeedForward, arbff);
armMotor.set(ControlMode.Position, armPos3, DemandType.ArbitraryFeedForward, arbff);
    }

    else if (joy.getRawButton(4)) {
      updateConstants();
    }

    else {
      // if (!ls_upperActive)
      double demand = (stick * .2) + arbff;
      armMotor.set(ControlMode.PercentOutput, demand);
    }

  }

  public void updateConstants() {
    armkG = SmartDashboard.getNumber("arm kG", 0);
    armkS=SmartDashboard.getNumber("arm kS", 0);

    armkG2 = SmartDashboard.getNumber("arm kG2`", 0);
    armkS2=SmartDashboard.getNumber("arm kS2", 0);

    armkG3 = SmartDashboard.getNumber("arm kG3`", 0);
    armkS3=SmartDashboard.getNumber("arm kS3", 0);


    armkP = SmartDashboard.getNumber("arm kP", 0);
    armkI = SmartDashboard.getNumber("arm kI", 0);
    armkD = SmartDashboard.getNumber("arm kD", 0);
    armkF = SmartDashboard.getNumber("arm kF", 0);
    armkIZone = SmartDashboard.getNumber("arm kIzone", 0);

    armkPdown = SmartDashboard.getNumber("arm kP down", 0);
    armkIdown = SmartDashboard.getNumber("arm kI down", 0);
    armkDdown = SmartDashboard.getNumber("arm kD down", 0);
    armkFdown = SmartDashboard.getNumber("arm kF down", 0);
    armkIZonedown = SmartDashboard.getNumber("arm kIzone down", 0);

    armMotor.config_kP(0, armkP);
    armMotor.config_kI(0, armkI);
    armMotor.config_kD(0, armkD);
    armMotor.config_kF(0, armkF);
    armMotor.config_IntegralZone(0, armkIZone);

    armMotor.config_kP(1, armkPdown);
    armMotor.config_kI(1, armkIdown);
    armMotor.config_kD(1, armkDdown);
    armMotor.config_kF(1, armkFdown);
    armMotor.config_IntegralZone(1, armkIZonedown);

    armMMVel = SmartDashboard.getNumber("arm MMvel", 0);
    armMMAcc = SmartDashboard.getNumber("arm MMacc", 0);
    armMotor.configMotionCruiseVelocity(armMMVel);
    armMotor.configMotionAcceleration(armMMAcc);

    armForwardSensorLim = SmartDashboard.getNumber("arm ForSensorLim", 0);
    armReverseSensorLim = SmartDashboard.getNumber("arm RevSensorLim", 0);
    armMotor.configForwardSoftLimitThreshold(armForwardSensorLim);
    armMotor.configReverseSoftLimitThreshold(armReverseSensorLim);

    armMaxOutput = SmartDashboard.getNumber("arm MaxOutput", 0);
    armMotor.configClosedLoopPeakOutput(0, armMaxOutput);
    armMotor.configPeakOutputForward(armMaxOutput);
    armMotor.configPeakOutputReverse(-armMaxOutput);

    armPos1 = SmartDashboard.getNumber("arm pos1", 0);
    armPos2 = SmartDashboard.getNumber("arm pos2", 0);
    armPos3 = SmartDashboard.getNumber("arm pos3", 0);

  }

  public void isUpperLimitActive(double stick) {
    if (ls_upper)
      ls_upperActive = true;
    else if (ls_upperActive && stick > 0)
      ls_upperActive = true;
    else
      ls_upperActive = false;
  }

  public void logData(){
    try {
      String txt = Double.toString(time) + "," +
          Double.toString(voltageauto) + "," +
          Double.toString(angleCounts) +","+
          Double.toString(angleRate) + ","+ 
          Double.toString(angleRate2) + ","+
          "\n";
      writer.write(txt);
    } catch (IOException e) {
      System.out.println("An error occurred.");
      e.printStackTrace();
    }

  }


}

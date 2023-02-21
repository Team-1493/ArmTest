
package frc.robot;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Robot extends TimedRobot {
  
  TalonFX armMotor=new TalonFX(9);
  CANCoder enc=new CANCoder(19);
  DigitalInput limitUpper = new DigitalInput(0);
  DigitalInput limitLower = new DigitalInput(1);
  Joystick joy = new Joystick(0);
  ArmFeedforward feedFwd;
  double absPos,relPos;
  double armkP=0,armkD=0,armkI=0,armkIZone,armkF=0;
  double armkG=0,armkS=0,armkV=0,armkA=0;
  double armPos1=1300,armPos2=1600,armPos3=2200;
  double armMMVel=1,armMMAcc=1;
  double armForwardSensorLim=2500,armReverseSensorLim=1200;
  double armMaxOutput=.4;

  double angle,angleCounts;
  double angleFromHorizontalDeg,angleFromHorizontalCounts,angleFromHorizontalRad;
  double cosAngleFromHorizontal;
  double angleOffsetDeg=150;
  double angleOffsetCounts=angleOffsetDeg*4096/360.;
  double angleRate,angleRatePrev=0,angleRate2;
  double voltage;
  double time=0,timePrev=0;

  double stick;
  boolean logging=false,loggingPrev=false;
  boolean ls_upper=true,ls_lower=false,ls_upperActive=false;
  DoubleLogEntry voltageLog,angleLog,angleRateLog,angleRate2Log,cosAngleLog,signAngleRateLog;
  DataLog log;
  FileWriter writer;


  public Robot() {
    super(0.005);
    LiveWindow.disableAllTelemetry();
}


  @Override
  public void robotInit() {

/*     DataLogManager.start();
    log = DataLogManager.getLog();

    voltageLog = new DoubleLogEntry(log, "voltage");
    angleLog = new DoubleLogEntry(log, "angle");
    angleRateLog = new DoubleLogEntry(log, "angleRate");
    angleRate2Log = new DoubleLogEntry(log, "angleRate2");
    cosAngleLog = new DoubleLogEntry(log, "cosAngle");
    signAngleRateLog = new DoubleLogEntry(log, "signAngleRate");
*/

    try {
      String fileName = "data"+new SimpleDateFormat("yyyyMMddhhmm'.txt'").format(new Date());
      writer = new FileWriter(fileName);
    } catch (IOException e) {
      System.out.println("An error occurred.");
      e.printStackTrace();
    }


    enc.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
    enc.setPositionToAbsolute(20);
    enc.configSensorDirection(true);
    enc.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 10, 20);
    
    armMotor.configFactoryDefault();
    
    // set voltage compensation
    armMotor.configVoltageCompSaturation(11.5);
    armMotor.enableVoltageCompensation(true);

  //  Set encoder values as limit switch  
    armMotor.configForwardSoftLimitThreshold(armForwardSensorLim);
    armMotor.configReverseSoftLimitThreshold(armReverseSensorLim);
    armMotor.configForwardSoftLimitEnable(true);
    armMotor.configReverseSoftLimitEnable(true);
    
  // set to brake mode
    armMotor.setNeutralMode(NeutralMode.Coast);

  // set feedback sensor to cancoder
    armMotor.configRemoteFeedbackFilter(enc, 0);
    armMotor.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0  , 0, 20);
  
 // set max closed loop output
    armMotor.configClosedLoopPeakOutput(0, armMaxOutput);   
    armMotor.configPeakOutputForward(armMaxOutput);
    armMotor.configPeakOutputReverse(-armMaxOutput);


   // set current limit 
    StatorCurrentLimitConfiguration currentConfig = 
        new StatorCurrentLimitConfiguration(true, 20, 25, 1);
    //armMotor.configGetStatorCurrentLimit(currentConfig)  ;

    armMotor.setStatusFramePeriod(StatusFrame.Status_10_Targets, 10);
    armMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0,10);

    SmartDashboard.putNumber("arm kP", armkP);
    SmartDashboard.putNumber("arm kI", armkI);
    SmartDashboard.putNumber("arm kD", armkD);
    SmartDashboard.putNumber("arm kF", armkF);
    SmartDashboard.putNumber("arm kIzone", armkIZone);
    SmartDashboard.putNumber("arm kG", armkG);
    SmartDashboard.putNumber("arm kS", armkS);
    SmartDashboard.putNumber("arm kV", armkV);
    SmartDashboard.putNumber("arm kA", armkA);
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
    ls_upper=limitUpper.get();
    ls_lower=limitLower.get();

    stick = joy.getRawAxis(0);    
    if (Math.abs(stick)<0.05)stick=0;
    isUpperLimitActive(stick);

    time=Timer.getFPGATimestamp();
  
    voltage=armMotor.getMotorOutputVoltage();
    angle=enc.getPosition();
    angleCounts=armMotor.getSelectedSensorPosition();
    angleRate=armMotor.getSelectedSensorVelocity();

    angleFromHorizontalDeg=angle-angleOffsetDeg;
    angleFromHorizontalCounts=angleCounts-angleOffsetCounts;
    angleFromHorizontalRad=angleFromHorizontalDeg*Math.PI/180.;
    cosAngleFromHorizontal=Math.cos(angleFromHorizontalDeg*Math.PI/180.);

    angleRate2=(angleRate-angleRatePrev)/(time-timePrev);

    angleRatePrev=angleRate;
    timePrev=time;

    SmartDashboard.putNumber("arm Pos Counts", angleCounts);
    SmartDashboard.putNumber("arm Pos Deg", angle);
    SmartDashboard.putNumber("arm Cos(Pos)", cosAngleFromHorizontal);

    SmartDashboard.putNumber("arm angle from Horiz Deg",angleFromHorizontalDeg);
    SmartDashboard.putNumber("arm angle from Horiz Counts",angleFromHorizontalCounts);
    SmartDashboard.putNumber("arm angle rate",angleRate);
    SmartDashboard.putNumber("arm angle rate2",angleRate2);

    SmartDashboard.putNumber("arm voltage", voltage);
    SmartDashboard.putNumber("arm current", armMotor.getStatorCurrent());
    SmartDashboard.putNumber("arm current", armMotor.getStatorCurrent());
    SmartDashboard.putNumber("arm CLE", armMotor.getClosedLoopError());  
    SmartDashboard.putBoolean("arm Upper LS",ls_upper);  
    SmartDashboard.putBoolean("arm Lower LS",ls_lower);  


    logging=SmartDashboard.getBoolean("logging", false);

    /* 
  // toggle logging on and off
    if (logging && !loggingPrev) log.resume();
    if(!logging && loggingPrev) log.pause();
    loggingPrev=logging;


// append to log , takes no action if logging is paused
  voltageLog.append(voltage);
  angleLog.append(angleCounts);
  cosAngleLog.append(cosAngleFromHorizontal);
  signAngleRateLog.append(Math.signum(angleRate));
  angleRateLog.append(angleRate);
  angleRate2Log.append(angleRate2);
*/


if(logging){
  try {
   String txt=Double.toString(time)+","+Double.toString(angle)+","+
      Double.toString(angleCounts)+"\n";
    writer.write(txt);
  } catch (IOException e) {
    System.out.println("An error occurred.");
    e.printStackTrace();
  }
  }
}

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    if(joy.getRawButton(1)){
//      armMotor.set(ControlMode.MotionMagic,pos1,
//      DemandType.ArbitraryFeedForward,feedFwd.calculate(angleFromHorizontalRad, 0));
    }
    else if(joy.getRawButton(2)){
//      armMotor.set(ControlMode.MotionMagic,pos2,
//      DemandType.ArbitraryFeedForward,feedFwd.calculate(angleFromHorizontalRad, 0));
    }
    else if(joy.getRawButton(4)){
      updateConstants();
    }
    else{
        double demand = 0.2*stick+getFeedForward();
//        if (!ls_upperActive) armMotor.set(ControlMode.PercentOutput, demand);
//        if (!ls_upperActive) armMotor.set(ControlMode.PercentOutput, demand
//        ,DemandType.ArbitraryFeedForward,feedFwd.calculate(angleFromHorizontalRad, 0));

    }
  }


  public void updateConstants(){
    feedFwd=new ArmFeedforward(
        SmartDashboard.getNumber("arm kS",0),
        SmartDashboard.getNumber("arm kG",0),
        SmartDashboard.getNumber("arm kV",0),
        SmartDashboard.getNumber("arm kA",0)
        );

    armMotor.config_kP(0,SmartDashboard.getNumber("arm kP", 0));
    armMotor.config_kI(0,SmartDashboard.getNumber("arm kI", 0));
    armMotor.config_kD(0,SmartDashboard.getNumber("arm kD", 0));
    armMotor.config_kF(0,SmartDashboard.getNumber("arm kF", 0));
    armMotor.config_IntegralZone(
        0,SmartDashboard.getNumber("arm kIzone", 0));

    armMMVel=SmartDashboard.getNumber("arm MMvel", 0);
    armMMAcc=SmartDashboard.getNumber("arm MMacc", 0);  
    armMotor.configMotionCruiseVelocity(armMMVel);
    armMotor.configMotionAcceleration(armMMAcc);
    
    armForwardSensorLim = SmartDashboard.getNumber("arm ForSensorLim",0 );
    armReverseSensorLim = SmartDashboard.getNumber("arm RevSensorLim", 0);
    armMotor.configForwardSoftLimitThreshold(armForwardSensorLim);
    armMotor.configReverseSoftLimitThreshold(armReverseSensorLim);

    armMaxOutput=SmartDashboard.getNumber("arm MaxOutput",0);
    armMotor.configClosedLoopPeakOutput(0, armMaxOutput);
    armMotor.configPeakOutputForward(armMaxOutput);
    armMotor.configPeakOutputReverse(-armMaxOutput);

    armPos1=SmartDashboard.getNumber("arm pos1", 0);
    armPos2=SmartDashboard.getNumber("arm pos2", 0);
    armPos3=SmartDashboard.getNumber("arm pos3", 0);

  }

  public void  isUpperLimitActive(double stick){
    if (ls_upper) ls_upperActive=true;
    else if (ls_upperActive && stick>0) ls_upperActive=true;
    else ls_upperActive=false;    
  }

  public double getFeedForward(){
    return feedFwd.calculate(angleFromHorizontalDeg*Math.PI/180, 0);
  }
  
 
}

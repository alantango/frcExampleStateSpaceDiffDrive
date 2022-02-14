// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * This is a sample program to demonstrate the use of state-space classes in robot simulation. This
 * robot has a flywheel, elevator, arm and differential drivetrain, and interfaces with the sim
 * GUI's {@link edu.wpi.first.wpilibj.simulation.Field2d} class.
 */
public class Robot extends TimedRobot {
  private RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    // Flush NetworkTables every loop. This ensures that robot pose and other values
    // are sent during every loop iteration.
    setNetworkTablesFlushEnabled(true);
    log("__robotInit()");
  }

  @Override
  public void simulationPeriodic() {
    // Here we calculate the battery voltage based on drawn current.
    // As our robot draws more power from the battery its voltage drops.
    // The estimated voltage is highly dependent on the battery's internal
    // resistance.
    double drawCurrent = m_robotContainer.getRobotDrive().getDrawnCurrentAmps();
    double loadedVoltage = BatterySim.calculateDefaultBatteryLoadedVoltage(drawCurrent);
    RoboRioSim.setVInVoltage(loadedVoltage);
    log("__robot SIM periodic");
  }

  @Override
  public void robotPeriodic() {
    log("__robot periodic");
    CommandScheduler.getInstance().run();
  }

  @Override
  public void autonomousInit() {
    m_robotContainer.getAutonomousCommand().schedule();
    // m_robotContainer.getDummyAutoCommand().schedule();
    Robot.log("--- auto Init");
  }

  @Override
  public void autonomousPeriodic(){
    Robot.log("--- auto periodic");
  }

  @Override
  public void disabledInit() {
    CommandScheduler.getInstance().cancelAll();
    m_robotContainer.zeroAllOutputs();
  }


  /* simple console logging */
  private static DateTimeFormatter df=DateTimeFormatter.ofPattern("HH:mm:ss");
  private static Map<String, String> tick=new HashMap<>();
  public static void log(String s){
    log(s, 3);
  }
  public static void log(String s, int increment){
    LocalDateTime ts = LocalDateTime.now();
    String xts = ts.format(df);
    String cts = tick.get(s);
    if(cts==null ){
      tick.put(s, cts);
    }
    if(ts.getSecond()%increment == 0 && !xts.equals(cts)){
        System.out.println( "<" + xts + "> " + s);
        tick.put(s, xts);
    }
  }

}

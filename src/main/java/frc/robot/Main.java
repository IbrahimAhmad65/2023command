// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import edu.wpi.first.wpilibj.RobotBase;
import frc.PoseEstimado;

public final class Main {
  private Main() {}

  public static void main(String... args) {
    ExecutorService executor = Executors.newFixedThreadPool(1);
    executor.execute(new PoseEstimado());

    RobotBase.startRobot(Robot::new);
  }
}

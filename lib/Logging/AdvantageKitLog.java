// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team696.lib.Logging;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.BuildConstants;

/** Add your docs here. */
public class AdvantageKitLog {
    public static void setup() {
        Logger.recordMetadata("ProjectName", "2024OffSeason");
        Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
        switch (BuildConstants.DIRTY) {
          case 0:
            Logger.recordMetadata("GitDirty", "All changes committed");
            break;
          case 1:
            Logger.recordMetadata("GitDirty", "Uncomitted changes");
            break;
          default:
            Logger.recordMetadata("GitDirty", "Unknown");
            break;
        }

        Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
        if (RobotBase.isReal()) {
            Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
        }

        Logger.start();
    }
}

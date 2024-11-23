package frc.team696.lib;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team696.lib.Dashboards.ShuffleDashboard;
import frc.team696.lib.Logging.PLog;
import frc.team696.lib.Swerve.SwerveDriveSubsystem;

/**
 * Houses all methods related to the Autonomous period and self driving during teleop
 */
public class Auto {
    /**
     * Wrapper For Pathplanners NamedCommand System
     */
    public static class NamedCommand {
        public String name;
        public Command command;

        /**
         * @param name to be used inside of the pathplanner GUI
         * @param command to be run when called inside pathplanner
         */
        public NamedCommand(String name, Command command) {
            this.name = name;
            this.command = command;
        }

        public void register() {
            NamedCommands.registerCommand(name, command);
        }
    }

    public static Auto m_instance;
    private SwerveDriveSubsystem _swerve;

    private SendableChooser<Command> autoChooser;

    private StringPublisher chooserChanger;

    private Auto (SwerveDriveSubsystem swerve, NamedCommand... commandsToRegister) {
        _swerve = swerve;
        
        RobotConfig config;
        try{
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            // Handle exception as needed
            PLog.fatalException("Auto", "Failed To Get RobotConfig", e); 
            return;
        }

        AutoBuilder.configure(
            _swerve::getPose, 
            _swerve::resetPose, 
            _swerve::getRobotRelativeSpeeds,
            (speeds, feedforwards) -> _swerve.Drive(speeds), 
            new PPHolonomicDriveController(  
                    new PIDConstants(5.0, 0.0, 0.0), 
                    new PIDConstants(5.0, 0.0, 0.0)),
            config,
            () -> {
                Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            },
            _swerve
        );

        for (NamedCommand namedCommand : commandsToRegister) {
            namedCommand.register();
        }

        PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
            Logger.recordOutput("696/Auto/Desired", pose);
        });

        PathPlannerLogging.setLogActivePathCallback((poses) -> {
            ShuffleDashboard.field.getObject("Path").setPoses(poses);
        });

        autoChooser = AutoBuilder.buildAutoChooser();
        ShuffleDashboard.addAutoChooser(autoChooser);

        chooserChanger = NetworkTableInstance.getDefault().getStringTopic("Shuffleboard/Telemetry/Autos/selected").publish();

        ShuffleDashboard.addObject("ChooseNearest (Doesn't Work)", Commands.runOnce(()->chooserChanger.accept(ClosestName())).ignoringDisable(true)).withPosition(7, 1).withSize(2,1);

        autoChooser.onChange((command)-> {
            visualize();
        });
    }

    /**
     * Initializes all things related to auto (ex. pathplanner)
     * 
     * @param swerve The swerve subsystem
     * @param commandsToRegister each command to register for path planner
     */
    public static void Initialize(SwerveDriveSubsystem swerve, NamedCommand... commandsToRegister){
        if (m_instance != null) throw new RuntimeException ("Don't Initialize Twice!");
        
        m_instance = new Auto(swerve, commandsToRegister);
    }

    /**
     * @return The instance Of the Auto Class
     */
    public static Auto get(){
        if (m_instance == null) throw new RuntimeException ("Please Initialize First!");

        return m_instance;
    }

    /**
     * @return Selected Command From SendableChooser
     */
    public Command Selected() {
        return m_instance.autoChooser.getSelected();
    }

    /**
     * @return Selected Command From SendableChooser Which Will End After 15 Seconds.
     *              Used for testing on home field
     */
    public Command SelectedEndAt15() {
        return Selected().raceWith(new WaitCommand(15.0));
    }

    public String ClosestName() {
        List<String> autoNames = AutoBuilder.getAllAutoNames();
        double minDist = 100;
        String closestName = "None";
        for (int i = 0; i < autoNames.size(); i++) {
            String autoName = autoNames.get(i);
            try {
                List<PathPlannerPath> poses = PathPlannerAuto.getPathGroupFromAutoFile(autoName);
                if (poses.size() == 0) continue;
                Pose2d startingPose = poses.get(0).getStartingDifferentialPose();
                Translation2d autoStartingPose = startingPose.getTranslation();
                double dist = _swerve.getPose().getTranslation().getDistance(autoStartingPose);
                PLog.info("Test", dist);
                if (dist < minDist) {
                    minDist = dist;
                    closestName = autoName;
                }
            } catch (Exception e) {
                PLog.fatalException("Auto", autoName, e);
            } 
        }

        return closestName;
    }

    public Command ClosestCommand() {
        if (ClosestName().compareTo("None") == 0) return new WaitCommand(0);

        return AutoBuilder.buildAuto(ClosestName());
    }

    public static Command PathFind(Pose2d end) {
        return AutoBuilder.pathfindToPose(end, new PathConstraints(1, 1, Math.PI,Math.PI));
    }

    public Command PathFindToAutoBeginning() {
        Pose2d initialPose;
        try {
            List<PathPlannerPath> paths = PathPlannerAuto.getPathGroupFromAutoFile(autoChooser.getSelected().getName());

            if (paths.size() <= 0) return new WaitCommand(0);

            initialPose = paths.get(0).getStartingDifferentialPose();
        } catch (Exception e) {
            PLog.fatalException("Auto", "Failed To Find Path", e);
            return new WaitCommand(0);
        }

        return AutoBuilder.pathfindToPose(initialPose, new PathConstraints(1, 1, Math.PI, Math.PI));
    }

    public void visualize(String name) {
        List<PathPlannerPath> paths;
        List<Pose2d> pathPoses = new ArrayList<Pose2d>();
        try {
            paths = PathPlannerAuto.getPathGroupFromAutoFile(name);
        } catch (Exception e) {
            PLog.fatalException("Auto", "Failed To Find Path", e);
            return;
        }
        for (int i = 0; i < paths.size(); i++) {
            PathPlannerPath path = paths.get(i);
            if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red)
                path = path.flipPath();
            pathPoses.addAll(path.getPathPoses());
        }
        ShuffleDashboard.field.getObject("traj").setPoses(pathPoses);
    }

    public void visualize() {
        visualize(autoChooser.getSelected().getName());
    }
}
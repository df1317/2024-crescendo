package frc.robot.subsystems;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AutoSubsystem extends SubsystemBase {

    public final SendableChooser<Autos> autoChooser = new SendableChooser<>();

    public enum Autos {
        A_Leave,
        B_Leave,
        C_Leave,
        A_Amp,
        A_2Amp,
        A_Speaker,
        A_2Speaker,
        B_Speaker,
        B_2Speaker,
        B_3Speaker,
        C_Speaker,
        C_2Speaker,
        C_StealCenterNotes
    }

    PathPlannerPath A_Leave = PathPlannerPath.fromPathFile("A_Leave.auto");
    PathPlannerPath A_Amp = PathPlannerPath.fromPathFile("A_Amp.auto");
    PathPlannerPath A_2Amp = PathPlannerPath.fromPathFile("A_2Amp.auto");
    PathPlannerPath A_Speaker = PathPlannerPath.fromPathFile("A_Speaker.auto");
    PathPlannerPath A_2Speaker = PathPlannerPath.fromPathFile("A_2Speaker.auto");
    PathPlannerPath B_Leave = PathPlannerPath.fromPathFile("B_Leave.auto");
    PathPlannerPath B_Speaker = PathPlannerPath.fromPathFile("B_Speaker.auto");
    PathPlannerPath B_2Speaker = PathPlannerPath.fromPathFile("B_2Speaker.auto");
    PathPlannerPath B_3Speaker = PathPlannerPath.fromPathFile("B_3Speaker.auto");
    PathPlannerPath C_Leave = PathPlannerPath.fromPathFile("C_Leave.auto");
    PathPlannerPath C_Speaker = PathPlannerPath.fromPathFile("C_Speaker.auto");
    PathPlannerPath C_2Speaker = PathPlannerPath.fromPathFile("C_2Speaker.auto");
    PathPlannerPath C_StealCenterNotes = PathPlannerPath.fromPathFile("C_StealCenterNotes.auto");

    // Possible other way to call our autos??
    // PathPlanner A_Leave =
    // PathPlannerAuto.getPathGroupFromAutoFile("A_Leave.auto");

    private Autos selectedAuto;// making a selected auto for later use

    public AutoSubsystem() {
        autoChooser.setDefaultOption("A_Leave", Autos.A_Leave);
        for (Autos autoChoice : Autos.values()) {
            // going through all options and putting them in a drop down in the driver
            // station
            autoChooser.addOption(autoChoice.name(), autoChoice);
        }
    }

    public Autos getSelectedAuto() {
        selectedAuto = autoChooser.getSelected();
        return selectedAuto;
    }

}
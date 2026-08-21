// Create this new file in: org.firstinspires.ftc.teamcode.commands
package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

import java.util.Locale;

/**
 * A command that waits for the Turret to be on target and settled.
 * It includes a timeout to prevent getting stuck if the target is never acquired.
 */
public class WaitForTurretAimCommand extends CommandBase {

//    private Turret turret;
    private final ElapsedTime timer = new ElapsedTime();
    private final double timeoutSeconds;

    /**
     * Constructor for the command.
     * @param timeoutInSeconds The maximum time to wait for the turret to aim.
     */
    public WaitForTurretAimCommand(double timeoutInSeconds) {
        this.timeoutSeconds = timeoutInSeconds;
    }

    /**
     * Default constructor with a 2-second timeout.
     */
    public WaitForTurretAimCommand() {
        this(2.0); // Default to a 2-second timeout
    }

    @Override
    public void start(Drivetrain drivetrain, Intake intake) {
//        this.turret = turret;
        timer.reset();
        log(getName(), String.format(Locale.US, "Now waiting for turret to aim and settle (Timeout: %.1fs).", timeoutSeconds));
    }

    @Override
    public void update() {
        // Nothing to do in the update loop, we are just waiting.
    }

    @Override
    public boolean isFinished() {
//        if (turret == null) {
//            return true; // Failsafe
//        }

        // The command is finished if either:
        // 1. The turret is on target and has settled.
        // 2. The timeout has been reached.
        return  timer.seconds() > timeoutSeconds;
    }

    @Override
    public void end() {
//        if (turret != null && turret.isOnTargetAndSettled()) {
//            log(getName(), "Turret is aimed and settled. Proceeding.");
//        } else {
//            log(getName(), String.format(Locale.US, "Timed out after %.1fs waiting for turret. Proceeding anyway.", timeoutSeconds));
//        }
    }
}

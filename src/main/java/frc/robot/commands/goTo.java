package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

public class goTo extends Command{
    private Drive drive;
    private Pose2d goalPos;
    private ChassisSpeeds goalSpeeds;
    private ChassisSpeeds startingSpeeds;
    private Timer timer;

    public goTo(Drive drive, Pose2d goalPos)
    {
        this.drive = drive;
        this.goalPos = goalPos;
        this.goalSpeeds = new ChassisSpeeds();
    }

    public goTo(Drive drive, Pose2d goalPos, ChassisSpeeds goalSpeeds)
    {
        this.drive = drive;
        this.goalPos = goalPos;
        this.goalSpeeds = goalSpeeds;
    }

    @Override
    public void initialize()
    {
        this.startingSpeeds = drive.getChassisSpeeds();
        timer = new Timer();
        timer.start();
    }

    @Override 
    public void execute()
    {
        double xa =;
        double vx = this.startingSpeeds.vxMetersPerSecond + 
        double vy = 
        double vr = 

        drive.runVelocity(vx, vy, vr);
    }
}

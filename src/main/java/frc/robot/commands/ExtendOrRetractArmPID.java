// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.
// // DEPRECTATED: This functionality is now built into the Arm Extension PID subsystem
// package frc.robot.commands;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj2.command.PIDCommand;
// import frc.robot.subsystems.ArmPIDSubsystem;
// import frc.robot.subsystems.ArmExtensionPID;


// // NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// // information, see:
// // https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
// public class ExtendOrRetractArmPID extends PIDCommand {
//   /** Creates a new ExtendOrRetractArmPID. */
//   public ExtendOrRetractArmPID(double distance, ArmExtensionPID m_telescope) {
//     super(
//         // The controller that the command will use
//         new PIDController(.01, 0, 0),
//         // This should return the measurement
//         () -> m_telescope.getArmExtensionInches(),
//         // This should return the setpoint (can also be a constant)
//         () -> distance,
//         // This uses the output
//         output -> {
//           // Use the output here
//           m_telescope.telescope(output);
//         });
//     // Use addRequirements() here to declare subsystem dependencies.
//     // Configure additional PID options by calling `getController` here.
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }

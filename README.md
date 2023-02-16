# CommandBased_2023
FRC Robot 2023 Java Code: Command-Based framework

### Subsystems

#### DriveSubsystem
* SparkMax controlled 4-Motor (2 Leaders/2 Followers) [RevLib Vendor Library](https://software-metadata.revrobotics.com/REVLib-2023.json)
* Kauai Labs NavX2 IMU - [Kaui Labs NavX Vendor Library](https://dev.studica.com/releases/2023/NavX.json) 

#### Arm PID Rotation Subsystem
* 2x SparkMax controlled brushless Neo motors, leader/follower (inverted) for rotation
#### Arm PID Extension Subsystem
* 1x SparkMax controlled Neo motor for extension

#### Claw
* Pneumatic Actuator for open/close
* 1x CANbus/Talon controlled for intake/eject gamepiece
* [CTRE CANTalon Vendor Library](https://maven.ctr-electronics.com/release/com/ctre/phoenix/Phoenix5-frc2023-latest.json)

#### GroundIntake
* Pneumatic controlled extend/retract
* SparkMax controlled intake/eject motor

#### SignalLED Subsystem
* PWM controlled LED strips: One or two combination or independently controlled: 
**Signaling gamepiece to human planer
**Indicating phase of game
**Countdown timer during endgame

### Vision Subsystems
#### PhotonVision
* Raspberry Pi two-camera AprilTag tracking and location detection

#### LimeLight
* Off-the-shelf AprilTag tracking
### Commands
* AprilTagAlign: Auto-assist to align with Nodes
* AutoBalancePID: Balance based on "roll" axis (NavX Orientation is sideways)
* AutoDriveSequential: Sequence of drive distance & turning commands
* DriveOnAndBalanceChargingStation: Autonomous sequential command
* DriveToChargeStation: Autonomous mode drives forward until pitch angle of charging station is detected
* DriveToDistance: Drive an arbitrary distance
* DriveToDistancePID: Unused
* SequentialVisionAlign: Sequential command, enables the Limelight then runs VisionAlign 
* TurnToAngle(angle): Unused
* TurnToAngleProfiled(angle): Turn to an arbitrary angle (PID)
* VisionAlign: Uses Limelight targeting to adjust Z-rotation (left/right turning) of drive subsystem

### Command Groups
* AutoDriveSequential: DriveToDistance (onto ramp); AutoBalancePID

## Gradle Note:

In order to capture some compile-time data, we have modified the build.gradle file. Wpilib season updates may overwrite build.gradle. If this happens, you'll need to add the following to the end:

```
task writeBuildProperties() {
	outputs.upToDateWhen { false }  // never mark as up-to-date to ensure that this gets run every time


	doLast {
        def propFile = new File("${project.rootDir}/src/main/resources/build.properties")

        def prop = new Properties()

        prop.setProperty('build.time', (new Date()).format("yyyy_MM_dd HH:mm:ss"))
        prop.setProperty('build.computer', InetAddress.getLocalHost().getHostName())

        propFile.createNewFile();
        prop.store(propFile.newWriter(), null);

        println("task ran")
    }
}

processResources {
	outputs.upToDateWhen { false }  // never mark as up-to-date to ensure that this gets run every time
	dependsOn("writeBuildProperties")
}

```

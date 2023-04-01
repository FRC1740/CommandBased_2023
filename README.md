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
* [Playing With Fusion Vendor Library](https://www.playingwithfusion.com/frc/playingwithfusion2023.json)

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

### Paths
* McDouble: Score one piece high, score one low
* McTriple: Score one piece high, score two low
* McDouble Combo Meal: Score one piece high, score one low, balance
* McDouble Deluxe: Score two pieces high 

*Automatic mirroring for Alliance color, Blue 1 works for Red 3, Red 1 for Blue 3

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
### Todo List
`ArmConstants.java`  
- [ ] Tune ManualArm Tele Max
- [ ] Tune ManualArm Tele Min
- [ ] Tune ManualArm Tele Speed
- [ ] Tune ManualArm Rotate Max
- [ ] Tune ManualArm Rotate Min
- [ ] Tune ManualArm Rotate Speed
- [ ] Tune AutoArm High Score Angle
- [ ] Tune AutoArm Med Score Angle
- [ ] Tune AutoArm Low Score Angle
- [ ] Tune AutoArm Med Retrieve Angle
- [ ] Tune AutoArm Low Retrieve Angle
- [ ] Tune AutoArm High Score Position
- [ ] Tune AutoArm Med Score Position
- [ ] Tune AutoArm Low Score Position
- [ ] Tune AutoArm Med Retrieve Position
- [ ] Tune AutoArm Low Retrieve Position  

`GroundIntakeConstants.java`  
- [ ] Tune kConeIntakeSpeed
- [ ] Tune kConeEjectSpeed
- [ ] Tune kCubeIntakeSpeed
- [ ] Tune kCubeEjectSpeed 
- [ ] Tune kConeGraspSpeed 
- [ ] Tune kCubeGraspSpeed  

`Claw`
- [ ] Add tof sensor hardware
- [ ] Add tof to claw motor control

`Robot Controller UI`  
- [ ] Test ManualArmUpDown
- [ ] Test ManualArmExtendRetract
- [ ] Test ManualRollerOut
- [ ] Test ManualRollerIn
- [ ] Test AllStow  
- [ ] Test ManualClawOpen
- [ ] Test ManualClawClose

`_Cube`
- [ ] Test ArmScore 
- [ ] Test AutoArmScoreHigh
- [ ] Test AutoArmScoreMedium
- [ ] Test AutoArmScoreLow
- [ ] Test GamePieceToggle
- [ ] Test IntakeRetrieve
- [ ] Test IntakeGrasp
- [ ] Test IntakeScore
- [ ] Test AutoArmRetrieveMedium
- [ ] Test AutoArmRetrieveLow

`_Cone`
- [ ] Test ArmScore 
- [ ] Test AutoArmScoreHigh
- [ ] Test AutoArmScoreMedium
- [ ] Test AutoArmScoreLow
- [ ] Test GamePieceToggle
- [ ] Test IntakeRetrieve
- [ ] Test IntakeGrasp
- [ ] Test IntakeScore
- [ ] Test AutoArmRetrieveMedium
- [ ] Test AutoArmRetrieveLow

`DriveTrain`
- [ ] Get sysId after ballast is finalized
- [ ] Save sysId values to `DriveConstants.java`
- [ ] Test AutoDrive `curvy path`
- [ ] Test AutoDrive `straight`
- [ ] Test AutoDrive `Short Straight path`
- [ ] Test AutoDrive `more curvy path`
- [ ] Test with AprilTags
- [ ] Add Field display to Shuffleboard

`LEDs`
- [ ] Add one or two strings to robot
- [ ] If needed, add SW support for second string

`User Interface`
- [ ] Update XBox button mappings for missing functionality
- [ ] Add Robot Control device box
- [ ] Create mappings for Robot Control device

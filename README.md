# CommandBased_2023
FRC Robot 2023 Java Code: Command-Based framework
** Updated to 2023.1.1 Release version

### Subsystems

#### DriveSubsystem
* SparkMax controlled 4-Motor (2 Leaders/2 Followers) Drive SystemDepends upon RevRobotics RevLib vendor library
* Kauai Labs NavX IMU - [2023 Vendor Library](https://dev.studica.com/releases/2023/NavX.json) 

#### ExampleSubsystem
* Spike relay test light (on/off)

### Commands

* TurnToAngle(angle): Turn to an arbitrary angle (Right Positive) 
* TurnToAngleProfiled(angle): PID version of above

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
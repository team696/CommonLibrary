# 696 Library for FRC 2025 season

## Usage: 

### Automatic Install
 - Initialize git for your project (you don't actually need to make a github page, just press the button on the left side in VS Code called Source Control)
 - Go To Root of your project (one with the src folder and gradle files) 
 - Run
``` 
git submodule add --name team696Library https://github.com/team696/CommonLibrary.git src/main/java/frc/team696 
```

### Manual Install
Download Zip And Drag team696 Folder Into /frc/ Directory Next To /robot/

# Requirements:

### Libraries:

Easily Install Through WPILIB Vendor Dependencies Tab (New for 2025)

* Phoenix 6
* Path Planner
* Advantage Kit (Recommended for logging, though optional)
    - Follow Instructions on [Advantage Kit Docs](https://docs.advantagekit.org/installation/)
        - Must Also Install [Version Control Section](https://docs.advantagekit.org/installation/version-control)
* Photonvision (If Needed)
    - Commented Out By Default

### Additional Installation

Add to build.gradle dependencies 
```
implementation 'com.sun.net.httpserver:http:20070405'
implementation 'org.java-websocket:Java-WebSocket:1.5.5'
implementation 'org.slf4j:slf4j-nop:2.0.6'
```

## Other Useful Information
Build.gradle
   * Change DeleteOldFiles to true
        

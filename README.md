# 696 Library for FRC 2025 season

## Usage: 
### Automatic Install
 - Go To Root of your project (one with the src folder and gradle files) 
 - Run
``` 
git submodule add --name team696Library https://github.com/team696/CommonLibrary.git src/main/java/frc/team696 
```

### Manual Install
Download Zip And Drag team696 Folder Into /frc/ Directory Next To /robot/

### Add to build.gradle dependencies 
```
implementation 'com.sun.net.httpserver:http:20070405'
implementation 'org.java-websocket:Java-WebSocket:1.5.5'
implementation 'org.slf4j:slf4j-nop:2.0.6'
```

## Requirements (Libraries):
* Phoenix 6
* Path Planner
* Advantage Kit
* Photonvision 
    - To Remove Just Delete team696/lib/Camera/PhotonVisionCam.java file

## Other Useful Information
Build.gradle
   * Change DeleteOldFiles to true
        

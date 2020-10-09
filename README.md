# LandByTrackRect
## Add camera to iris:
https://blog.csdn.net/u013083665/article/details/104840286

## CorrectImgFrom320To640:
```
$ cd LandByTrackRect  
$ mv fpv_cam.sdf ~/Firmware/Tools/sitl_gazebo/models/fpv_cam
```
## AddIris:
```
$ mv iris.sdf ~/Firmware/Tools/sitl_gazebo/models/fpv_cam
```
## AddMark:
```
$ mv apriltag ~/Firmware/Tools/sitl_gazebo/models  
$ mv arucotag ~/Firmware/Tools/sitl_gazebo/models  
$ mv landmark ~/Firmware/Tools/sitl_gazebo/models  
```
## AddTfmini:
https://xindong324.github.io/2020/06/26/mavros%E8%B8%A9%E5%9D%91%E8%AE%B0%E5%BD%95/

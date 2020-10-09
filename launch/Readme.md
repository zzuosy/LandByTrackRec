*640 *320 is the cam pixel
# group 1   
1. cam_calibed_start-640 will start the usb_cam and img_proc node
2. simple_landing will start the keyboard node, the track node and the simple_landing_node
>> this is good for the print log of the track node, cause the usb_cam have lots of noise log

# group 2

1. simple cam test will start usb_cam,img_proc, track and the keyboard

# group 3
1. simple_landing_cam will start the usb_cam,img_proc,track, keyboard and **simple_landing**  

# group4
1. the only difference between usb_cam.launch and simple_landing_cam is usb will start the landing node rather than simple_landing node
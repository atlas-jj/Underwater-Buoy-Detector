# au_vision (Geometric Shape Detector: arc_detector)
## libraries needed
1. opencv
2. lapack blas

## compile
This repository depends on other codes (e.g., au_core) written by our ARVP team, University of Alberta. As a result, you can't get it compiled unless you the other code. However, the geometric shape detector itself is self-complete. All the algorithm codes are included.

## test
1. remember to source setup.bash
2. roslaunch au_vision arc_detector.launch
3. rosrun au_vision mock_front_camera.py path/to/your/video/file
4. rqt
5. in rqt GUI:
    Detector GUI 
    -->> input image topic /front/camera/image_raw
    -->> debug image topic /ArcDetector/debug
    press Detect Button.
6. Check detection result.    
  
## problems
1. false detection.
2. color detection. (detected color results are mostly, green.)
3. Detection often fails when buoy is close. Still trying to find solutions.

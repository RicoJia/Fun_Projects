 ========================================================================
 ## Project Plan 
 ========================================================================
1. Research camera calibration for single view and stereo view. 
    1. cv2 calibration  
        - Input: 3D coordinates of the grid. 2D coordinates on the image (can be easily found)
        - Need at least 10 patterns for good results. 
            1. Should be on different different planes
            2. origin shouldn't matter. 
    2. Extracting chessboard: findChessboard is not responsive: requirements: 
        1. number of rows and clns must match, (the outer most rim doesn't count, because the corner points are the intersection of two black boxes)
        2. white space around the chessboard, the wider the better. 
        3. Need cornersubpix for better accuracy. 
        4. Maybe try thresholding first? No need, already does that

    3. coords (D)
    4. undistort (D)

2. Experiment with single-cam (D)

3. Experiment with stereo
    1. separate single cams 
    2. calibration 
        - modify the code
        - Instantiate two calibrators
    3. Check result 
        - Corners will distort
    4. Calibrate stereo
        - save file 
        - load params from calibrator, then from file 
        - R, T stereoCalibrate: 
            1. single_cam 
            2. Recalibrate single cam 
            3. Recalibrate stereo

4. Get depth image and point clouds
    1. Depth image
        - Infrastructure: Make Record image, load image. (A nice general infrastructure to have!)
    2. Estimate depth
        - get depth at certain points. print them on screen
            - why point cloud gives the (cln, row) instead of (row, cln)?
    3. Visualize the 3D projected points. 
        - Need to associate point RGB with point cloud 
        - Visualize


    

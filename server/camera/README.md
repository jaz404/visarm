## trying detection with hough transforms

idea is to create two point to line errors for the control law 
each for an opposite edge of the cube 

hough transforms is decent but it also detects all the other lines in the environment
need some sort of filtering 

the new version uses the opencv's contours which is given the edges as an input from the canny filter

based on the contours, it formalizes best candidates for the cube and then picks the best one 

the cube detection is working (right now for single detections)
the vertical lines are shown as L1 and L2 in the window 
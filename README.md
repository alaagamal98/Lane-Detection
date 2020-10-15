# **Finding Lane Lines on the Road** 
In this project, I built a computer vision pipeline for detecting lane lines and creating averaged and extrapolated boundary lines.
The pipeline is as follows: 

1) Convert frame to grayscale  
2) Create masks for yellow and white pixels  
3) Apply a Gaussian smoothing  
4) Apply a Canny edge detection  
5) Create an additional mask to focus on the "region of interest" in front of the vehicle  
6) Convert the points(i.e. pixels) in XY space to a line in Hough space  
7) Where the lines in Hough space intersect (i.e. a point) a line exists in XY space  
8) Using the extrema of the lines generated, create two averaged line  s
9) Create two averaged lines across frames for a smooth video playback  
10) Draw the lines to each frame
***

<figure>
 <img src="images/line-segments-example.jpg" width="380" alt="Combined Image" />
 <figcaption>
 <p></p> 
 <p style="text-align: center;"> Raw lines after conversion from Hough space </p> 
 </figcaption>
</figure>
 <p></p> 
<figure>
 <img src="images/laneLines_thirdPass.jpg" width="380" alt="Combined Image" />
 <figcaption>
 <p></p> 
 <p style="text-align: center;"> Averaged lines for smooth playback</p> 
 </figcaption>
</figure>


# Loopy Belief Propagation for Stereo Matching
A C++ implementation of Loopy Belief Propagation for stereo matching that offers the choice between six message update schedules. It uses the "Min-Sum" version of the algorithm with a small improvement for better results. The improvement is that in the calculation of belief, the data cost does not add up (normally it had to add up one time).

## Input Image
The Tsukuba stereo image that used as input.

<p align="center">
  <img src="left.png"> 
</p>

## Output Image
The disparity map that created at the output using the "Synchronous" message update schedule.

<p align="center">
  <img src="results/disparity1.png"> 
</p>

The disparity map that created at the output using the "Bipartite" message update schedule.

<p align="center">
  <img src="results/disparity2.png"> 
</p>

The disparity map that created at the output using the "Accelerated" message update schedule.

<p align="center">
  <img src="results/disparity3.png"> 
</p>

The disparity map that created at the output using the "Accelerated2" message update schedule.

<p align="center">
  <img src="results/disparity4.png"> 
</p>

The disparity map that created at the output using the "ByRow" message update schedule.

<p align="center">
  <img src="results/disparity5.png"> 
</p>

The disparity map that created at the output using the "ByRow2" message update schedule.

<p align="center">
  <img src="results/disparity6.png"> 
</p>


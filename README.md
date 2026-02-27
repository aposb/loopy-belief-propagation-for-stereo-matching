# Loopy Belief Propagation for Stereo Matching
A C++ implementation of Loopy Belief Propagation for stereo matching, featuring six alternative message update schedules to analyze convergence behavior and solution quality.

## Input Image
The Tsukuba stereo image that used as input.

![Tsukuba Left](left.png) ![Tsukuba Right](right.png)

## Output Image
The disparity maps that created at the output.

### Belief Propagation (Synchronous)

![Belief Propagation (Synchronous) Disparity Map](results/disparity1.png)

### Belief Propagation (Bipartite)

![Belief Propagation (Bipartite) Disparity Map](results/disparity2.png)

### Belief Propagation (Accelerated)

![Belief Propagation (Accelerated) Disparity Map](results/disparity3.png)

### Belief Propagation (Accelerated2)

![Belief Propagation (Accelerated2) Disparity Map](results/disparity4.png)

### Belief Propagation (ByRow)

![Belief Propagation (ByRow) Disparity Map](results/disparity5.png)

### Belief Propagation (ByRow2)

![Belief Propagation (ByRow2) Disparity Map](results/disparity6.png)

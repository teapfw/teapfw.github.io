---
title: Transparent Grime, Glare, & Noise Removal
author: freidank
date: 2025-04-29 12:00:00 +0800
categories: [Robotics]
tags: [computer vision,apf]
render_with_liquid: true
math: true
---

Question: Can we develop an algorithm to separate distant visual features of interest from transparent foreground noise?

Sensors and robust signal processing form one leg of the proverbial Sense-Decide-Act loop. But as robotocists are well-aware, our sensors rarely receive or provide pristine information. It's up to us to either improve our sensing capabilities or design post-processing algorithms to eliminate noise.

As a motivating example for the particular question, consider the job of a landing signal officer abord a naval aircraft carrier. The LSO is responsible for giving corrective guidance to the landing aircraft. Those who prefer to fly less by the seat of their pants will make use of a platform camera embeedded in the flight deck and oriented towards the landing aircraft with an angle calibrated to the correct approach path. Consider what may happen if this camera experienced fogging, glare, or some other transparent noise pattern that would hinder the LSO's undistracted view, or confuse some automatic processing software. Can we remove this noise?

One option could leverage the parallax effect of the noise pattern being produced by the lens, which is significantly closer to the sensor frame than the rest of the objects contributing to the final image. For the time being, let's assume that the target signal is produced by objects located at a much larger distance from the camera than the noise signal; large enough that the parallax effect is negligible except for the noise signal. If we place a stereo camera pair behind a shared external lens shield, then we should expect the noise pattern to be identical and merely shifted across the left and right images. we can express the signal contribution for pixel column $$P_i$$ as follows:

$$
^R P_i = S_i + N_{i+d}
$$

$$
^L P_i = S_i + N_{i-d}
$$

where $$S$$ is the target isgnal, $$N$$ is the noise signal, and $$d$$ is the pixel disparity between the noise signals across the stereo pair. We find ourselves requiring $$ N_k = 0 $$ if $$ k \notin [0,r) $$ for an image of $$r$$ columns, which is simply to say that the noise pattern must be wholly visible in both images.

From here, we can form a system of equations of the form $$\mathbf{A}x=b$$ that can be solved by inversion. Here's an example:

```matlab
clc; clear all; close all;

S = [1,2,3,4];
N = [0,1,1,0];
d = 1;

r = length(S);
P_L = S + [zeros(1,d), N(1:end-d)]
P_R = S + [N(1+d:end), zeros(1,d)]

b = [P_L'; P_R']
A = zeros(r);

for i = 1:r
    % P_L equations
    A(i,i) = 1;

    j = i-d;
    if j > 0 && j <= r
        A(i,r+j) = 1;
    end

    % P_R equations
    A(r+i,i) = 1;

    j = i+d;
    if j > 0 && j <= r
        A(r+i,r+j) = 1;
    end
end

% Solve
pred = A \ b
```

Incidentally, I found a paper later that operated on a similar if not the same idea {% cite Tsurumi2012RemovalOR %}.

We could conjecture further how we might deal with imperfections in our assumptions. Let's say there is in fact some minor parallax effect in the target signal, and that our anticipated noise disparity is not exactly equal to its true value. We could apply a sort of kernel that averages neighboring columns in an attempt to improve the algorithm's result in the presence of such model distortions.

$$
^L P_i = \mathcal{S}_i + ^L \mathcal{N}_i
$$

$$
^R P_i = \mathcal{S}_i + ^R \mathcal{N}_i
$$

$$
\mathcal{S}_i = \sigma_0 S_i + \frac{1}{2} \sum_{j=1}^{n} \sigma_j (S_{i-j} + S_{i+j})
$$

$$
^L \mathcal{N}_i = \eta_0 N_{i+d} + \frac{1}{2} \sum_{j=1}^{n} \eta_j (N_{i+d-j} + N_{i+d-j})
$$

$$
^R \mathcal{N}_i = \eta_0 N_{i-d} + \frac{1}{2} \sum_{j=1}^{n} \eta_j (N_{i-d-j} + N_{i-d-j})
$$

for some kernel width $$n$$, where $$ 1 = \Sigma \sigma_i = \Sigma \eta_i $$.

Perhaps the algorithm could be run for different weighting factors $$\sigma$$ and $$\eta$$ in search of optimizing some output metric, such as sharpness of the calculated target signal.

# References
{% bibliography -q @inproceedings --cited %}
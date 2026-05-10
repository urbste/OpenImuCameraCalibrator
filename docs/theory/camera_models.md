# Camera Models in Theory

OpenICC relies on the [TheiaSfM](http://theia-sfm.org) camera models. Below is a short mathematical overview of the models supported by the toolbox.

## Pinhole

Standard perspective projection:

\[
\mathbf{u} = \begin{bmatrix} f_x & 0 & c_x \\ 0 & f_y & c_y \\ 0 & 0 & 1 \end{bmatrix} \cdot \frac{\mathbf{p}}{p_z}
\]

## Division Undistortion

A single-parameter radial distortion model introduced by Fitzgibbon (CVPR 2001):

\[
r_d = \frac{r_u}{1 + \lambda r_u^2}
\]

where \(r_u\) is the undistorted radius and \(r_d\) the distorted radius.

## Double Sphere

Introduced by Usenko et al. (3DV 2018). The projection is modelled by two consecutive unit spheres:

\[
\mathbf{m} = \begin{bmatrix} x \\ y \\ z \end{bmatrix} = \frac{1}{\xi d_1 + \sqrt{d_2}} \begin{bmatrix} X \\ Y \\ \xi d_1 + \sqrt{d_2} \, Z \end{bmatrix}
\]

with \(d_1 = \sqrt{X^2+Y^2+Z^2}\) and \(d_2 = Z^2 + (1-\xi^2)(X^2+Y^2)\).

## Extended Unified

Khomutenko et al. (IEEE RA-L 2016):

\[
r_d = \frac{r_u}{\beta \sqrt{1 + (1-\beta^2)\alpha^2 r_u^2} + (1-\beta) }
\]

## Fisheye (Kannala-Brandt)

A generic polynomial model for fisheye lenses:

\[
\theta_d = \theta (1 + d_1 \theta^2 + d_2 \theta^4 + d_3 \theta^6 + d_4 \theta^8)
\]

where \(\theta\) is the angle between the optical axis and the incoming ray.

## References

- [2] Usenko et al., "The Double Sphere Camera Model", 3DV 2018.
- [4] Khomutenko et al., "An Enhanced Unified Camera Model", IEEE RA-L 2016.
- [5] Fitzgibbon, "Simultaneous Linear Estimation of Multiple View Distortion", CVPR 2001.
- [6] Kannala & Brandt, "A Generic Camera Model and Calibration Method", TPAMI 2006.

# Supported Camera Models

OpenICC supports several camera projection models. Choosing the right model depends on your lens and desired accuracy.

| Model | Use case | Parameters |
|-------|----------|------------|
| **PINHOLE** | No distortion, e.g. smartphone linear mode | fx, fy, cx, cy |
| **PINHOLE_RADIAL_TANGENTIAL** | Standard DSLR / smartphone with mild distortion | fx, fy, cx, cy, k1, k2, k3, p1, p2 |
| **DIVISION_UNDISTORTION** | Wide-angle lenses with simple radial distortion | fx, fy, cx, cy, lambda |
| **DOUBLE_SPHERE** | Fisheye / very wide action cameras | fx, fy, cx, cy, xi, alpha |
| **EXTENDED_UNIFIED** | Wide-angle lenses, generalizes Kannala-Brandt | fx, fy, cx, cy, alpha, beta |
| **FISHEYE** | Strong fisheye distortion (Kannala-Brandt) | fx, fy, cx, cy, d1, d2, d3, d4 |
| **FOV** | Field-of-View model (Devernay-Faugeras) | fx, fy, cx, cy, omega |

!!! tip
    For GoPro cameras in **Wide** mode we recommend **Division Undistortion** or **Extended Unified** as a good trade-off between accuracy and parameter count.  
    For **Max Lens Mod** or 360° cameras consider **Fisheye** or **Double Sphere**.
    Only use **PINHOLE** when you do not expect any distortion (e.g. smartphone in Linear mode).

## Recommended settings

| Device | Setting | Suggested model |
|--------|---------|-----------------|
| GoPro 6/7/8/9 (Wide) | 1080p / 960p Wide | Division Undistortion |
| GoPro 9 (Max Lens Mod) | 1440p MaxLens | Fisheye |
| Smartphone (Linear) | Any | Pinhole |
| ZED 2i | Default | Pinhole + Radial-Tangential |

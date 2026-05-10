# Contributing

Contributions are welcome! Here are a few ways you can help:

- **Bug reports**: If you encounter a crash, wrong calibration result or build failure, please open an issue with a minimal reproducer.
- **New camera models**: TheiaSfM supports additional models (e.g. Scaramuzza omnidirectional). Adding them to OpenICC is mostly a matter of plumbing the model name through the Python scripts and C++ initializers.
- **Pose estimation improvements**: Currently the toolbox undistorts corners and runs vanilla PnP. Implementing UPNP / MLPnP would enable direct calibration of ultra-wide fisheye lenses without undistortion.
- **Better rolling-shutter calibration**: Improving residual weighting or initialization for the line-delay parameter.
- **Documentation**: More tutorials, datasets, or translated pages are always appreciated.

## Code style

- C++ code is formatted with `.clang-format`. Please run `clang-format` before submitting a PR.
- Python scripts should follow PEP 8.

## License

The project is released under the **GNU Affero General Public License v3** (or later). By contributing you agree that your contributions will be licensed under the same terms.

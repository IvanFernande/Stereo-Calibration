
# Camera Calibration and Depth Perception Toolkit

This repository contains Python scripts designed for camera calibration, checking camera status, and estimating depth using stereo vision. These tools are essential for computer vision tasks that require precise measurements and depth information from camera inputs.

## Scripts Included

- **calibration.py**: Captures video from stereo cameras and performs calibration using a chessboard pattern to find camera intrinsics and distortion coefficients.
- **camera_check.py**: Checks the operational status of the connected cameras to ensure they are functional and capturing video streams correctly.
- **depth.py**: Uses the calibration data to compute the depth map from the stereo images, providing real-time depth perception.

## Installation

Clone this repository to your local machine using:

```bash
git clone <repository-url>
```

Ensure you have Python installed, and install the required libraries:

```bash
pip install numpy opencv-python
```

## Usage

Each script can be run independently depending on the task. Here is a brief on running each script:

### Camera Calibration

```bash
python calibration.py
```

Follow the on-screen instructions to capture calibration images by pressing 'c'. Quit the program using 'q'.

### Camera Check

```bash
python camera_check.py
```

This script will automatically check and display the video feed from the cameras.

### Depth Estimation

```bash
python depth.py
```

Ensure calibration data is correctly loaded for accurate depth estimation. Adjust parameters using '+' or '-' for fine-tuning.

## Contributing

Contributions to this project are welcome! Please fork the repository and submit a pull request with your improvements.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

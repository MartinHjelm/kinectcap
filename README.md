Added a homebrew tap and formula for installation https://github.com/MartinHjelm/homebrew-taps

# KINECTCAP

Captures Kinect image and point cloud data, and saves them to disk when a key is pressed.

Included is also a small PCD viewer for viewing the captured point cloud.


##### Usage

- **To capture:** ./kcap
- **To view:** ./viewpcd path-to-file


**Requirements:** OpenCV & Point Cloud Library(PCL)


## TODO
- Fix bug that doesn't make image display when capturing data
- Add install to slash bin to cmake
- Add q keypress to exit pcd viewer
- Fix bug that delays info to terminal about which image was saved for two keystrokes.

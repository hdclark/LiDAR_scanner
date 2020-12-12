
## About

The main aim of this project is to support a hand-held LiDAR scanner built using
an `Intel Realsense L515` camera module and a `Raspberry Pi` 4.

This software is being actively developed. At the moment it produces lightly
pre-processed point clouds that need to be stiched together with auxiliary
programs.

Currently, reading from the camera and writing to disk are separated; frames are
captured in memory until a fixed-size RAM buffer has been filled. Data is
written to disk after collection, and is slower than collection. So the camera
can only operate in bursts. This slightly awkward procedure is used to capture
as many frames as possible, as regularly as possible. Capturing frames quickly
helps reduce frame stutter and motion artifacts.

The `L515` includes motion tracking via an inertial measurement unit (IMU). One
of the goals of this project is to be able to walk around and scan a static
scene. To help assist in combining individual frames into a final consistent
point cloud, frames should honour the position and orientation of the camera
module. If data collection and data writing were interleaved, collection would
be stymied by writing, and motion correction accuracy would suffer. Instead, the
large RAM offered by the `Raspberry Pi` 4 is used to read bursts.

Individual frames are transformed according to IMU measurements before
writing, but must be registered, simplified, and merged to provide a final
composite.


## License

GPLv3.


## Dependencies

- [`librealsense2`](https://github.com/IntelRealSense/librealsense)
- [`libygor`](https://github.com/hdclark/Ygor)


## Camera Operation

This camera will support point-and-click operation. Data is a bit more tricky,
and some additional tweaking might be needed to suit your hardware.

To keep the design simple (initially), the only feedback provided will be via
LED(s). This may change if more information is required.


## Development Log

- December 2020
  - Project started.
  - Struggled to compile `librealsense2` for `Arch Linux ARM` using mainline
    `aarch64` kernel with `U-boot`. Neither `libuvc`, `rsusb`, nor kernel
    patching worked (customizing patches failed too). Device failed to be
    recognized. Found pre-built `Raspbian` image with `librealsense2`
    pre-installed, but it also did not pick up the `L515`. Determined that
    compiling most recent `librealsense2` release with `rsusb` on `Raspian`
    worked correctly, but performance was limited. Switched to `Arch Linux ARM`
    using `armv7` image and successfully compiled from the
    `librealsense-v2.40.0-1` `AUR` package.
  - Minimal working setup achieved. Camera provides one point cloud with
    registered colour information for each measured frame. 2GB or RAM is assumed
    to be available. The IMU is not yet used, and no transformation is applied
    to point clouds, so all registration must be from scratch. Works best with
    slow panning for simple objects/subjects.


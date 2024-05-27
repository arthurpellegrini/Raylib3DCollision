Raylib C/C++ - Maths 3D Project
==============================================================

Description
--------------------------------------------------------------
This project is a culmination of three practical sessions (TD1, TD2, and TD3) carried out as part of the first year of the Application Development and 3D Engineering cycle at ESIEE. The project was developed by a group of three students.

Visual
--------------------------------------------------------------

Here is a screenshot of the 3D Primitives built.
![3D Primitives][visuals]


TD1
--------------------------------------------------------------
### Prerequisites:
* Basic Raylib project provided beforehand
* Development of Cylindrical and Spherical structures
* Implementation of Cartesian ↔ Cylindrical and Cartesian ↔ Spherical conversion methods:
  * Cylindrical CartesianToCylindrical(Vector3 cart)
  * Vector3 CylindricalToCartesian(Cylindrical cyl)
  * Spherical CartesianToSpherical(Vector3 cart)
  * Vector3 SphericalToCartesian(Spherical sph)

### Objectives:
* Applying the spherical coordinate system to the development of an orbital camera
* Implementing C structures to define 3D primitive objects
* Developing and optimizing rendering methods for these 3D objects using immediate OpenGL rendering

### Additional goals:
* Implementing rendering methods for a portion of a cylinder
* Implementing rendering methods for a portion of a sphere


TD2
--------------------------------------------------------------
### Prerequisites:
* Basic Raylib project provided beforehand
* Development of Cylindrical and Spherical structures
* Implementation of Cartesian ↔ Cylindrical and Cartesian ↔ Spherical conversion methods:
  * Cylindrical CartesianToCylindrical(Vector3 cart)
  * Vector3 CylindricalToCartesian(Cylindrical cyl)
  * Spherical CartesianToSpherical(Vector3 cart)
  * Vector3 SphericalToCartesian(Spherical sph)
* Sufficient progress in TD1 to have the display methods for 3D primitives used in TD2:
  * Plane, Quad
  * Disk
  * Sphere
  * Cylinder
  * Capsule
  * Box
  * RoundedBox
  
### Objectives:
* Implementing methods to calculate intersection between a segment and various 3D objects

TD3
--------------------------------------------------------------
### Prerequisites:
* Completion of TD1 and TD2, specifically:
    * Display methods for Sphere and RoundedBox
    * Intersection calculation method for a Segment and a RoundedBox

### Objectives:
* Modeling and implementing dynamic behavior of a Sphere subject to a gravitational field
* Developing a collision detection method between a Sphere moving in a straight line and a single static Box (or RoundedBox)
  * Calculating the Sphere's position and movement direction after rebound
* Developing a collision detection method between a Sphere moving in a straight line and a group of static Boxes (or RoundedBoxes)
  * Calculating the Sphere's position and movement direction after rebound
  * This method relies on the previous method

### Additional goals:
* Developing a method for detecting multiple collisions (within one frame) between a Sphere moving in a straight line and a group of static Boxes (or RoundedBoxes)
  * Calculating the Sphere's position and movement direction after rebound(s)
  * This method also relies on the previous method

Remarks
--------------------------------------------------------------
The main limitation of this project is the absence of the function "GetSphereNewPositionAndVelocityIfMultiCollidingWithRoundedBoxes" due to its complexity.
During the development of our project, we realized that when using the Sphere-Vector<RoundedBox> collision, it could malfunction after multiple collisions.
Possible areas of improvement include optimizing the algorithms and calculations in the code, but this would require a deeper understanding of certain mathematical concepts underlying this project.

Conclusion
--------------------------------------------------------------
We found this project to be very interesting and enriching, although we initially spent a bit too much time understanding the workings of Raylib.

[//]: # (Reference link used in the body of the Markdown)
[visuals]: https://i.goopics.net/s9nyf4.jpg

Raylib README
--------------------------------------------------------------

<img align="left" src="https://github.com/raysan5/raylib/blob/master/logo/raylib_256x256.png" width=256>

**Raylib is a simple and easy-to-use library to enjoy videogames programming.**

Raylib is highly inspired by Borland BGI graphics lib and by XNA framework and it's specially well suited for prototyping, tooling, graphical applications, embedded systems and education.

* NOTE for ADVENTURERS: Raylib is a programming library to enjoy videogames programming; no fancy interface, no visual helpers, no auto-debugging... just coding in the most pure spartan-programmers way.*

Ready to learn? Jump to [code examples!](http://www.raylib.com/examples.html)

[![GitHub contributors](https://img.shields.io/github/contributors/raysan5/raylib)](https://github.com/raysan5/raylib/graphs/contributors)
[![GitHub All Releases](https://img.shields.io/github/downloads/raysan5/raylib/total)](https://github.com/raysan5/raylib/releases)
[![License](https://img.shields.io/badge/license-zlib%2Flibpng-blue.svg)](LICENSE)
[![Chat on Discord](https://img.shields.io/discord/426912293134270465.svg?logo=discord)](https://discord.gg/VkzNHUE)

[![Windows](https://github.com/raysan5/raylib/workflows/Windows/badge.svg)](https://github.com/raysan5/raylib/actions?query=workflow%3AWindows)
[![Linux](https://github.com/raysan5/raylib/workflows/Linux/badge.svg)](https://github.com/raysan5/raylib/actions?query=workflow%3ALinux)
[![macOS](https://github.com/raysan5/raylib/workflows/macOS/badge.svg)](https://github.com/raysan5/raylib/actions?query=workflow%3AmacOS)
[![Android](https://github.com/raysan5/raylib/workflows/Android/badge.svg)](https://github.com/raysan5/raylib/actions?query=workflow%3AAndroid)
[![WebAssembly](https://github.com/raysan5/raylib/workflows/WebAssembly/badge.svg)](https://github.com/raysan5/raylib/actions?query=workflow%3AWebAssembly)

Raylib Features
--------
  - **NO external dependencies**, all required libraries are bundled into Raylib
  - Multiple platforms supported: **Windows, Linux, MacOS, RPI, Android, HTML5... and more!**
  - Written in plain C code (C99) in PascalCase/camelCase notation
  - Hardware accelerated with OpenGL (**1.1, 2.1, 3.3 or ES 2.0**)
  - **Unique OpenGL abstraction layer** (usable as standalone module): [rlgl](https://github.com/raysan5/raylib/blob/master/src/rlgl.h)
  - Multiple **Fonts** formats supported (TTF, XNA fonts, AngelCode fonts)
  - Outstanding texture formats support, including compressed formats (DXT, ETC, ASTC)
  - **Full 3D support**, including 3D Shapes, Models, Billboards, Heightmaps and more! 
  - Flexible Materials system, supporting classic maps and **PBR maps**
  - **Animated 3D models** supported (skeletal bones animation)
  - Shaders support, including model and **postprocessing** shaders.
  - **Powerful math module** for Vector, Matrix and Quaternion operations: [raymath](https://github.com/raysan5/raylib/blob/master/src/raymath.h)
  - Audio loading and playing with streaming support (WAV, OGG, MP3, FLAC, XM, MOD)
  - **VR stereo rendering** support with configurable HMD device parameters
  - Huge examples collection with [+120 code examples](https://github.com/raysan5/raylib/tree/master/examples)!
  - Bindings to [+50 programming languages](https://github.com/raysan5/raylib/blob/master/BINDINGS.md)!
  - Free and open source.

Raylib uses on its [core](https://github.com/raysan5/raylib/blob/master/src/core.c) module the outstanding [GLFW3](http://www.glfw.org/) library, embedded in the form of [rglfw](https://github.com/raysan5/raylib/blob/master/src/rglfw.c) module, to avoid external dependencies.

Raylib uses on its [raudio](https://github.com/raysan5/raylib/blob/master/src/raudio.c) module, the amazing [miniaudio](https://github.com/dr-soft/miniaudio) library to support multiple platforms and multiple audio backends.

Raylib uses internally several single-file header-only libraries to support different fileformats loading and saving, all those libraries are embedded with Raylib and available in [src/external](https://github.com/raysan5/raylib/tree/master/src/external) directory. Check [raylib Wiki](https://github.com/raysan5/raylib/wiki/raylib-dependencies) for a detailed list.

* On Android platform, `native_app_glue` module (provided by Android NDK) and native Android libraries are used to manage window/context, inputs and activity life cycle.*

* On Raspberry Pi 0,1,2,3 platform (native mode), `Videocore API` and `EGL` libraries are used for window/context management. Inputs are processed using `evdev` Linux libraries*

* On Raspberry Pi 4 platform (native mode), `DRM subsystem` and `GBM API` libraries are used for window/context management. Inputs are processed using `evdev` Linux libraries*

* On Web platform, Raylib uses `emscripten` provided libraries for several input events management, specially noticeable the touch events support.*

Build and Installation
----------------------
Raylib binary releases for Windows, Linux and macOS are available at the [Github Releases page](https://github.com/raysan5/raylib/releases).
Raylib is also available via multiple [package managers](https://github.com/raysan5/raylib/issues/613) on multiple OS distributions.

#### Installing and building Raylib via vcpkg
You can download and install Raylib using the [vcpkg](https://github.com/Microsoft/vcpkg) dependency manager:
      git clone https://github.com/Microsoft/vcpkg.git
      cd vcpkg
      ./bootstrap-vcpkg.sh
      ./vcpkg integrate install
      vcpkg install raylib

* The Raylib port in vcpkg is kept up to date by Microsoft team members and community contributors. If the version is out of date, please [create an issue or pull request](https://github.com/Microsoft/vcpkg) on the vcpkg repository.*

#### Building Raylib on multiple platforms
[raylib Wiki](https://github.com/raysan5/raylib/wiki#development-platforms) contains detailed instructions on building and usage on multiple platforms.
 - [Working on Windows](https://github.com/raysan5/raylib/wiki/Working-on-Windows)
 - [Working on macOS](https://github.com/raysan5/raylib/wiki/Working-on-macOS)
 - [Working on GNU Linux](https://github.com/raysan5/raylib/wiki/Working-on-GNU-Linux)
 - [Working on FreeBSD](https://github.com/raysan5/raylib/wiki/Working-on-FreeBSD)
 - [Working on Raspberry Pi](https://github.com/raysan5/raylib/wiki/Working-on-Raspberry-Pi)
 - [Working for Android](https://github.com/raysan5/raylib/wiki/Working-for-Android)
 - [Working for Web (HTML5)](https://github.com/raysan5/raylib/wiki/Working-for-Web-(HTML5))
 - [Working for UWP (Universal Window Platform)](https://github.com/raysan5/raylib/wiki/Working-for-UWP)
 - [Working anywhere with CMake](https://github.com/raysan5/raylib/wiki/Working-with-CMake)

* Note that Wiki is open for edit, if you find some issue while building Raylib for your target platform, feel free to edit the Wiki or open and issue related to it.*

#### Using Raylib with multiple IDEs

Raylib has been developed on Windows platform using [Notepad++](https://notepad-plus-plus.org/) and [MinGW GCC](http://mingw-w64.org/doku.php) compiler but it can be used with other IDEs on multiple platforms.
[Projects directory](https://github.com/raysan5/raylib/tree/master/projects) contains several ready-to-use **project templates** to build Raylib and code examples with multiple IDEs.

* Note that there are lots of IDEs supported, some of the provided templates could require some review, please, if you find some issue with some template or you think they could be improved, feel free to send a PR or open a related issue.*

Contact
-------
   * Webpage: [http://www.raylib.com](http://www.raylib.com)
   * Discord: [https://discord.gg/raylib](https://discord.gg/VkzNHUE)
   * Twitter: [http://www.twitter.com/raysan5](http://www.twitter.com/raysan5)
   * Twitch: [http://www.twitch.tv/raysan5](http://www.twitch.tv/raysan5)
   * Reddit: [https://www.reddit.com/r/raylib](https://www.reddit.com/r/raylib)
   * Patreon: [https://www.patreon.com/raylib](https://www.patreon.com/raylib)
   * YouTube: [https://www.youtube.com/channel/raylib](https://www.youtube.com/channel/UC8WIBkhYb5sBNqXO1mZ7WSQ)

If you are using Raylib and enjoying it, please, join our [Discord server](https://discord.gg/VkzNHUE) and let us know! :)

License
-------
Raylib is licensed under an unmodified zlib/libpng license, which is an OSI-certified, BSD-like license that allows static linking with closed source software. Check [LICENSE](LICENSE) for further details.

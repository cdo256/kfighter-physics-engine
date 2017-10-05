# KFighter physics engine

![KFighter screenshot](https://github.com/cdo256/kfighter-physics-engine/raw/master/kfighter/media/screenshot4.png "KFighter screenshot")

The rigid-body physics engine for the unfinished game 'KFighter'.

## About

It's written from scratch in C++ but it may as well be in C since it uses so few of C++ features. It's currently Windows only but it's designed to be extended to other platforms in the future. I started it by following [a tutorial](https://handmadehero.org/) by the brilliant [Casey Muratori](https://mollyrocket.com/casey/about.html). The physics engine is based off talks done by [Erin Catto](https://github.com/erincatto) of [Box2D](http://box2d.org/) fame and works by iteratively solving a set of constraints.

## Features

- Collision detection and resolution
  - Rectangle collision
  - Friction
  - Collision islands (collections of objects typically connected by joints or other constraints that shouldn't collide with each other)
  - Multiple collision points
  - Stable stacking
- Joints
  - Angle constraints
  - Joint friction
  - PID controlled motors
- Loop live code editing
- Pose matching

## Controls

- `Arrow keys`: Adjust gravity
- `Space bar`: Cycle poses
- `Escape`: Quit
- `[`,`]`: Increment/decrement random seed
- `L`: Cycle key recording; cycle between the following three actions: __begin recording__, __end recording and begin playback__, and __end playback__
- `R`: Reset game (keeping the seed the same)

## How to run

### You will need:

- Windows 7 or later (only tested on Windows 10, may work on older versions).
- Visual Studio build tools 2015 or later (may work on  earlier versions)

### Instructions:

1. Download this repo as a zip (or clone it)
2. Run `kfighter\build\win32_kfighter.exe`

## Directories

- [`kfighter\build`](https://github.com/cdo256/kfighter-physics-engine/tree/master/kfighter/build) - The directory for VC's output
- [`kfighter\code`](https://github.com/cdo256/kfighter-physics-engine/tree/master/kfighter/code) - The main code directory, all the compiled code is in here
- [`kfighter\media`](https://github.com/cdo256/kfighter-physics-engine/tree/master/kfighter/media) - Images for marketing
- [`kfighter\misc`](https://github.com/cdo256/kfighter-physics-engine/tree/master/kfighter/misc) - Editor tools and config files
- [`kfighter\reference`](https://github.com/cdo256/kfighter-physics-engine/tree/master/kfighter/reference) - Links to useful resources

## How to build

### You will need:

- Windows 7 or later (only tested on Windows 10, may work on older versions).
- Visual Studio build tools 2015 or later (may work on  earlier versions)

### Instructions:

1. Download this repo as a zip (or clone it)
2. Open `kfighter\code\build.bat` in a text editor
3. Change the second line: `pushd "C:\Program Files (x86)\Microsoft Visual Studio\2017\Community\VC\Auxiliary\Build\"` to be inside the directory of your version of Visual Studio
4. `cd kfighter\code`
5. Run `build.bat`

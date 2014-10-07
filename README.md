Kemar-GenoM Module
========
Copyright (c) 2013 CNRS/LAAS  
Permission to use, copy, modify, and/or distribute this software for any purpose with or without fee is hereby granted, provided that the above copyright notice and this permission notice appear in all copies.  
THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.  
Summary
------------------
The **Kemar-GenoM** module controls the Kemar Head&Torso thorugh GenoM. It has the ability to "Control in Position", "Control in Speed" and retrieve the "Current State" (position and speed). It can be controlled and manipulated via TCL or throguh the [Genomix-Matlab Bridge](https://dev.qu.tu-berlin.de/projects/twoears-matlab-genomix-bridge).

Quick Start
-----------

#### 0. Compiling and Installing "Elmo-Axis" library.
A special library has been coded at LAAS for the Elmo Controller. Therefore, it has to be built before it can be included in the GenoM module.  
The steps to do this are the following, to be run in a *Terminal*
```
> git clone git://git.openrobots.org/robots/elmo-axis-libs.git
> cd elmo-axis-libs
> git checkout --track origin/kemar-head
> mkdir m4
> autoreconf -vif
> mkdir build
> cd build
> ../configure --prefix=$INSTALL_DIR
> make
> sudo make install
```

#### 1. How to build the Kemar-GenoM Module
- Clone the repository:  
    0.1. Open a new Terminal and clone the repository (if not done beofre).
```
> git clone https://dev.qu.tu-berlin.de/git/twoears-kemar-genom-module/repository
```
- Build the module  
    1.1. Go to path /twoears-kemar-genom-module  
```
> genom3 skeleton -m auto kemar.gen
> ./bootstrap.sh
> mkdir build
> cd build
> ../configure --prefix=$INSTALL_DIR --with-templates=$TEMPLATES
> make
> sudo make install
```  
note: Be sure to include **codels-require "elmo-axis-libs";** in the component section in the .gen file.

#### 2. How to run the Kemar-GenoM Module
In the same Terminal:
```
> roscore &
> kemar-ros -b &
> genomixd &
> eltclsh
> package require genomix
> ::genomix::connect
> genomix1 load kemar 
```

#### 3. How to use the Kemar-GenoM Module
- *Homing*  
This service has to be called **every time** on *initialitation* and if necessary.
```
eltclsh > ::kemar::Homing
```
- *Move Absolute Position (Control in Position)*  
This service will move to the *absolute* position within left and right limits determined during Homing, being zero the head "looking" to the front.
```
eltclsh > ::kemar::MoveAbsolutePosition
double target >
double velocity >
```
where target is the desired position. Both values are in deg and deg/sec.

- * Move Relative Position (Control in Position)*  
This service will move to the *relative* position from the current position.
```
eltclsh > ::kemar::MoveRealtivePosition 
double target >
double velocity >
```
where target is the position. Both values are in deg and deg/sec.

- *Control In Speed*  
This service will set a speed and the head will move at that speed until it is modified, set to zero (by calling it again) or when the head reaches one of the limits, when the set is set to zero.
```
eltclsh > ::kemar::ControlInSpeed 
double velocity >
``` 
where velocity is in deg/sec.

- *Current State*  
This service shows the current *position* and *velocity* on the shell. 
```
eltclsh > ::kemar::CurrentPosition 
```
They are also published every 10ms on the port called currentState.

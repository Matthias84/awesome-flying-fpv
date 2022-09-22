# awesome-flying-fpv

This is a list of Free Software and Open Hardware dedicated for Multicopters as well as airplanes or flying wings. We list esp. established projects with a active community and also some classics that might be legacy but established projects with a lot of mods.

We hope to give you some orientation where to start and present you the full spectrum of open source development on the world of UAVs.

> **Warning**
> You as creator, machanic, spotter and pilot **are responsible** for your drone and doing. It is on you to avoid damages and not to hurt any person or animal.
> You are also responsible to match the local regulations of your country and knowing where and how to fly with a minimum risk to others and yourself.
> Please respect [good airmanship](https://en.wikipedia.org/wiki/Airmanship).

> **Warning**
> In past and recent wars and **military conflicts**, parties make use of UAVs and sometimes DIY aircraft to spot and attack opponents.
> My intention on this tecnology is opposite a peaceful research and learning on technologies and nature, not on killing people! [stopkillerrobots.org](https://www.stopkillerrobots.org)

## Airframes

The body of a UAV depends on the kind of aircraft and is optimised for it's usecase. Is it high speed for racing, improved agility for acrobatic freestyle, a heavy lifter for filming , long distance observations, ... . This requirements result in different mechanics and materials and take DIY aspect into account.

You will find also a lot of spare parts or mods / extensions to commercial vehicles, which we don't list up here!

> **Note**
> Building a UAV from scratch is a **pretty challengin task** for a beginner, esp. if you have limited amount of time.
> So it might be wise, to try existing solutions or kits before, so you don't get stuck to early and have a handy manual. Later on, you can try different mods or start with custom builds, if you have gained more experience and know how to avoid common pitfaults.
> See also: [My Raspberry Pi drone: the story so far by Matchstic](https://www.youtube.com/watch?v=ZCOlT_sz6Gs).

### Multicopters

Copters are built using differnt materials from alu / carbon profiles to CNC cuts to full 3D printed cases and have different configurations for different number of rotors (bi... octo):

* [NanoLongRange](https://www.thingiverse.com/thing:4769576) 3D printed frame with mostly a 18650 Liion cell and a all-in-one board for whoops
* [NanoLongRange 2](https://www.thingiverse.com/thing:4818009) supporting also 21700 Liion cell and slightly lighter
* [Ultimate 3D printable Cinewhoop](https://www.thingiverse.com/thing:4502805), 2020
* [TinyTina](https://blog.prusaprinters.org/how-to-build-a-3d-printed-micro-drone_29310/) 3D printed whoop, 2018
* [Heavy Lift Quadcopter Frame](https://www.thingiverse.com/thing:4089842) CNC cut by carbon, 2020
* [The CogniFly](https://thecognifly.github.io) robust frame for research, swarms and indoor with RPI companion computer, 2021
* [TBS Source One](https://github.com/tbs-trappy/source_one) carbon racing frame in 5 revisions, 2021
* [TBS Source Two](https://www.team-blacksheep.com/products/prod:source_two_5in) carbon racing frame, 2019
* [TBS Source Podracer](https://github.com/ps915/source_podracer) 3D carbon racing frame, 2020
* [TBS Source V](https://www.team-blacksheep.com/products/prod:source_v) 5" carbon racing frame, 2021
* [TBS Source X](https://github.com/ps915/source_x) carbon racing frame, 2019
* [AESIR II](https://www.thingiverse.com/thing:4868250) modular and customizable 3D and carbon frame, 2021
* [Foldable Drone Frame](https://www.thingiverse.com/thing:2004357) 3D printed with gimbal option 2017
* [OpenRC Quadcopter](https://www.thingiverse.com/thing:793425) 3D printed with closed case , 2015
* [Hovership MHQ2](https://www.thingiverse.com/thing:511668) 3D printed foldable, 2014
* [Crossfire 2](https://www.thingiverse.com/thing:234867) huge quad 3D printed, 2014
* [Spyda 500](https://www.thingiverse.com/thing:160607) huge quad 3D printed, 2013

### Fixed Wing / Planes

Traditional RC airplanes are created using balsa wood and foil covered rips for the wings. Commercial models make use of foam, that you can also cut by CNC and lasers to create wing profiles.  Also fully 3D printed planes are possible, that you glue and strength with carbon roods. Here is a growing community for commercial 3D models to print [Craycle Hobby](https://craycle.com/), [Eclipson airplanes](https://www.eclipson-airplanes.com/),[3D lab print](https://3dlabprint.com/product-category/printable-airplanes/), [Plane Print](https://www.planeprint.com/) , [OWLplane](https://owlplane.com/) or [rc-jetprint.de](https://rc-jetprint.de/en/) using lightweight PLA to save weigth

* [LukiSegler](https://www.printables.com/de/model/76098-lukisegler-electric-rc-glider) glider, 2021
* [Eclipson Model V](https://www.thingiverse.com/thing:4011218) mostly 3D printed commercial airplane with wheels, from 2019
* [Eclipson Model Y](https://www.thingiverse.com/thing:2752892) mostly 3D printed commercial airplane with wheels, from 2018
* [Northern Pike](https://www.thingiverse.com/thing:3040294) 36" fixed wing 3D printed, 2018
* [RC Flying Wing](https://www.thingiverse.com/thing:2044074) , 2017
* [GASB Three](https://www.thingiverse.com/thing:3605665) fixed wing 3D printed, 2019
* [GASB Two](https://www.thingiverse.com/thing:1831295) fixed wing jet 3D printed with electric ducted fan (EDF) instead of classic propeller, 2016
* [GASB One](https://www.thingiverse.com/thing:1659724) 80cm fixed wing 3D printed, developed in 6 revisions, 2016

### VTOLs

Vertical take-off and landing are air vehicles that transform from copter to a gliding wing, making it very easy to start/land but require a more complex mechanic and become a tip heavier than fixed wings.

* <https://www.youtube.com/watch?v=gPEeCjVrTBw&t=190s> VTOL in 5 revisions
* [bicopter kit](https://hackaday.com/2018/08/27/the-best-new-quad-is-a-bicopter/) with CNC carbon, 2018
* <https://www.printables.com/de/model/261434-vase-mode-wing> Wing Profile 3D printing with LW-PLA

## Batteries & Power Control

Commercial LiPo battery packs are the defaults in RC, but can be replaced with custom 18650 based Liion packs. The onboard power bus powers the ESC and FC directly, which offer 5V outputs to supply onboard units.

* 18650 Liion packs
  * [Using Li-Ion Battery Pack for Long Range FPV Flying](https://oscarliang.com/li-ion-battery-long-range/) 4S and background infos
  * [build a „LongRange“ Lithium Ion Battery](https://blog.seidel-philipp.de/diy-build-a-longrange-lithium-ion-battery/) 4S 3000mAh, 2020
  * [DIY FPV Goggle Battery Pack](https://nuxnik.com/diy-fpv-goggle-battery-pack/) for googles, with charge meter and 3D case
  * [18650 spaceholder](https://www.printables.com/de/model/1181-18650-improved-spacerholder) 3d print
  * ?Calculation for Configuration
  * ?BMS
* PDB?
* BEC?
* Solar plane
  * [Solar Dragon - Solar Plane Might Be Able To Last Through The Night](https://hackaday.com/2022/08/06/solar-plane-might-be-able-to-last-through-the-night/) plane with beplanned ribs, 2022
  * [rctestflight series](https://www.youtube.com/watch?v=1OGrDvInUAY) 8hrs30 fixed wing full of solar cells with great measurements and background infos

## Motor Control

Usually brushless DC motors (BLDC) are used for their power and precision. They require Electronic Speed Controller (ESC) per motor.

* [BLheli](https://github.com/bitdump/BLHeli) popular Firmware for ESCs with finegrained control
* [BlueJay](https://github.com/mathiasvr/bluejay) Digital ESC firmware for controlling brushless motors in multirotors forked from BLHeli

## Flight Control

Modern autopilot software require STM32 based boards with F4/F7 chip generation and usually no longer support legacy boards like NAZE32, CC3D, ArduPilot, etc. Most projects are base on Cleanflight firmware and Desktop Configurator

* [INAV](https://github.com/light/inav) focus on GPS based flight planning / autonomous flights for wings and copters
* [betaflight](https://github.com/betaflight/betaflight) focus on racing and agility for wings and copters
* [EmuFlight](https://github.com/emuflight/EmuFlight) focus on modern algorithms
* [dRonin](https://github.com/d-ronin/dronin/) supporting Openpilot and other target boards
* [Ardupilot](https://ardupilot.org) full ecosystem including professional / research use for wings and copters and even land or water vehicles. A lot of information, experiences and posibilieties, but also more complex than INAV
* [Drehmflight](https://github.com/nickrehm/dRehmFlight) for Teensy Boards only
* [Paparazzi UAV](https://github.com/paparazzi/paparazzi)
* [LibrePilot](https://github.com/librepilot/LibrePilot) stall since 2018

## RC Transmitters & Handcontroller

Radio control transmitters (RC TX, your side) support extension bays with the [JR / JR lite formfactor](https://github.com/pascallanger/DIY-Multiprotocol-TX-Module/blob/master/docs/Module_BG_4-in-1.md) and serial interfaces to adapt to different radio protocols. See also Ground station.  
Most receivers (RX, drone side) support standard serial protocols like Crossfire (CSRF) to talk with the FC.

* [EdgeTX](https://github.com/EdgeTX/edgetx) sucessor of OpenTX under active development
* [OpenTX](https://github.com/opentx/opentx) firmware for popular handtransmitters including Desktop manager and sound packs
* [inav-opentx-sounds](https://github.com/JyeSmith/inav-opentx-sounds) addon sounds for modes
* [transmitter-sound-pack](https://inavfixedwinggroup.com/guides/transmitter-models/transmitter-sound-pack/) INAV sounds and complete configs for wings
* [VTx](https://github.com/teckel12/VTx) stripped down betaflight Lua script to controll only your VTX
* [betaflight-tx-lua-scripts](https://github.com/Matze-Jung/betaflight-tx-lua-scripts) extended BF lua script
* [opentx-lua-widgets](https://github.com/Matze-Jung/opentx-lua-widgets) more UI widgets to present telemetry
* [opentx-lua-running-graphs](https://github.com/Matze-Jung/opentx-lua-running-graphs) more visual graphs as widgets
* [OpenTX-Pong](https://github.com/SpechtD/OpenTX-Pong) simple game for your TX
* [OpenAVRc](https://github.com/Ingwie/OpenAVRc_Hw) Custom TX based on Arduino Mega2560 boards

### Modules

Hardware and firmware to establish custom radio links. Nowadays usuallay a bidirectional link, so pure TX / RX side

* [Multi Module](https://github.com/pascallanger/DIY-Multiprotocol-TX-Module) Supports different protocols like FrSky, FlySky, Walkera, Futaba, ...
* [ExpressLRS](https://github.com/ExpressLRS/ExpressLRS) ELRS for long range or better latency. Support flashing some existing hardware, but also provide commercial modules for 868/915 MHz or 2.4 / 5.8 GHz.
* [openLRSng](https://github.com/openLRSng/openLRSng)
* [Raven LRS](https://github.com/RavenLRS/raven) Lora based, 2019
* [OpenSky](https://fishpepper.de/projects/opensky/) alternative firmware for FrSky modules, 2016
* [DeviationTX](https://deviationtx.com/) alternative firmware for Walkera, 2016

## VTX

Videotransmitter (VTX) are analog or digital radio transmitter on your drone, which send usually a videostream of your front camera (first person view - FPV) but can also transmit arbitary information or establish a somewhat regulard bidiractional link between the drone and a groundstation which also offers control uplink etc. See also Ground station.

* [OpenHD](https://github.com/OpenHD/Open.HD) Use 2.4 / 5.8 GHz wifi hardware and SBCs on air and groundside to provide a video and telemetry downlink and an optional control uplink. Try to develop a more efficient dedicated hardware board.
* [RubyFPV](https://rubyfpv.com) Use 2.4 / 5.8 GHz wifi hardware and RPIs to provide a video and telemetry downlink and an optional control uplink. No source provided but plugin system
* [Wifibroadcast NG](https://github.com/svpcom/wifibroadcast) Use 2.4 / 5.8 GHz wifi hardware and RPIs to provide a video and telemetry downlink
* [DroneBridge](https://github.com/DroneBridge/DroneBridge) Use 2.4 GHz wifi hardware and RPIs, ESP32 and Android App for bidirectional link, [Comparison](https://dronebridge.gitbook.io/docs/comparison) to the other protocolls here
* [EZ Wifibroadcast](https://github.com/rodizio1/EZ-WifiBroadcast) oldest first wifi
* [wtfos](https://github.com/fpv-wtf/wtfos) rooting and mod DJI FPV sender and receiver
* [DigiView-SBC](https://github.com/fpvout/DigiView-SBC) receive DJI HD signal, alpha from 2021
* [OpenVTx](https://github.com/OpenVTx/OpenVTx) free firmware for one dedicated anlog VTX

## Camera & Gimbals

Cameras feed the onboard videotransmitter for downlink or record as DVR with higher quality. See also VTX for custom systems which allow you different camera setups.

* [Gyroflow](https://github.com/gyroflow/gyroflow) use IMU sensor data to smooth HD video recordings
* [OpenHD on thermal cameras](https://openhd.gitbook.io/open-hd/hardware/cameras)
* [RC Headtracker](https://github.com/dlktdr/HeadTracker) turn camera gimbal when you turn your googles. Based on Arduino and Bluetooth
* [STORM32BGC](https://github.com/olliw42/storm32bgc) Firmware and brushless gimbal controller
* [Open Brushless Gimbal](https://www.thingiverse.com/thing:110731) from 2013

## GPS

Global Navigation Systems like GPS allow to determine the current position of your drone. Consumer GPS modules are cheap, but some can be tweaked to gain better accuracy on postprcessing or live.

* [GNSS SDR](https://gnss-sdr.org) Software toolchain to process radio signals of GPS, Baidu, GLONASS that are received by SDR hardware radio backends
* [rtklib](https://www.rtklib.com) Software toolchain to inrecease precission of GNSS signals by appling realtime or postprocessing to eliminate interferances. Signals recorded by SDR or some commercial GPS modules

## Sensors

A drone uses more sensors like compass, barometer, airspeed or current to increase position estimation, or to show the system performance

* See [Ardupilot - Optional hardware](https://ardupilot.org/copter/docs/common-optional-hardware.html) for ideas

## Video Receivers

Googles have modul bays to support different radio protocols or HDMI input. See also VTX for custom systems which allow you different camera setups.

* [DIY Homemade FPV Monitor](https://hackaday.io/project/160893-diy-homemade-fpv-monitor) 5,8GHz analog display with diversity
* [FENIX-rx5808-pro-diversity](https://github.com/JyeSmith/FENIX-rx5808-pro-diversity) 5,8GHz analog module with diversity for googles
* [rpi-rx5808-stream](https://github.com/xythobuz/rpi-rx5808-stream) RPI based 5,8GHz analog with diversity streaming server

## Antennas and Trackers

Transceiver as well as receivers can be tweaked with custom antenna configurations and tracker can support directional antennas. They use multiple receivers and diversity, or telemetry to point to your drone.
This is more professional equipment to do beyond VLOS flights and not needed for novice pilots. There are also approaches to use 4G for video and control link for extra extend

* [u360gts](https://github.com/raul-ortega/u360gts/) 360° motor tracker using F2/F3 controllers that control, firmware + hardware + case 2020
* [AntTracker](https://github.com/zs6buj/AntTracker) servo based using F1 / ESP8266 / ESP32 controllers, 2019
* [open360tracker v2](https://www.thingiverse.com/thing:2568906) simplified design, all components in moving head
* [open360tracker](https://github.com/SamuelBrucksch/open360tracker) 360° servo tracker 2016
  * [Amv-open360tracker](https://github.com/raul-ortega/amv-open360tracker) fork 2016
  * [Amv-open360tracker 36bit](https://github.com/ericyao2013/amv-open360tracker-32bits) fork 2016
* [Ghettostation Antenna Tracker](https://www.thingiverse.com/thing:547358) different forks 2014
* helical
* clover leaf

## Telemetry & Logs

Sensor values and control information are shared via common serial protocols which can be recorded onboard (blackbox) on internal SD cards at FC, or transfered to your TX handset or Groundstation. Useful to find lost drones, as well as debug and tune PID and flight behaviour

* [MAVlink](https://github.com/mavlink/mavlink) modern extensible protocol from hobbiests ... commercial UAV
* [Cyphal](https://opencyphal.org) fka. UAVCAN industrial only drone bus system
* [YAMSPy](https://github.com/thecognifly/YAMSPy) read MSP serial protocol with Python
* [LuaTelemetry](https://github.com/teckel12/LuaTelemetry) OpenTX / EdgeTX script that renders live cockpit and map from telemetry datastream
* [betaflight-tx-lua-scripts](https://github.com/betaflight/betaflight-tx-lua-scripts) script to show telemetry and control e.g. CAM, VTX settings
* [otxtelemetry](https://github.com/olliw42/otxtelemetry) OpenTX / EdgeTX script to add Mavlink support
* [INAV blackbox viewer](https://github.com/iNavFlight/blackbox-log-viewer) Render sensor / motor values as video overlay OSD
* [INAV blackbox tools](https://github.com/iNavFlight/blackbox-tools) Convert to CSV timeseries files or as visual OSD overlay
* [flightlog2x](https://github.com/stronnag/bbl2kml) Convert blackbox logs of INAV, OpenTX, ... to CSV, GPX, KML and render tracks and trajectory with different performance styles, seperate [GUI](https://github.com/stronnag/fl2xui)
* [UAVLogViewer](https://github.com/ardupilot/uavlogviewer) web application for Ardupilot logs
* [OSD-subtitles](https://github.com/kristjanbjarni/osd-subtitles) render Blackbox logs to OSD as subtitle for synconous plaback with video file
* [openXsensor](https://github.com/openXsensor/openXsensor) Convert and alter telemetry protocols
* [OpenLog](https://github.com/sparkfun/OpenLog) with [blackbox](https://github.com/thenickdude/blackbox/) firmware for blackbox data recorder (today usually part of main FC)

## Mission Control & Basestation

Ground Control Stations (GCS) on laptops / tablets allow better overview of flight parameters and position for mission control during long range / time flights. Also see [Ardupilot.org - Choosing GCS](https://ardupilot.org/copter/docs/common-choosing-a-ground-station.html).

* [mwptools](https://github.com/stronnag/mwptools) mission planner esp. for INAV including INAV Radar and ADS-B sources
* [APM Planner 2.0](https://ardupilot.org/planner2/) Mavlink compatible, with experience from MP and Q Groundcontrol
* [QGroundControl](https://github.com/mavlink/qgroundcontrol) Mavlink, Desktop and mobile
* [MissionPlanner](https://ardupilot.org/planner/index.html)
* [MAVProxy](https://ardupilot.org/mavproxy/) commandline and GUI mission planer and telemetry viewer and processor
* [BulletGCSS](https://github.com/danarrib/BulletGCSS) uses GSM and MQTT for extra long range links
* [Dreka GCS](https://github.com/Midgrad/Dreka) a new GCS (currently limited but more modern look & feel)

## Companion Computers & Integration

Your FC is focused on realtime control for maneuvers, while additional computers add ressources for more complex data processing, see also [Arudpilot.org - Companion Computers](https://ardupilot.org/dev/docs/companion-computers.html) and digital VTX systems above.

* [APsync](https://ardupilot.org/dev/docs/apsync-intro.html) mavlink focused OS for different SBCs
* [RPanion](https://www.docs.rpanion.com/software/rpanion-server) mavlink focused RPI image
* [ROS](https://github.com/ros/ros) robot operating system, to handle more complex and interactive flights
* [DroneKit](https://github.com/dronekit/dronekit-python) multi platform integration ecosystem including mavlink radio link

## Computer Vision

CV is on the processing of what you see at the UAV live image or recordings. This is about aerial mapping, or machine learning powered flight planning. See also [UAV Mapping Guidelines](https://uav-guidelines.openaerialmap.org/).

* [OpenDroneMap](https://www.opendronemap.org/) stitch photos to aerial imagery, calc 3D models, ....
* [OpenAerialMap](https://github.com/hotosm/OpenAerialMap/) share Drone shots for disaster response etc
* [DroneDB](https://github.com/DroneDB/DroneDB) store and archive drone shots and aerial imagery
* [OpenMMS](https://www.openmms.org/) mobile mapping system that carries a laser scanner
* [BANet](https://github.com/lironui/BANet) ML segmentation of areas for aerial imagery
* [AVCBet](https://github.com/lironui/ABCNet) ML segmentation of areas for aerial imagery
* [Faster](https://github.com/mit-acl/faster) ML let drones learn to avoid obstacles
* [Fast-Planner](https://github.com/HKUST-Aerial-Robotics/Fast-Planner) learn drones to avoid obstacles on the course
* [Autonomous Drone Dodges Obstacles Without GPS](https://hackaday.com/2021/11/03/autonomous-drone-dodges-obstacles-without-gps/) RPI based CV and route planning and obstacle avoidance
* [Drone-net](https://github.com/chuanenlin/drone-net) ML detect quadcopters within photos / videos using YOLO v4
* [Fire Detection UAV](https://github.com/AlirezaShamsoshoara/Fire-Detection-UAV-Aerial-Image-Classification-Segmentation-UnmannedAerialVehicle) ML learn drones to spot fire
* [DroneAid](https://github.com/Call-for-Code/DroneAid) ML find persons in disaster response by emergency markers
* [AirPose](https://github.com/robot-perception-group/AirPose) ML human pose estimation from drone perspective



## Complete Systems

Dedicated drones and toolchains to help on a dedicated topic

* [SearchWing](https://www.hs-augsburg.de/searchwing/de/willkommen/) A Search and Rescue RC airplane for visual inspection of huge areas on sea to rescue people from refugee boats at the EU sea border. Waterproof to land beside the SAR mothership.
* [Dronecoria](https://dronecoria.org) octocopter made from wood as heavy lift to drop seeds
* [Crazyflie](https://www.bitcraze.io/documentation/system/platform/) a not so FPV drone that uses custom modules and different technology for swarm control
* [Wifree-copter](https://open-diy-projects.com/wifree-copter/) easy 3D printed copter that uses RPI as WIFI remote control with App

## Security & Safety

### Simulators

Before you damage any hardware you can learn to avoid common pitfalls and train by virtual flights using your handhelt TX. Other allow to test / benchmark autopilots to work in controlled environments.

The most consumer friendly trainer simlulators are commercial, but available for Linux, MacOS as well ([Freerider Recarged](https://fpv-freerider.itch.io/fpv-freerider-recharged), [Liftoff](https://store.steampowered.com/app/410340/Liftoff_FPV_Drone_Racing/), [DRL Sim](https://thedroneracingleague.com/drlsim/), [Velocidrone](https://www.velocidrone.com/)

* [crrcsim](https://sourceforge.net/projects/crrcsim/) for rc airplanes, 2018
* FlightGear - usually for big airplanes, but can be paired with FC good description [from PaparazziUAV](https://wiki.paparazziuav.org/wiki/FlightGear) or [by Arduplane](https://ardupilot.org/dev/docs/simulation-2.html) for simulation
* [AirSim](https://github.com/microsoft/AirSim) by Microsoft for algorithm testing
* [jMAVSim](https://github.com/PX4/jMAVSim) for MAVlink
* [JSBsim](https://github.com/JSBSim-Team/jsbsim) with bindings to Python, Matlab
* [GAZEBOsim](https://github.com/gazebosim/gz-sim) multi robot
* ROS supports simulations like described [by PX4](https://docs.px4.io/master/en/ros/ros2_comm.html)

### Checklists

Malfunction can have dramatic consequences, as well as your drone can cause massive damages. To avoid unnessesary risks, a step by step protocol and documentation is mandatory for every flight in case you might use your insurance.

* build power check
* maiden flight check
* regular flight check
  * [Ardupilot Copter Checklist](https://ardupilot.org/copter/docs/checklist.html)

### ID Systems

We share the airspace with other pilots. RC copters and planes are hard to spot, so it is recommend to share your position via transponder systems. This allows also tracking of illegal maneuvers.

* ADS-B transmittters of airplanes can be received using software defined radios can be realized using cheap USB DVB-T receivers. Can be integrated via radar extensions like [mwp-radar-view](https://github.com/stronnag/mwptools/wiki/mwp-Radar-View), [Ardupilot ADS-B receiver](/https://ardupilot.org/copter/docs/common-ads-b-receiver.html) or in OpenHD. Is part of mavlink protocol and shows up on most GCS.  Can also be viewed online like [adsb-exchange.com](https://globe.adsbexchange.com/)
* [INAV Radar](https://github.com/OlivierC-FR/ESP32-INAV-Radar) LORA radio and ESP32 broadcast positions and show it at your OSD
* [SoftRF](https://github.com/lyusupov/SoftRF) UAV edition, supports also FLARM and more
* [Glidernet](https://www.glidernet.org) share FLARM / ADS-B positions online
* [Opensky Network](https://opensky-network.org) share ADS-B positions online

## Hacking & Hijacking

Radio links per se are unsave and can easily be jammed.

* [5.8GHz video demodulation](https://www.youtube.com/watch?app=desktop&v=rl8ACNnjPFA) using hack-rf SDR
* [GPS jamming](https://www.researchgate.net/publication/339824302_Effective_GPS_Jamming_Techniques_for_UAVs_Using_Low-Cost_SDR_Platforms) or [GPS spoofing](https://rnl.ae.utexas.edu/images/stories/files/papers/unmannedCapture.pdf)
* [Robot Vulnerability Database](https://github.com/aliasrobotics/RVD) CVEs for semi-autonomous machines

## Accesoirs

With 3D printing it is easy to get usefull addons for your equipment and models

* [Delta 5 race timer](https://github.com/scottgchin/delta5_race_timer) use 5.8GHz video signals to trigger lap counter
* [Capture The Flag for drones](https://github.com/SeekND/CaptureTheFlag) optical system to emulate a flag for close team-fights
* Gimbal protection
* holder & stands
* actioncam mounts
* rotor guards
* ...

### Mobile Apps

Free and useful applications to use on your mobile device. Might be not nessesarry open source


* [SpeedyBee}(https://www.speedybee.com/speedy-bee-app/) - is an Appllication for Flight controller parameters settings and blackbox log reader for betaflight, iNAV, emuflight  [Android](https://play.google.com/store/apps/details?id=com.runcam.android.runcambf), [iOS](https://apps.apple.com/us/app/speedybee-app/id1150315028)
* [BLHeli_32](https://play.google.com/store/apps/details?id=org.blheli.BLHeli_32) - app is for configuring BLHeli_32 ESCs.
* [FPV Video Channelsorter 5.8GHz](https://play.google.com/store/apps/details?id=florian.felix.flesch.fpvvideochannelsorter) - Sorts the channels for each pilot over the available frequencies.
* [UAV Forecast](https://www.uavforecast.com) - See the weather forecast, GPS satellites, solar activity (Kp), No-Fly Zones and flight restrictions [Android](https://play.google.com/store/apps/details?id=com.uavforecast), [iOS](https://apps.apple.com/us/app/uav-forecast/id1050023752)
* [Go FPV](https://play.google.com/store/apps/details?id=com.vertile.fpv3d) - A UVC Video Camera display and capture app, built for my DIY First Person View goggles.

## Workbench

* [smoke stopper](https://oscarliang.com/smoke-stopper/) avoid damaging your components during build up
* [4AxisFoamCutter](https://github.com/rahulsarchive/4AxisFoamCutter) create aerodynamic wings from foam

## Legal Information

Depending on the country you live in, the laws and legal rules of airspace may vary.

* [Luftfahrt Bundesamt](https://www.lba.de/DE/Drohnen/Drohnen_node.html) 🇩🇪

## Communities

Every hobby makes more fun and progress if you share your ideas and questions to others. Beside the UAV pilot communities there are plenty of modder and hackers out there.

* [Dronecode foundation](https://www.dronecode.org) home for MavLink, QGroundcontrol and PX4, part of Linux foundation
* [FPV Freedom Coalation](https://fpvfc.org/) keep drones hackabel and safe
* [Deutscher Modellflieger Verband e.V.](https://www.dmfv.aero) 🇩🇪 events, local communities, assurance, ...
* [Deutscher Aero Club e.V.](https://www.daec.de) 🇩🇪

### Forums & Social Media

* [rcroups.com](http://rcroups.com) most projects offere here support
* [diydrones.com](http://diydrones.com) group to most projects, hardware and countries
* [rotorbuilds.com](http://rotorbuilds.com) recipes to custom builds
* [openrcforums.com](http://openrcforums.com) from the old days till the present working on open modells
* [Stackexchange Drones](https://drones.stackexchange.com/) is stackoverflow Q&A for drone building
* [reddit \\motorcopter](https://www.reddit.com/r/Multicopter/) Everything from flights, crashes, repairs, custom mods, ...
* [OscarLiang.com](http://OscarLiang.com) - important blog with builds and configurations and knowledge
* [intofpv.com](http://intofpv.com) - Forum with helpful information about anything FPV related.
* [INAV fixed wing group](https://inavfixedwinggroup.com/) - Forum, Blog, Builds on fixed wing esp. with INAV compatible autopilot
* [fpv-community.de](http://fpv-community.de) [🇩🇪](https://emojipedia.org/de/flagge-deutschland/) auch Selbstbau
* [RC-Network.de](http://RC-Network.de) [🇩🇪](https://emojipedia.org/de/flagge-deutschland/) über Selbstbau auch von Schiffen und Autos. Sehr umfangreiches [Wiki](https://wiki.rc-network.de/wiki/Hauptseite)

### Video Channels

* [Painless 360](https://www.youtube.com/c/Painless360) UK builds, mods and configuration basics
* [ArxangelRC](https://www.youtube.com/c/ArxangelRC) BG builds and configurations as well as (a bit) mapping
* [Joshua Bardwell](https://www.youtube.com/c/JoshuaBardwell) US, build and general tipps on copters 'You gonna learn something today'
* [PawelSpechalski](https://www.youtube.com/c/Pawe%C5%82Spychalski) INAV core team, mostly copters 'Happy Flying'
* [Andrew Netwon](https://www.youtube.com/c/AndrewNewtonAustralia) AU, mostly airplane reviews and build tipps
* [CurryKitten](https://www.youtube.com/c/CurryKitten/) reviews, but also OpenHD and ExpressLRS
* [MarioFPV](https://www.youtube.com/channel/UCX2UiZjg485tDoq_Yl4Pysw) - OpenHD, RubyFPV, WFG-NG experiments
* [TreeOrbit ](https://www.youtube.com/user/montreetormee)- OpenHD, RubyFPV experiments
* [flitetest.com](https://flitetest.com) - TV show, unusual DIY builds
* [Livyu FPV](https://www.youtube.com/c/LivyuFPV/videos) - Flight footages and repair videos for DIY drone electronics
* [Adam G does FPV](https://www.youtube.com/c/AdamGdoesFPV) - builds, mods and basics

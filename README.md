ev3dev-pru-firmware
===================

This is the PRU firmware source code for ev3dev devices:

* FatcatLab EVB: Provides I2C for input ports.


About
-----

PRU stands for Programmable Runtime Unit. TI processors such as the AM18xx in
LEGO MINDSTORMS EV3 and AM335x contain two mini processor cores that run
independent of the main processor. These are useful for implementing real-time
systems. However, they have limited resources.


Hacking
-------

This repository has a submodule, so to clone...

    git clone --recursive https://github.com/ev3dev/ev3dev-pru-firmware

This requires [TI CSS6](http://processors.wiki.ti.com/index.php/Download_CCS)
with PRU Compiler Tools 2.1.2 to build. (Use the web installer to install
the minimum required components.)

Once CSS6 is installed, run it and open a workspace using the directory where
you cloned the git repository. Then press <key>CTRL</key> + <key>B</key> to
build.


Resources
---------

EV3 PRU:

* <http://processors.wiki.ti.com/index.php/Programmable_Realtime_Unit_Subsystem>

BeagleBone PRU:

* <http://processors.wiki.ti.com/index.php/PRU-ICSS>
* <https://github.com/beagleboard/am335x_pru_package> (outdated)

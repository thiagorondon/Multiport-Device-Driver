=============================================================================
      Multiport Device Driver Installation Guide
		    for Linux Kernel 2.4.x, 2.6.x
=============================================================================
Date: 2009-06-22

Content

1. Introduction
2. System Requirement
3. Installation
4. Utilities
-----------------------------------------------------------------------------

1. introduction 
 
   This driver and installation procedure have been developed upon Linux Kernel
   2.4.20 or 2.6.x. This driver supports Intel x86 hardware platform. In order 
   to maintain compatibility, this version has also been properly tested with 
   RedHat, Fedora.
   
   All the drivers and utilities are published in form of source code under
   GNU General Public License in this version. Please refer to GNU General
   Public License announcement in each source code file for more detail.

-----------------------------------------------------------------------------
2. System Requirement
   - Hardware platform: Intel x86 machine
   - Kernel version: 2.4.20 or 2.6.x
   - Tested Linux : Redhat 9.0, fedora 2/3/4/5/6/7
   - gcc version 2.72 or later

-----------------------------------------------------------------------------
3. Installation
  Before installation, kernel source file should installed in you Linux PC.
  
  3.1 Device Driver Installation 
  
  first of all, to install this driver, download the compressed file-MP_DRIVER_2.X.tar.gz
  to your home folder.
  # tar xvfz MP_DRIVER_2.X.tar.gz
  # cd MP_DRIVER_2.X
  # ./Install
  # ls /dev/ttyMP*
  you can check Serial Device node.
  
  To uninstall the driver 
  # ./Remove

   
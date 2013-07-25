/*
 *  propulsion-interface.h
 *
 *  BRIEF MODULE DESCRIPTION
 *      User space invocation interface definition for the LegoSNAPS propulsion
 *      system.
 *
 *  Copyright (C) 2012 RAF Research, LLC.
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *  THIS  SOFTWARE  IS PROVIDED   ``AS  IS'' AND   ANY  EXPRESS OR IMPLIED
 *  WARRANTIES,   INCLUDING, BUT NOT  LIMITED  TO, THE IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN
 *  NO  EVENT  SHALL   THE AUTHOR  BE    LIABLE FOR ANY   DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 *  NOT LIMITED   TO, PROCUREMENT OF  SUBSTITUTE GOODS  OR SERVICES; LOSS OF
 *  USE, DATA,  OR PROFITS; OR  BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 *  ANY THEORY OF LIABILITY, WHETHER IN  CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 *  THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *	The LegoSNAPS controller is connected to four Quad-Thruster Pods.
 *	Each Quad-Thruster Pod is on a branch of the Controller's i2c3 bus
 *	(i2c bus #3). Linux manages the bus multiplexer chip that creates these
 *	branches. Each branch of the bus is given it own name under Linux's
 *	/dev directory. They are accessed as i2c busses 4 through 11.
 *	Buses 4-7 are for SNAPS Thruster Ports. Buses 8-11 are for Lego Input
 *	Ports.
 *
 *	The LegoSNAPS satellite has the below layout.
 *	               +X
 *	             +----+
 *	             |    |
 *	        +----+    +----+
 *	      T7|              |T6
 *	        |              |
 *	    -Y  |              |  +Y
 *	        |              |
 *	      T4|              |T5
 *	        +----+    +----+
 *	             \    /
 *	              \  /
 *	               ++
 *	               -X
 *	Each Quad-Thruster Pod (T4-T7) is numbered with the i2c bus that controls it.
 *	The BCS axises are shown.
 *
 *	Each Quad-Thruster Pod has four thrusters designated 1-4.
 *	Thruster 1 fires through the side opposite the input connector.
 *	Thruster 2 fires through the Thruster's PCB.
 *	(Orient the Thruster Pod so that you see the PCB and Thruster 1 is on the
 *	left side.)
 *	Thruster 3 fires upward and Thruster 4 fires downward.
 *
 *	The SNAPS Propulsion Interface defines four virtual thrusters.
 *	One on the +Y axis that can produce thrust in either the + or - X-axis
 *		directions. Designated "x_thrust_yp".
 *	One on the -Y axis that can produce thrust in either the + or - X-axis
 *		directions. Designated "x_thrust_yn".
 *	One on the +X axis that can produce thrust in either the + or - Y-axis
 *		directions. Designated "y_thrust_xp".
 *	One on the -X axis that can produce thrust in either the + or - Y-axis
 *		directions. Designated "y_thrust_xn".
 *
 *	Each of these virtual thrusters is controlled by giving it an "on-duration"
 *	value in units of milliseconds. A value of 0 commands the thruster not
 *	to fire. A value  of 800 commands the thruster to fire for 800 ms
 *	generating thrust in the positive direction. A value  of -800 commands the
 *	thruster to fire for 800 ms generating thrust in the negative direction.
 *
 *	The mapping of virtual thrusters to Quad-Thruster Pod thrusters is:
 *	x_thrust_yp positive values	==>	T5-2
 *	x_thrust_yp negative values	==>	T6-2
 *	x_thrust_yn positive values	==>	T4-2
 *	x_thrust_yn negative values	==>	T7-2
 *	y_thrust_xp positive values	==>	T7-1
 *	y_thrust_xp negative values	==>	T6-1
 *	y_thrust_xn positive values	==>	T4-1
 *	y_thrust_xn negative values	==>	T5-1
 *	Currently, there is no mapping provided for Z-axis thrusters.
 */
#ifndef PROPULSION_INTERFACE_H
#define PROPULSION_INTERFACE_H
/*
 * User interface to the LegoSNAPS propulsion system.
 */
#include <linux/ioctl.h>
#include <linux/types.h>

/*
 *	prototype definitions
 */
extern int	propulsion_ifce_open();
extern void	propulsion_ifce_close();
extern int	propulsion_ifce_fire(
	int x_thrust_yp,		// positive Y-axis thruster milliseconds
	int x_thrust_yn,		// negative Y-axis thruster milliseconds
	int y_thrust_xp,		// positive X-axis thruster milliseconds
	int y_thrust_xn 		// negative X-axis thruster milliseconds
);
/*
 *	return code =
 *		0	= command accepted and being executed.
 *		0x80000001	= Can't open i2c bus branch 4
 *		0x80000002	= No device on i2c bus branch 4 is responding.
 *		0x80000004	= Device on i2c bus branch 4 is not a Quad-Thruster Pod.
 *		0x80000008	= Command to Quad-Thruster Pod 4 succeeded after reset.
 */
#define	THRUSTER_ERROR	0x80000000
#define THRUSTER_NOGPIO 0x40000000
#define	THRUSTER_NOBUS 	0x01
#define	THRUSTER_NODEV 	0x02
#define THRUSTER_NOTSUP 0x04
#define THRUSTER_RESET  0x08
#define THRUSTER_BUS4 0
#define THRUSTER_BUS5 4
#define THRUSTER_BUS6 8
#define THRUSTER_BUS7 12

#endif /* THRUSTER_INTERFACE_H */

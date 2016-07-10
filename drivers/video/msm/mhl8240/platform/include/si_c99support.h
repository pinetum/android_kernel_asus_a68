/*

SiI8240 Linux Driver

Copyright (C) 2011-2012 Silicon Image Inc.

This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License as
published by the Free Software Foundation version 2.

This program is distributed .as is. WITHOUT ANY WARRANTY of any
kind, whether express or implied; without even the implied warranty
of MERCHANTABILITY or FITNESS FOR A PARTICULAR
PURPOSE.  See the
GNU General Public License for more details.

*/

/*
   @file si_c99support.h
 */

#ifndef __SI_C99SUPPORT_H__
#define __SI_C99SUPPORT_H__


/*
This file is a place holder for C99 data types.
Since the GNU compiler is inherently C99 compliant, little is necessary here
However, this file must remain in the source tree to satisfy the #include directives
in the component, driver, and application modules.
*/

#include <linux/kernel.h>


// Emulate C99/C++ bool type to support the large amount of code that
// has yet to be ported to use bool.
typedef bool bool_t;

#ifndef __KERNEL__
# ifndef __intptr_t_defined
typedef int				intptr_t;
typedef unsigned int	uintptr_t;
#  define __intptr_t_defined
# endif
#endif

//typedef char BOOL;

#endif  // __SI_C99SUPPORT_H__


##
##	This file is part of qpOASES.
##
##	qpOASES -- An Implementation of the Online Active Set Strategy.
##	Copyright (C) 2007-2014 by Hans Joachim Ferreau, Andreas Potschka,
##	Christian Kirches et al. All rights reserved.
##
##	qpOASES is free software; you can redistribute it and/or
##	modify it under the terms of the GNU Lesser General Public
##	License as published by the Free Software Foundation; either
##	version 2.1 of the License, or (at your option) any later version.
##
##	qpOASES is distributed in the hope that it will be useful,
##	but WITHOUT ANY WARRANTY; without even the implied warranty of
##	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
##	See the GNU Lesser General Public License for more details.
##
##	You should have received a copy of the GNU Lesser General Public
##	License along with qpOASES; if not, write to the Free Software
##	Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
##



##
##	Filename:  make_linux.mk
##	Author:    Hans Joachim Ferreau, Andreas Potschka, Christian Kirches
##	Version:   3.0beta
##	Date:      2007-2014
##

################################################################################
# user configuration

# include directories, relative
IDIR = ${TOP}/include
SRCDIR = ${TOP}/src
BINDIR = ${TOP}/bin

# Matlab include directory (ADAPT TO YOUR LOCAL SETTINGS!)
#MATLAB_IDIR   = ${HOME}/Programs/matlab/extern/include/
MATLAB_IDIR = /usr/local/matlab/extern/include/
MATLAB_LIBDIR = /usr/local/matlab/bin/glnxa64/


# system or replacement BLAS/LAPACK
REPLACE_LINALG = 1

ifeq ($(REPLACE_LINALG), 1)
	LIB_BLAS =   ${SRCDIR}/BLASReplacement.o
	LIB_LAPACK = ${SRCDIR}/LAPACKReplacement.o
else
	LIB_BLAS =   /usr/lib/libblas.so
	LIB_LAPACK = /usr/lib/liblapack.so
	#LIB_BLAS =   /usr/lib/lapack/cygblas-0.dll
	#LIB_LAPACK = /usr/lib/lapack/cyglapack-0.dll
endif

################################################################################
# do not touch this

CPP = g++
AR  = ar
RM  = rm
F77 = gfortran
ECHO = echo
CD = cd
CP = cp

# file extensions
OBJEXT = o
LIBEXT = a
DLLEXT = so
MEXOCTEXT = mex
DEF_TARGET = -o $@

# 32 or 64 depending on target platform
BITS = $(shell getconf LONG_BIT)

# decide on MEX interface extension
ifeq ($(BITS), 32)
	MEXEXT = mexglx
else
	MEXEXT = mexa64
endif

CPPFLAGS = -Wall -pedantic -Wshadow -Wfloat-equal -O3 -finline-functions -fPIC -DLINUX
#          -g -D__DEBUG__ -D__NO_COPYRIGHT__ -D__SUPPRESSANYOUTPUT__ -D__USE_SINGLE_PRECISION__

FFLAGS = -Wall -O3 -fPIC -DLINUX -Wno-uninitialized
#        -g

# libraries to link against when building qpOASES .so files
LINK_LIBRARIES = ${LIB_LAPACK} ${LIB_BLAS} -lm
LINK_LIBRARIES_AW = ${LIB_LAPACK} ${LIB_BLAS} -lm -lgfortran -lhsl_ma57 -lfakemetis

# how to link against the qpOASES shared library
QPOASES_LINK = -L${BINDIR} -Wl,-rpath=${BINDIR} -lqpOASES
QPOASES_AW_LINK = -L${BINDIR} -Wl,-rpath=${BINDIR} -lqpOASES_aw

##
##	end of file
##

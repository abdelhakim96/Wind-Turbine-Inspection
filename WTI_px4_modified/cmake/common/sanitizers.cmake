############################################################################
#
# Copyright (c) 2017 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

option(SANITIZE_ADDRESS "Enable AddressSanitizer" Off)
option(SANITIZE_MEMORY "Enable MemorySanitizer" Off)
option(SANITIZE_THREAD "Enable ThreadSanitizer" Off)
option(SANITIZE_UNDEFINED "Enable UndefinedBehaviorSanitizer" Off)

if(DEFINED ENV{PX4_ASAN})
	set(SANITIZE_ADDRESS ON)
elseif(DEFINED ENV{PX4_MSAN})
	set(SANITIZE_MEMORY ON)
elseif(DEFINED ENV{PX4_TSAN})
	set(SANITIZE_THREAD ON)
elseif(DEFINED ENV{PX4_UBSAN})
	set(SANITIZE_UNDEFINED ON)
endif()

if (SANITIZE_ADDRESS)
        message(STATUS "AddressSanitizer enabled")

        # environment variables
        #  ASAN_OPTIONS=detect_stack_use_after_return=1
        #  ASAN_OPTIONS=check_initialization_order=1
        add_compile_options(
                -g3
                -fno-omit-frame-pointer
                -fsanitize=address
		#-fsanitize-address-use-after-scope
		-fno-optimize-sibling-calls
        )
        set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fsanitize=address" CACHE INTERNAL "" FORCE)

elseif(SANITIZE_MEMORY)
        message(STATUS "MemorySanitizer enabled")

        add_compile_options(
                -g3
                -fsanitize=memory
        )
	set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fsanitize=memory" CACHE INTERNAL "" FORCE)

elseif(SANITIZE_THREAD)
        message(STATUS "ThreadSanitizer enabled")

        add_compile_options(
                -g3
                -fsanitize=thread
        )
	set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fsanitize=thread" CACHE INTERNAL "" FORCE)

elseif(SANITIZE_UNDEFINED)
        message(STATUS "UndefinedBehaviorSanitizer enabled")

        add_compile_options(
                -g3
                #-fsanitize=alignment
                -fsanitize=bool
		#-fsanitize=builtin
                -fsanitize=bounds
                -fsanitize=enum
                -fsanitize=float-cast-overflow
                -fsanitize=float-divide-by-zero
                #-fsanitize=function
                -fsanitize=integer-divide-by-zero
                -fsanitize=nonnull-attribute
                -fsanitize=null
		#-fsanitize=nullability-arg
		#-fsanitize=nullability-assign
		#-fsanitize=nullability-return
                -fsanitize=object-size
		#-fsanitize=pointer-overflow
                -fsanitize=return
                -fsanitize=returns-nonnull-attribute
                -fsanitize=shift
                -fsanitize=signed-integer-overflow
                -fsanitize=unreachable
                #-fsanitize=unsigned-integer-overflow
                -fsanitize=vla-bound
                -fsanitize=vptr

		-fno-sanitize-recover=bounds,null
        )
	set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fsanitize=undefined" CACHE INTERNAL "" FORCE)

endif()

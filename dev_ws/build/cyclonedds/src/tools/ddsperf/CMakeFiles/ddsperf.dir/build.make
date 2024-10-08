# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.22

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /dev_ws/src/eclipse-cyclonedds/cyclonedds

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /dev_ws/build/cyclonedds

# Include any dependencies generated for this target.
include src/tools/ddsperf/CMakeFiles/ddsperf.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include src/tools/ddsperf/CMakeFiles/ddsperf.dir/compiler_depend.make

# Include the progress variables for this target.
include src/tools/ddsperf/CMakeFiles/ddsperf.dir/progress.make

# Include the compile flags for this target's objects.
include src/tools/ddsperf/CMakeFiles/ddsperf.dir/flags.make

src/tools/ddsperf/ddsperf_types.c: /dev_ws/src/eclipse-cyclonedds/cyclonedds/src/tools/ddsperf/ddsperf_types.idl
src/tools/ddsperf/ddsperf_types.c: bin/idlc
src/tools/ddsperf/ddsperf_types.c: lib/libcycloneddsidlc.so.0.11.0
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/dev_ws/build/cyclonedds/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Generating ddsperf_types.c, ddsperf_types.h"
	cd /dev_ws/build/cyclonedds/src/tools/ddsperf && ../../../bin/idlc -l/dev_ws/build/cyclonedds/lib/libcycloneddsidlc.so.0.11.0 -Wno-implicit-extensibility -o/dev_ws/build/cyclonedds/src/tools/ddsperf /dev_ws/src/eclipse-cyclonedds/cyclonedds/src/tools/ddsperf/ddsperf_types.idl

src/tools/ddsperf/ddsperf_types.h: src/tools/ddsperf/ddsperf_types.c
	@$(CMAKE_COMMAND) -E touch_nocreate src/tools/ddsperf/ddsperf_types.h

src/tools/ddsperf/CMakeFiles/ddsperf.dir/ddsperf.c.o: src/tools/ddsperf/CMakeFiles/ddsperf.dir/flags.make
src/tools/ddsperf/CMakeFiles/ddsperf.dir/ddsperf.c.o: /dev_ws/src/eclipse-cyclonedds/cyclonedds/src/tools/ddsperf/ddsperf.c
src/tools/ddsperf/CMakeFiles/ddsperf.dir/ddsperf.c.o: src/tools/ddsperf/CMakeFiles/ddsperf.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/dev_ws/build/cyclonedds/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building C object src/tools/ddsperf/CMakeFiles/ddsperf.dir/ddsperf.c.o"
	cd /dev_ws/build/cyclonedds/src/tools/ddsperf && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT src/tools/ddsperf/CMakeFiles/ddsperf.dir/ddsperf.c.o -MF CMakeFiles/ddsperf.dir/ddsperf.c.o.d -o CMakeFiles/ddsperf.dir/ddsperf.c.o -c /dev_ws/src/eclipse-cyclonedds/cyclonedds/src/tools/ddsperf/ddsperf.c

src/tools/ddsperf/CMakeFiles/ddsperf.dir/ddsperf.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/ddsperf.dir/ddsperf.c.i"
	cd /dev_ws/build/cyclonedds/src/tools/ddsperf && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /dev_ws/src/eclipse-cyclonedds/cyclonedds/src/tools/ddsperf/ddsperf.c > CMakeFiles/ddsperf.dir/ddsperf.c.i

src/tools/ddsperf/CMakeFiles/ddsperf.dir/ddsperf.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/ddsperf.dir/ddsperf.c.s"
	cd /dev_ws/build/cyclonedds/src/tools/ddsperf && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /dev_ws/src/eclipse-cyclonedds/cyclonedds/src/tools/ddsperf/ddsperf.c -o CMakeFiles/ddsperf.dir/ddsperf.c.s

src/tools/ddsperf/CMakeFiles/ddsperf.dir/cputime.c.o: src/tools/ddsperf/CMakeFiles/ddsperf.dir/flags.make
src/tools/ddsperf/CMakeFiles/ddsperf.dir/cputime.c.o: /dev_ws/src/eclipse-cyclonedds/cyclonedds/src/tools/ddsperf/cputime.c
src/tools/ddsperf/CMakeFiles/ddsperf.dir/cputime.c.o: src/tools/ddsperf/CMakeFiles/ddsperf.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/dev_ws/build/cyclonedds/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building C object src/tools/ddsperf/CMakeFiles/ddsperf.dir/cputime.c.o"
	cd /dev_ws/build/cyclonedds/src/tools/ddsperf && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT src/tools/ddsperf/CMakeFiles/ddsperf.dir/cputime.c.o -MF CMakeFiles/ddsperf.dir/cputime.c.o.d -o CMakeFiles/ddsperf.dir/cputime.c.o -c /dev_ws/src/eclipse-cyclonedds/cyclonedds/src/tools/ddsperf/cputime.c

src/tools/ddsperf/CMakeFiles/ddsperf.dir/cputime.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/ddsperf.dir/cputime.c.i"
	cd /dev_ws/build/cyclonedds/src/tools/ddsperf && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /dev_ws/src/eclipse-cyclonedds/cyclonedds/src/tools/ddsperf/cputime.c > CMakeFiles/ddsperf.dir/cputime.c.i

src/tools/ddsperf/CMakeFiles/ddsperf.dir/cputime.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/ddsperf.dir/cputime.c.s"
	cd /dev_ws/build/cyclonedds/src/tools/ddsperf && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /dev_ws/src/eclipse-cyclonedds/cyclonedds/src/tools/ddsperf/cputime.c -o CMakeFiles/ddsperf.dir/cputime.c.s

src/tools/ddsperf/CMakeFiles/ddsperf.dir/netload.c.o: src/tools/ddsperf/CMakeFiles/ddsperf.dir/flags.make
src/tools/ddsperf/CMakeFiles/ddsperf.dir/netload.c.o: /dev_ws/src/eclipse-cyclonedds/cyclonedds/src/tools/ddsperf/netload.c
src/tools/ddsperf/CMakeFiles/ddsperf.dir/netload.c.o: src/tools/ddsperf/CMakeFiles/ddsperf.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/dev_ws/build/cyclonedds/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building C object src/tools/ddsperf/CMakeFiles/ddsperf.dir/netload.c.o"
	cd /dev_ws/build/cyclonedds/src/tools/ddsperf && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT src/tools/ddsperf/CMakeFiles/ddsperf.dir/netload.c.o -MF CMakeFiles/ddsperf.dir/netload.c.o.d -o CMakeFiles/ddsperf.dir/netload.c.o -c /dev_ws/src/eclipse-cyclonedds/cyclonedds/src/tools/ddsperf/netload.c

src/tools/ddsperf/CMakeFiles/ddsperf.dir/netload.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/ddsperf.dir/netload.c.i"
	cd /dev_ws/build/cyclonedds/src/tools/ddsperf && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /dev_ws/src/eclipse-cyclonedds/cyclonedds/src/tools/ddsperf/netload.c > CMakeFiles/ddsperf.dir/netload.c.i

src/tools/ddsperf/CMakeFiles/ddsperf.dir/netload.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/ddsperf.dir/netload.c.s"
	cd /dev_ws/build/cyclonedds/src/tools/ddsperf && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /dev_ws/src/eclipse-cyclonedds/cyclonedds/src/tools/ddsperf/netload.c -o CMakeFiles/ddsperf.dir/netload.c.s

src/tools/ddsperf/CMakeFiles/ddsperf.dir/async_listener.c.o: src/tools/ddsperf/CMakeFiles/ddsperf.dir/flags.make
src/tools/ddsperf/CMakeFiles/ddsperf.dir/async_listener.c.o: /dev_ws/src/eclipse-cyclonedds/cyclonedds/src/tools/ddsperf/async_listener.c
src/tools/ddsperf/CMakeFiles/ddsperf.dir/async_listener.c.o: src/tools/ddsperf/CMakeFiles/ddsperf.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/dev_ws/build/cyclonedds/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building C object src/tools/ddsperf/CMakeFiles/ddsperf.dir/async_listener.c.o"
	cd /dev_ws/build/cyclonedds/src/tools/ddsperf && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT src/tools/ddsperf/CMakeFiles/ddsperf.dir/async_listener.c.o -MF CMakeFiles/ddsperf.dir/async_listener.c.o.d -o CMakeFiles/ddsperf.dir/async_listener.c.o -c /dev_ws/src/eclipse-cyclonedds/cyclonedds/src/tools/ddsperf/async_listener.c

src/tools/ddsperf/CMakeFiles/ddsperf.dir/async_listener.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/ddsperf.dir/async_listener.c.i"
	cd /dev_ws/build/cyclonedds/src/tools/ddsperf && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /dev_ws/src/eclipse-cyclonedds/cyclonedds/src/tools/ddsperf/async_listener.c > CMakeFiles/ddsperf.dir/async_listener.c.i

src/tools/ddsperf/CMakeFiles/ddsperf.dir/async_listener.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/ddsperf.dir/async_listener.c.s"
	cd /dev_ws/build/cyclonedds/src/tools/ddsperf && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /dev_ws/src/eclipse-cyclonedds/cyclonedds/src/tools/ddsperf/async_listener.c -o CMakeFiles/ddsperf.dir/async_listener.c.s

src/tools/ddsperf/CMakeFiles/ddsperf.dir/ddsperf_types.c.o: src/tools/ddsperf/CMakeFiles/ddsperf.dir/flags.make
src/tools/ddsperf/CMakeFiles/ddsperf.dir/ddsperf_types.c.o: src/tools/ddsperf/ddsperf_types.c
src/tools/ddsperf/CMakeFiles/ddsperf.dir/ddsperf_types.c.o: src/tools/ddsperf/CMakeFiles/ddsperf.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/dev_ws/build/cyclonedds/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building C object src/tools/ddsperf/CMakeFiles/ddsperf.dir/ddsperf_types.c.o"
	cd /dev_ws/build/cyclonedds/src/tools/ddsperf && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT src/tools/ddsperf/CMakeFiles/ddsperf.dir/ddsperf_types.c.o -MF CMakeFiles/ddsperf.dir/ddsperf_types.c.o.d -o CMakeFiles/ddsperf.dir/ddsperf_types.c.o -c /dev_ws/build/cyclonedds/src/tools/ddsperf/ddsperf_types.c

src/tools/ddsperf/CMakeFiles/ddsperf.dir/ddsperf_types.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/ddsperf.dir/ddsperf_types.c.i"
	cd /dev_ws/build/cyclonedds/src/tools/ddsperf && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /dev_ws/build/cyclonedds/src/tools/ddsperf/ddsperf_types.c > CMakeFiles/ddsperf.dir/ddsperf_types.c.i

src/tools/ddsperf/CMakeFiles/ddsperf.dir/ddsperf_types.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/ddsperf.dir/ddsperf_types.c.s"
	cd /dev_ws/build/cyclonedds/src/tools/ddsperf && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /dev_ws/build/cyclonedds/src/tools/ddsperf/ddsperf_types.c -o CMakeFiles/ddsperf.dir/ddsperf_types.c.s

# Object files for target ddsperf
ddsperf_OBJECTS = \
"CMakeFiles/ddsperf.dir/ddsperf.c.o" \
"CMakeFiles/ddsperf.dir/cputime.c.o" \
"CMakeFiles/ddsperf.dir/netload.c.o" \
"CMakeFiles/ddsperf.dir/async_listener.c.o" \
"CMakeFiles/ddsperf.dir/ddsperf_types.c.o"

# External object files for target ddsperf
ddsperf_EXTERNAL_OBJECTS =

bin/ddsperf: src/tools/ddsperf/CMakeFiles/ddsperf.dir/ddsperf.c.o
bin/ddsperf: src/tools/ddsperf/CMakeFiles/ddsperf.dir/cputime.c.o
bin/ddsperf: src/tools/ddsperf/CMakeFiles/ddsperf.dir/netload.c.o
bin/ddsperf: src/tools/ddsperf/CMakeFiles/ddsperf.dir/async_listener.c.o
bin/ddsperf: src/tools/ddsperf/CMakeFiles/ddsperf.dir/ddsperf_types.c.o
bin/ddsperf: src/tools/ddsperf/CMakeFiles/ddsperf.dir/build.make
bin/ddsperf: lib/libddsc.so.0.11.0
bin/ddsperf: src/tools/ddsperf/CMakeFiles/ddsperf.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/dev_ws/build/cyclonedds/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Linking C executable ../../../bin/ddsperf"
	cd /dev_ws/build/cyclonedds/src/tools/ddsperf && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/ddsperf.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/tools/ddsperf/CMakeFiles/ddsperf.dir/build: bin/ddsperf
.PHONY : src/tools/ddsperf/CMakeFiles/ddsperf.dir/build

src/tools/ddsperf/CMakeFiles/ddsperf.dir/clean:
	cd /dev_ws/build/cyclonedds/src/tools/ddsperf && $(CMAKE_COMMAND) -P CMakeFiles/ddsperf.dir/cmake_clean.cmake
.PHONY : src/tools/ddsperf/CMakeFiles/ddsperf.dir/clean

src/tools/ddsperf/CMakeFiles/ddsperf.dir/depend: src/tools/ddsperf/ddsperf_types.c
src/tools/ddsperf/CMakeFiles/ddsperf.dir/depend: src/tools/ddsperf/ddsperf_types.h
	cd /dev_ws/build/cyclonedds && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /dev_ws/src/eclipse-cyclonedds/cyclonedds /dev_ws/src/eclipse-cyclonedds/cyclonedds/src/tools/ddsperf /dev_ws/build/cyclonedds /dev_ws/build/cyclonedds/src/tools/ddsperf /dev_ws/build/cyclonedds/src/tools/ddsperf/CMakeFiles/ddsperf.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/tools/ddsperf/CMakeFiles/ddsperf.dir/depend


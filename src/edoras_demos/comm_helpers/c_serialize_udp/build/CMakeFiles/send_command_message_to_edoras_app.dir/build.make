# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.31

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
CMAKE_COMMAND = /home/ana/.local/lib/python3.12/site-packages/cmake/data/bin/cmake

# The command to remove a file.
RM = /home/ana/.local/lib/python3.12/site-packages/cmake/data/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/ana/ros2/edoras_ws/edoras/src/scratch/c_serialize_udp

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ana/ros2/edoras_ws/edoras/src/scratch/c_serialize_udp/build

# Include any dependencies generated for this target.
include CMakeFiles/send_command_message_to_edoras_app.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/send_command_message_to_edoras_app.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/send_command_message_to_edoras_app.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/send_command_message_to_edoras_app.dir/flags.make

CMakeFiles/send_command_message_to_edoras_app.dir/codegen:
.PHONY : CMakeFiles/send_command_message_to_edoras_app.dir/codegen

CMakeFiles/send_command_message_to_edoras_app.dir/send_command_message_to_edoras_app.c.o: CMakeFiles/send_command_message_to_edoras_app.dir/flags.make
CMakeFiles/send_command_message_to_edoras_app.dir/send_command_message_to_edoras_app.c.o: /home/ana/ros2/edoras_ws/edoras/src/scratch/c_serialize_udp/send_command_message_to_edoras_app.c
CMakeFiles/send_command_message_to_edoras_app.dir/send_command_message_to_edoras_app.c.o: CMakeFiles/send_command_message_to_edoras_app.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/ana/ros2/edoras_ws/edoras/src/scratch/c_serialize_udp/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object CMakeFiles/send_command_message_to_edoras_app.dir/send_command_message_to_edoras_app.c.o"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT CMakeFiles/send_command_message_to_edoras_app.dir/send_command_message_to_edoras_app.c.o -MF CMakeFiles/send_command_message_to_edoras_app.dir/send_command_message_to_edoras_app.c.o.d -o CMakeFiles/send_command_message_to_edoras_app.dir/send_command_message_to_edoras_app.c.o -c /home/ana/ros2/edoras_ws/edoras/src/scratch/c_serialize_udp/send_command_message_to_edoras_app.c

CMakeFiles/send_command_message_to_edoras_app.dir/send_command_message_to_edoras_app.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing C source to CMakeFiles/send_command_message_to_edoras_app.dir/send_command_message_to_edoras_app.c.i"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/ana/ros2/edoras_ws/edoras/src/scratch/c_serialize_udp/send_command_message_to_edoras_app.c > CMakeFiles/send_command_message_to_edoras_app.dir/send_command_message_to_edoras_app.c.i

CMakeFiles/send_command_message_to_edoras_app.dir/send_command_message_to_edoras_app.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling C source to assembly CMakeFiles/send_command_message_to_edoras_app.dir/send_command_message_to_edoras_app.c.s"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/ana/ros2/edoras_ws/edoras/src/scratch/c_serialize_udp/send_command_message_to_edoras_app.c -o CMakeFiles/send_command_message_to_edoras_app.dir/send_command_message_to_edoras_app.c.s

# Object files for target send_command_message_to_edoras_app
send_command_message_to_edoras_app_OBJECTS = \
"CMakeFiles/send_command_message_to_edoras_app.dir/send_command_message_to_edoras_app.c.o"

# External object files for target send_command_message_to_edoras_app
send_command_message_to_edoras_app_EXTERNAL_OBJECTS =

send_command_message_to_edoras_app: CMakeFiles/send_command_message_to_edoras_app.dir/send_command_message_to_edoras_app.c.o
send_command_message_to_edoras_app: CMakeFiles/send_command_message_to_edoras_app.dir/build.make
send_command_message_to_edoras_app: CMakeFiles/send_command_message_to_edoras_app.dir/compiler_depend.ts
send_command_message_to_edoras_app: libserialize_library.a
send_command_message_to_edoras_app: CMakeFiles/send_command_message_to_edoras_app.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --bold --progress-dir=/home/ana/ros2/edoras_ws/edoras/src/scratch/c_serialize_udp/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking C executable send_command_message_to_edoras_app"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/send_command_message_to_edoras_app.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/send_command_message_to_edoras_app.dir/build: send_command_message_to_edoras_app
.PHONY : CMakeFiles/send_command_message_to_edoras_app.dir/build

CMakeFiles/send_command_message_to_edoras_app.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/send_command_message_to_edoras_app.dir/cmake_clean.cmake
.PHONY : CMakeFiles/send_command_message_to_edoras_app.dir/clean

CMakeFiles/send_command_message_to_edoras_app.dir/depend:
	cd /home/ana/ros2/edoras_ws/edoras/src/scratch/c_serialize_udp/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ana/ros2/edoras_ws/edoras/src/scratch/c_serialize_udp /home/ana/ros2/edoras_ws/edoras/src/scratch/c_serialize_udp /home/ana/ros2/edoras_ws/edoras/src/scratch/c_serialize_udp/build /home/ana/ros2/edoras_ws/edoras/src/scratch/c_serialize_udp/build /home/ana/ros2/edoras_ws/edoras/src/scratch/c_serialize_udp/build/CMakeFiles/send_command_message_to_edoras_app.dir/DependInfo.cmake "--color=$(COLOR)"
.PHONY : CMakeFiles/send_command_message_to_edoras_app.dir/depend


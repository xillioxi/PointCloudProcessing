# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
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
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/kevin/PointCloudProcessing

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/kevin/PointCloudProcessing/build

# Include any dependencies generated for this target.
include CMakeFiles/voxel_visualizer.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/voxel_visualizer.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/voxel_visualizer.dir/flags.make

CMakeFiles/voxel_visualizer.dir/voxel_visualizer.cpp.o: CMakeFiles/voxel_visualizer.dir/flags.make
CMakeFiles/voxel_visualizer.dir/voxel_visualizer.cpp.o: ../voxel_visualizer.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/kevin/PointCloudProcessing/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/voxel_visualizer.dir/voxel_visualizer.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/voxel_visualizer.dir/voxel_visualizer.cpp.o -c /home/kevin/PointCloudProcessing/voxel_visualizer.cpp

CMakeFiles/voxel_visualizer.dir/voxel_visualizer.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/voxel_visualizer.dir/voxel_visualizer.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/kevin/PointCloudProcessing/voxel_visualizer.cpp > CMakeFiles/voxel_visualizer.dir/voxel_visualizer.cpp.i

CMakeFiles/voxel_visualizer.dir/voxel_visualizer.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/voxel_visualizer.dir/voxel_visualizer.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/kevin/PointCloudProcessing/voxel_visualizer.cpp -o CMakeFiles/voxel_visualizer.dir/voxel_visualizer.cpp.s

# Object files for target voxel_visualizer
voxel_visualizer_OBJECTS = \
"CMakeFiles/voxel_visualizer.dir/voxel_visualizer.cpp.o"

# External object files for target voxel_visualizer
voxel_visualizer_EXTERNAL_OBJECTS =

voxel_visualizer: CMakeFiles/voxel_visualizer.dir/voxel_visualizer.cpp.o
voxel_visualizer: CMakeFiles/voxel_visualizer.dir/build.make
voxel_visualizer: /usr/lib/x86_64-linux-gnu/libpcl_filters.so
voxel_visualizer: /usr/lib/x86_64-linux-gnu/libpcl_visualization.so
voxel_visualizer: /usr/lib/x86_64-linux-gnu/libboost_system.so
voxel_visualizer: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
voxel_visualizer: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
voxel_visualizer: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
voxel_visualizer: /usr/lib/x86_64-linux-gnu/libboost_regex.so
voxel_visualizer: /usr/lib/x86_64-linux-gnu/libvtkChartsCore-7.1.so.7.1p.1
voxel_visualizer: /usr/lib/x86_64-linux-gnu/libvtkInfovisCore-7.1.so.7.1p.1
voxel_visualizer: /usr/lib/x86_64-linux-gnu/libfreetype.so
voxel_visualizer: /usr/lib/x86_64-linux-gnu/libz.so
voxel_visualizer: /usr/lib/x86_64-linux-gnu/libjpeg.so
voxel_visualizer: /usr/lib/x86_64-linux-gnu/libpng.so
voxel_visualizer: /usr/lib/x86_64-linux-gnu/libtiff.so
voxel_visualizer: /usr/lib/x86_64-linux-gnu/libexpat.so
voxel_visualizer: /usr/lib/x86_64-linux-gnu/libvtkIOGeometry-7.1.so.7.1p.1
voxel_visualizer: /usr/lib/x86_64-linux-gnu/libvtkIOLegacy-7.1.so.7.1p.1
voxel_visualizer: /usr/lib/x86_64-linux-gnu/libvtkIOPLY-7.1.so.7.1p.1
voxel_visualizer: /usr/lib/x86_64-linux-gnu/libvtkRenderingLOD-7.1.so.7.1p.1
voxel_visualizer: /usr/lib/x86_64-linux-gnu/libvtkViewsContext2D-7.1.so.7.1p.1
voxel_visualizer: /usr/lib/x86_64-linux-gnu/libvtkViewsCore-7.1.so.7.1p.1
voxel_visualizer: /usr/lib/x86_64-linux-gnu/libvtkRenderingContextOpenGL2-7.1.so.7.1p.1
voxel_visualizer: /usr/lib/x86_64-linux-gnu/libvtkRenderingOpenGL2-7.1.so.7.1p.1
voxel_visualizer: /usr/lib/x86_64-linux-gnu/libflann_cpp.so
voxel_visualizer: /usr/lib/x86_64-linux-gnu/libpcl_sample_consensus.so
voxel_visualizer: /usr/lib/x86_64-linux-gnu/libpcl_io.so
voxel_visualizer: /usr/lib/x86_64-linux-gnu/libpcl_search.so
voxel_visualizer: /usr/lib/x86_64-linux-gnu/libpcl_octree.so
voxel_visualizer: /usr/lib/x86_64-linux-gnu/libpcl_kdtree.so
voxel_visualizer: /usr/lib/x86_64-linux-gnu/libpcl_common.so
voxel_visualizer: /usr/lib/x86_64-linux-gnu/libvtkInteractionWidgets-7.1.so.7.1p.1
voxel_visualizer: /usr/lib/x86_64-linux-gnu/libvtkFiltersModeling-7.1.so.7.1p.1
voxel_visualizer: /usr/lib/x86_64-linux-gnu/libvtkInteractionStyle-7.1.so.7.1p.1
voxel_visualizer: /usr/lib/x86_64-linux-gnu/libvtkFiltersExtraction-7.1.so.7.1p.1
voxel_visualizer: /usr/lib/x86_64-linux-gnu/libvtkFiltersStatistics-7.1.so.7.1p.1
voxel_visualizer: /usr/lib/x86_64-linux-gnu/libvtkImagingFourier-7.1.so.7.1p.1
voxel_visualizer: /usr/lib/x86_64-linux-gnu/libvtkalglib-7.1.so.7.1p.1
voxel_visualizer: /usr/lib/x86_64-linux-gnu/libvtkFiltersHybrid-7.1.so.7.1p.1
voxel_visualizer: /usr/lib/x86_64-linux-gnu/libvtkImagingGeneral-7.1.so.7.1p.1
voxel_visualizer: /usr/lib/x86_64-linux-gnu/libvtkImagingSources-7.1.so.7.1p.1
voxel_visualizer: /usr/lib/x86_64-linux-gnu/libvtkImagingHybrid-7.1.so.7.1p.1
voxel_visualizer: /usr/lib/x86_64-linux-gnu/libvtkRenderingAnnotation-7.1.so.7.1p.1
voxel_visualizer: /usr/lib/x86_64-linux-gnu/libvtkImagingColor-7.1.so.7.1p.1
voxel_visualizer: /usr/lib/x86_64-linux-gnu/libvtkRenderingVolume-7.1.so.7.1p.1
voxel_visualizer: /usr/lib/x86_64-linux-gnu/libvtkIOXML-7.1.so.7.1p.1
voxel_visualizer: /usr/lib/x86_64-linux-gnu/libvtkIOXMLParser-7.1.so.7.1p.1
voxel_visualizer: /usr/lib/x86_64-linux-gnu/libvtkIOCore-7.1.so.7.1p.1
voxel_visualizer: /usr/lib/x86_64-linux-gnu/libvtkRenderingContext2D-7.1.so.7.1p.1
voxel_visualizer: /usr/lib/x86_64-linux-gnu/libvtkRenderingFreeType-7.1.so.7.1p.1
voxel_visualizer: /usr/lib/x86_64-linux-gnu/libfreetype.so
voxel_visualizer: /usr/lib/x86_64-linux-gnu/libvtkImagingCore-7.1.so.7.1p.1
voxel_visualizer: /usr/lib/x86_64-linux-gnu/libvtkRenderingCore-7.1.so.7.1p.1
voxel_visualizer: /usr/lib/x86_64-linux-gnu/libvtkCommonColor-7.1.so.7.1p.1
voxel_visualizer: /usr/lib/x86_64-linux-gnu/libvtkFiltersGeometry-7.1.so.7.1p.1
voxel_visualizer: /usr/lib/x86_64-linux-gnu/libvtkFiltersSources-7.1.so.7.1p.1
voxel_visualizer: /usr/lib/x86_64-linux-gnu/libvtkFiltersGeneral-7.1.so.7.1p.1
voxel_visualizer: /usr/lib/x86_64-linux-gnu/libvtkCommonComputationalGeometry-7.1.so.7.1p.1
voxel_visualizer: /usr/lib/x86_64-linux-gnu/libvtkFiltersCore-7.1.so.7.1p.1
voxel_visualizer: /usr/lib/x86_64-linux-gnu/libvtkIOImage-7.1.so.7.1p.1
voxel_visualizer: /usr/lib/x86_64-linux-gnu/libvtkCommonExecutionModel-7.1.so.7.1p.1
voxel_visualizer: /usr/lib/x86_64-linux-gnu/libvtkCommonDataModel-7.1.so.7.1p.1
voxel_visualizer: /usr/lib/x86_64-linux-gnu/libvtkCommonTransforms-7.1.so.7.1p.1
voxel_visualizer: /usr/lib/x86_64-linux-gnu/libvtkCommonMisc-7.1.so.7.1p.1
voxel_visualizer: /usr/lib/x86_64-linux-gnu/libvtkCommonMath-7.1.so.7.1p.1
voxel_visualizer: /usr/lib/x86_64-linux-gnu/libvtkCommonSystem-7.1.so.7.1p.1
voxel_visualizer: /usr/lib/x86_64-linux-gnu/libvtkCommonCore-7.1.so.7.1p.1
voxel_visualizer: /usr/lib/x86_64-linux-gnu/libvtksys-7.1.so.7.1p.1
voxel_visualizer: /usr/lib/x86_64-linux-gnu/libvtkDICOMParser-7.1.so.7.1p.1
voxel_visualizer: /usr/lib/x86_64-linux-gnu/libvtkmetaio-7.1.so.7.1p.1
voxel_visualizer: /usr/lib/x86_64-linux-gnu/libz.so
voxel_visualizer: /usr/lib/x86_64-linux-gnu/libGLEW.so
voxel_visualizer: /usr/lib/x86_64-linux-gnu/libSM.so
voxel_visualizer: /usr/lib/x86_64-linux-gnu/libICE.so
voxel_visualizer: /usr/lib/x86_64-linux-gnu/libX11.so
voxel_visualizer: /usr/lib/x86_64-linux-gnu/libXext.so
voxel_visualizer: /usr/lib/x86_64-linux-gnu/libXt.so
voxel_visualizer: CMakeFiles/voxel_visualizer.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/kevin/PointCloudProcessing/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable voxel_visualizer"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/voxel_visualizer.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/voxel_visualizer.dir/build: voxel_visualizer

.PHONY : CMakeFiles/voxel_visualizer.dir/build

CMakeFiles/voxel_visualizer.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/voxel_visualizer.dir/cmake_clean.cmake
.PHONY : CMakeFiles/voxel_visualizer.dir/clean

CMakeFiles/voxel_visualizer.dir/depend:
	cd /home/kevin/PointCloudProcessing/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/kevin/PointCloudProcessing /home/kevin/PointCloudProcessing /home/kevin/PointCloudProcessing/build /home/kevin/PointCloudProcessing/build /home/kevin/PointCloudProcessing/build/CMakeFiles/voxel_visualizer.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/voxel_visualizer.dir/depend


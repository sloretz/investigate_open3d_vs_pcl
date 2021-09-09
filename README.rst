Open3D and PCL Comparison
=========================

Who's using Open3D in the ROS community
---------------------------------------

There is a `rosdep key python3-open3d-pip <https://github.com/ros/rosdistro/blob/d208d0b7fee8dcf2cad4d540fc055d9d9be3b6a8/rosdep/python.yaml#L2988>`_ added in `ros/rosdistro#28352 <https://github.com/ros/rosdistro/pull/28352>`_.
This kind of key cannot be used by released packages, so its presence suggests someone is using it in an unreleased ROS package.

There is a ``perception_open3d`` repository that is released into Melodic and Noetic (ROS 1) `ros-perception/perception_open3d <https://github.com/ros-perception/perception_open3d>`_ that was originally available at `ntnu-arl/open3d_ros <https://github.com/ntnu-arl/open3d_ros>`_.
There is a `ROS World 2020 Lightning talk about it <https://vimeo.com/480560723>`_.
The package depends on Open3d, but does not list a rosdep key in its ``package.xml``.
Instead it documents that Open3D must be installed manually.
The repo provides a conversions package and an examples package.
It has conversions from ``open3d::t::geometry::PointCloud`` and ``open3d::geometry::PointCloud`` to/from ``sensor_msgs::PointCloud2``.
There `is a foxy-devel branch <https://github.com/ros-perception/perception_open3d/tree/foxy-devel/open3d_conversions>`_ with conversions from ``open3d::geometry::PointCloud`` to/from ``sensor_msgs::PointCloud2``, but it has not been released.

An improvement here would be to create rosdep keys for ``libopen3d-dev``, ``python3-open3d`` and ``libopen3d0d``.
These packages won't be available for use in Rolling until the next LTS release of Ubuntu.
The version in Debian upstream is 0.9.0. Some time spent updating the version in Debian may be a good idea before Ubuntu branches off of it.

* https://packages.debian.org/bullseye/libopen3d-dev
* https://packages.debian.org/bullseye/libopen3d0d
* https://packages.debian.org/bullseye/python3-open3d
* https://packages.ubuntu.com/hirsute/libopen3d-dev
* https://packages.ubuntu.com/hirsute/python3-open3d
* https://packages.ubuntu.com/hirsute/libopen3d0d

There is a Python package for converting Open3D types to PointCloud messages (ROS Melodic only?) `SeungBack/open3d-ros-helper <https://github.com/SeungBack/open3d-ros-helper>`_.


Can I pass Python datastructures to C++ and back?
-------------------------------------------------

This question is answered in the ``cpp_py`` folder.

* Open3d - Yes, at least when using ``open3d::geometry::PointCloud`` and Pybind11
* PCL - No official Python bindings exist, but there are some unofficial ones
    * `strawlab/python-pcl <https://github.com/strawlab/python-pcl/issues/395>`_ is popular but unmaintained and uses cython
    * `davidcaron/pclpy <https://github.com/davidcaron/pclpy>`_ is another that uses Pybind11
        *  Also seems unmaintained, I could not try it on 20.04 because of this bug https://github.com/davidcaron/pclpy/issues/91


.. code-block:: console

  cd cpp_py
  mkdir build
  cd build
  cmake ..
  make
  cd ..
  PYTHONPATH=$(pwd)/build python3 test_03d.py


Is Open3D more or less actively developed than PCL?
---------------------------------------------------

In short PCL and Open3D seem to be developed at a similar level.

Checking out the libraries and looking at the number of commits using

.. code-block:: console

    git log --since '2020/08/30' --dense --oneline | wc -l

with different dates at the default branch yields:

.. list-table:: Number of commits

    * - Library
      - 3 year commits
      - 1 year commits
      - 6 month commits
      - 3 month commits
    * - PCL
      - 2906
      - 569
      - 304
      - 110
    * - Open3D
      - 1717
      - 690
      - 344
      - 153

They seem similar in the number of commits, but Open3D has more.

Looking at the number of commit authors with

.. code-block:: console

    git log --since '2021/05/31' --dense --pretty=%an | sort | uniq | wc -l

at different dates at the default branch yeilds:

.. list-table:: Number of contributors

    * - Library
      - 3 year contributors
      - 1 year contributors
      - 6 month contributors
      - 3 month contributors
    * - PCL
      - 160
      - 58
      - 41
      - 24
    * - Open3D
      - 106
      - 50
      - 27
      - 19

They seem similar in the number of contributors too, but PCL has more.


What's the support for GPUs look like in Open3D and PCL?
--------------------------------------------------------

Open3d starts to have CUDA support in 0.10.0.
The developers are `incrementally rolling it out <https://github.com/isl-org/Open3D/issues/942#issuecomment-670255854>`_.
It looks like types in the ``open3d::t`` namespace (``t`` stands for tensor) support being run on an NVidia GPU.

PCL appears to have CUDA implementations of some features available in the ``pcl::gpu::`` C++ namespace.
NVIDIA maintains a library `NVIDIA-AI-IOT/cuda-pcl <https://github.com/NVIDIA-AI-IOT/cuda-pcl>`_ for some operations on PCL types.


What's the difference in execution time?
----------------------------------------

The ``benchmark`` folder attempts to answer this.
Open3D 0.9.0 and PCL 1.11.1 from Debian packages on Debian Bullseye were used.

.. code-block:: console

    cd benchmark
    mkdir build
    cd build
    cmake ..
    make
    ln -s ../bunny.pcd
    ./iterative_closest_point
    ./voxel_downsample


Comparing Voxel downsampling seems reasonable as the two libraries should have the same output.
Open3D seems significantly faster.


.. code-block:: console

    $ ./voxel_downsample
    2021-09-09T12:03:14-07:00
    Running ./voxel_downsample
    Run on (24 X 4950.19 MHz CPU s)
    CPU Caches:
      L1 Data 32 KiB (x12)
      L1 Instruction 32 KiB (x12)
      L2 Unified 512 KiB (x12)
      L3 Unified 32768 KiB (x2)
    Load Average: 0.27, 0.38, 0.36
    ***WARNING*** CPU scaling is enabled, the benchmark real time measurements may be noisy and will incur extra overhead.
    -----------------------------------------------------
    Benchmark           Time             CPU   Iterations
    -----------------------------------------------------
    BM_open3d       12449 ns        12431 ns        56480
    BM_pcl          49400 ns        49332 ns        14117


Comparing ICP may not be reasonable as the two libraries have different convergence criteria.
PCL appears to have rotation and translation thresholds, while Open3D has relative fitness and RMSE thresholds.
The data below may not be useful.


.. code-block:: console

    $ ./iterative_closest_point
    2021-09-09T12:08:04-07:00
    Running ./iterative_closest_point
    Run on (24 X 4950.19 MHz CPU s)
    CPU Caches:
      L1 Data 32 KiB (x12)
      L1 Instruction 32 KiB (x12)
      L2 Unified 512 KiB (x12)
      L3 Unified 32768 KiB (x2)
    Load Average: 0.22, 0.32, 0.34
    ***WARNING*** CPU scaling is enabled, the benchmark real time measurements may be noisy and will incur extra overhead.
    -----------------------------------------------------
    Benchmark           Time             CPU   Iterations
    -----------------------------------------------------
    BM_open3d     2002711 ns      1971130 ns          350
    BM_pcl       15637302 ns     15633577 ns           45
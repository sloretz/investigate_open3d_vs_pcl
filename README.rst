Open3D and PCL Comparison
=========================

Can I pass Python datastructures to C++ and back?
-------------------------------------------------

This question is answered in the ``cpp_py`` folder.

* Open3d - Yes, at least when using ``open3d::geometry::PointCloud`` and Pybind11
* PCL - No official Python bindings exist, but there some unofficial ones
  * `strawlab/python-pcl <https://github.com/strawlab/python-pcl/issues/395>`_ is popular but unmaintained and uses cython
  * `davidcaron/pclpy https://github.com/davidcaron/pclpy>`_ is another that uses Pybind11

Open3D

.. code-block:: console

  cd cpp_py
  mkdir build
  cd build
  cmake ..
  make
  cd ..
  PYTHONPATH=$(pwd)/build python3 test_03d.py


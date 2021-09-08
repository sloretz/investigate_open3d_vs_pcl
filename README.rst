Open3D and PCL Comparison
=========================

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


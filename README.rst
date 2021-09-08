Open3D and PCL Comparison
=========================

Can I pass Python datastructures to C++ and back?
-------------------------------------------------

This question is answered in the ``cpp_py`` folder.

* Open3d - Yes, at least when using open3d::geometry::PointCloud and Pybind11
* PCL - There's no official Python bindings exist, `only unmaintained ones <https://github.com/strawlab/python-pcl/issues/395>`_


.. code-block:: console

  PYTHONPATH=$(pwd)/build python3 test_03d.py



- Sophus::SE3f represents a 3D transformation (including rotation and translation).
- Tcw stands for camera to world

The following lines compute pose inversion(world to camera), translation and orientation:

Sophus::SE3f Twc = Tcw.inverse();
Eigen::Vector3f twc = Twc.translation();
Eigen::Quaternionf q = Twc.unit_quaternion();

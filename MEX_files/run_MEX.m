mex -R2018a bxbybz_to_euler_mex.cpp
mex -R2018a apply_transformation_mex.cpp
mex -R2018a ascent_IK_mex.cpp iiwa_utilities.cpp opt_obj.cpp /usr/local/lib/libnlopt.so
mex -R2018a get_iiwa7_FK_all_joints_mex.cpp;
mex -R2018a get_iiwa14_FK_all_joints_mex.cpp;
mex -R2018a get_iiwa7_FK_mex.cpp;



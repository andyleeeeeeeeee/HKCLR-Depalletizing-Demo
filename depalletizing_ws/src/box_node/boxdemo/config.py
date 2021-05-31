import argparse

parser = argparse.ArgumentParser()

# # WUXI LAB
# # segment relevant
# parser.add_argument('--type', type=int, default= 1)
# parser.add_argument('--model', type=str, default='model.yml')
# parser.add_argument('--downsample_resolution', type=float, default=0.002, help='the resolution for pcl voxel grid filter')
# parser.add_argument('--filter_z_min', type=float, default=2.8) # 1354 for 11.30 data
# parser.add_argument('--filter_z_max', type=float, default=1.7) # 1000 for 11.30 data
# parser.add_argument('--filter_y_min', type=float, default=2)  # roi in y-axis
# parser.add_argument('--filter_y_max', type=float, default=-2) # roi in y-axis
#
# # clustering relevant
# parser.add_argument('--Radius_clustering', type=int, default=0.00475)
# parser.add_argument('--KNN_number', type=int, default=10)  # 30
# parser.add_argument('--seg_min_number', type=int, default=3000)
# parser.add_argument('--seg_max_number', type=int, default=200000)
# parser.add_argument('--seg_smoothness', type=float, default=0.13)
# parser.add_argument('--seg_curvature', type=int, default=0.025)
#
# # filtering edge points relevant
# parser.add_argument('--Radius_KD', type=int, default=0.005)
#
# # filtering background relevant
# parser.add_argument('--filter_slope_plane_para', type=list,
#                     default=[-0.021592, 0.761064, 0.648317, -0.285321, 0.463227, 2.301724])
# parser.add_argument('--filter_ground_plane_para', type=list,
#                     default=[-0.031156, 0.150190, 0.988166, 0.133395, 0.095292, 2.456849])  # ground plane paras
# parser.add_argument('--dist2plane', type=float, default=0.02)
#
# # gripper relevant
# parser.add_argument('--grasper_size_para', type=list,
#                     default=[0.28, 0.2, 0.16, 0.115])  # grasper_box_x/y/z/Sucker length
# # parser.add_argument('--big_gripper_size_para', type=int,
# #                     default=0.20)  # big gripper x==y==0.4
# parser.add_argument('--big_gripper_size_para', type=list,
#                     default=[0.64, 0.13, 0.07])  # long gripper
#
# # grasp plane selection relevant
# parser.add_argument('--origin_para', type=list,
#                     default=[-1, -1, 0])  # big gripper x==y==0.4
# parser.add_argument('--sort_height', type=float, default=0.08)
#
#
# # edge detection relevant
# parser.add_argument('--camera_intrinsics', type=list,
#                     default=[2.3395201744955079e+03, 2.3395201744955079e+03, 799, 980])  # camera value3: smaller->left
# parser.add_argument('--PREPROCESS_THRESHOLD_DEFAULT', type=int, default=30)  # threshold for edge detection
#
# # task relevant
# parser.add_argument('--UNSTACK', type=int, default=1)  # toggle for UNSTACK or BOX-PICKING


# SHENZHEN LAB
# segment relevant
parser.add_argument('--type', type=int, default= 1)
parser.add_argument('--model', type=str, default='/home/andylee/HKCLR-Depalletizing-Demo/depalletizing_ws/src/box_node/boxdemo/model.yml')
parser.add_argument('--downsample_resolution', type=float, default=0.002, help='the resolution for pcl voxel grid filter')
parser.add_argument('--filter_z_min', type=float, default=2.8) # 1354 for 11.30 data
parser.add_argument('--filter_z_max', type=float, default=1.7) # 1000 for 11.30 data
parser.add_argument('--filter_y_min', type=float, default=2)  # roi in y-axis
parser.add_argument('--filter_y_max', type=float, default=-2) # roi in y-axis
parser.add_argument('--filter_x_min', type=float, default=0.5)  # roi in y-axis
parser.add_argument('--filter_x_max', type=float, default=-0.78) # roi in y-axis

# clustering relevant
parser.add_argument('--Radius_clustering', type=int, default=0.008)
parser.add_argument('--KNN_number', type=int, default=60)  # 30
parser.add_argument('--seg_min_number', type=int, default=500)
parser.add_argument('--seg_max_number', type=int, default=2000000)
parser.add_argument('--seg_smoothness', type=float, default=0.13)
parser.add_argument('--seg_curvature', type=int, default=0.025)

# filtering edge points relevant
parser.add_argument('--Radius_KD', type=int, default=0.005)

# filtering background relevant
parser.add_argument('--filter_slope_plane_para', type=list,
                    default=[-0.019901, 0.747266, 0.664227, -0.070619, 0.524372, 2.256102])
parser.add_argument('--filter_ground_plane_para', type=list,
                    default=[-0.031060, 0.150862, 0.988067, -0.138820, -0.592458, 2.566179])  # ground plane paras
parser.add_argument('--dist2plane', type=float, default=0.02)

# gripper relevant
parser.add_argument('--grasper_size_para', type=list,
                    default=[0.28, 0.2, 0.16, 0.115])  # grasper_box_x/y/z/Sucker length
# parser.add_argument('--big_gripper_size_para', type=int,
#                     default=0.20)  # big gripper x==y==0.4
parser.add_argument('--big_gripper_size_para', type=list,
                    default=[0.64, 0.13, 0.07])  # long gripper

# grasp plane selection relevant
parser.add_argument('--origin_para', type=list,
                    default=[-1, -1, 0])  # big gripper x==y==0.4
parser.add_argument('--sort_height', type=float, default=0.08)


# edge detection relevant
parser.add_argument('--camera_intrinsics', type=list,
                    default=[2.3395201744955079e+03, 2.3395201744955079e+03, 799, 980])  # camera value3: smaller->left
parser.add_argument('--PREPROCESS_THRESHOLD_DEFAULT', type=int, default=135)  # threshold for edge detection

# task relevant
parser.add_argument('--UNSTACK', type=int, default=1)  # toggle for UNSTACK or BOX-PICKING
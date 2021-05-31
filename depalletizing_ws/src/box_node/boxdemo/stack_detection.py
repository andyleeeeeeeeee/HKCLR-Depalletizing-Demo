import pcl
import cv2
from segment_3D import plane_extract, is_perp_plane
from grasp_selection import grasp_selection, grasp_selection_zxy
from pose import pose_estimation
from config import parser
from collision import plane_gripper_collision_test2
from edge_detection import egde_detect
from conversion import get_edge_point_cloud, get_point_cloud_bb, get_plane_corners
from visualize import visualize_scene, visualize_pose_in_raw_pts_one_box
import numpy as np
import struct
import time
import pcl2_img
# import open3d as o3d

params = parser.parse_args()
#  get camera intrinsic parameters
camera_paras = params.camera_intrinsics
intrinsics = np.identity(3, np.float32)
intrinsics[0, 0] = camera_paras[0]
intrinsics[1, 1] = camera_paras[1]
intrinsics[0, 2] = camera_paras[2]
intrinsics[1, 2] = camera_paras[3]

def checkDirection(R):
    box_z_axis = R[:, 2]
    box_z_axis[0] = 0
    camera_z_axis = [0, 0, 1]
    dirt = np.cross(box_z_axis, camera_z_axis)
    if(dirt[0] >= 0):
        return True
    else:
        return False

def detect(np_cloud, gray_img):
    success, result = detect_with_view(np_cloud, gray_img)
    if(success):
        _, _, pose, rec_wid, rec_len = result[0]
        print(pose[0], pose[1], pose[2], pose[3], pose[4], pose[5], pose[6], rec_wid, rec_len)
        return True, pose[0], pose[1], pose[2], pose[3], pose[4], pose[5], pose[6], rec_wid, rec_len

    print("No plane detected")
    return False, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0

def detect_with_view_DL(pts, gray_img):
    # 1. plane detection from image
    pc_array = pts.reshape((gray_img.shape[0], gray_img.shape[1], 4))
    masks = segment_2D(gray_img)

    # 2. crop all 3D plans from point cloud
    masked_pc_list = crop_point_cloud(pc_array, masks)

    # 3. pre-processing for these cropped planes
    masked_pc_filter_list = []
    i = 0
    for masked_pc in masked_pc_list:
        masked_pc = masked_pc[:, :3]
        # 1. transform input_pts to pcl.PointCloud
        cloud_in = pcl.PointCloud()
        cloud_in.from_array(masked_pc)
        # print('input successful...')

        # 2. filter outlier
        sor_Filter = cloud_in.make_statistical_outlier_filter()
        sor_Filter.set_mean_k(90)
        sor_Filter.set_std_dev_mul_thresh(1.0)
        cloud_sor = sor_Filter.filter()
        masked_pc_filter = cloud_sor.to_array()
        masked_pc_filter = masked_pc_filter[np.where(np.all(masked_pc_filter!=[0.0, 0.0, 0.0], axis=-1)==True)[0]]
        masked_pc_filter_list.append(masked_pc_filter)
        # np.savetxt(str(i)+"filter.txt", masked_pc_filter)
        i = i+1

    # 3. planes grasping selection
    grasp_plane_ids = grasp_selection_zxy(masked_pc_filter_list, params)
    print(grasp_plane_ids)

    # 4. pose estimation for all planes
    if (len(grasp_plane_ids) != 0):
        for i in range(len(grasp_plane_ids)):
            # pose estimation
            idx = grasp_plane_ids[i]
            cluster_planes_i = masked_pc_filter_list[idx] # Pose estimation
            R, t, pose, rec_wid, rec_len = pose_estimation(cluster_planes_i)
            return True, [[R, t, pose, rec_wid, rec_len], cluster_planes_i, masked_pc_filter_list, 0]


def detect_with_view_multi(pts):
    pts = pts[:, 0:3]
    print(pts.shape)

    # 1. segment point cloud
    grasp_box = pts
    cluster_planes, cloud_ds = plane_extract(pts, params)
    visualize_scene(cluster_planes)

    if(len(cluster_planes) == 0):
        print("No plane detected!")
        return False, []
    # visualize_scene(cluster_planes)

    # 2. estimate pose for each plane
    pose_list = []
    for idx in range(len(cluster_planes)):
        cluster_id = idx
        # R, t, pose, rec_wid, rec_len = pose_estimation(cluster_planes[cluster_id])
        pose_list.append(pose_estimation(cluster_planes[cluster_id]))
       
    return True, pose_list


def detect_with_view_single(pts, gray_img):
    pts = pts[:, 0:3]
    print(pts.shape)

    # 3. segment point cloud
    grasp_box = pts
    cluster_planes, cloud_ds = plane_extract(pts, params)
    # print('time cost for segmentation: ', time.time() - time1)
    grasp_plane_ids = grasp_selection(cluster_planes, params)
    visualize_scene(cluster_planes)

    if (len(cluster_planes) != 0):
        for i in range(len(grasp_plane_ids)):
            # 3.1 get the corresponding sub-image for this extracted plane
            idx = grasp_plane_ids[i]
            cluster_planes_i = cluster_planes[idx][:, 0:3]
            if(is_perp_plane(cluster_planes_i, params.filter_ground_plane_para)):
                continue
                # print("perpendicular")
            else:
                # rmin, rmax, cmin, cmax = get_point_cloud_bb(cluster_planes_i, intrinsics)
                rmin, rmax, cmin, cmax = get_point_cloud_bb(cluster_planes_i, pts, gray_img.shape)
                gray_masked = gray_img[rmin:rmax, cmin:cmax]

                # 3.2 get the edges in this sub-image
                # time1 = time.time()
                edges_masked = egde_detect(gray_masked, params.model)  # 2. detect edges from image
                edges = np.zeros_like(gray_img)
                edges[rmin:rmax, cmin: cmax] = edges_masked
                # print('time cost for detecting edges in the image: ', time.time() - time1)

                # 3.3 get the edge points
                edge_pts = get_edge_point_cloud(pts, edges)
                # print("edge_pts: ", edge_pts.shape)
                # np.savetxt("edge_pts.txt", edge_pts)

                # 3.4 filtering the neighboring points near the edge points
                # time1 = time.time()
                pc = pcl.PointCloud(cluster_planes_i)
                kd = pcl.KdTreeFLANN(pc)
                edge_pts = np.float32(edge_pts)
                pc_edges = pcl.PointCloud(edge_pts)
                # indices, sqr_distances = kd.nearest_k_search_for_cloud(pc_edges, 25)
                indices, sqr_distances = kd.radius_search_for_cloud(pc_edges, params.Radius_KD, 25)
                # print("indice: ", indices.shape)
                # indices, sqr_distances = kd.radius_search_for_cloud(pc_edges, 0.009)
                indices = indices.flatten()
                cluster_planes_i_center = np.mean(cluster_planes_i, axis=0)
                cluster_planes_i[indices, :] = cluster_planes_i_center
                # print('time cost for filtering the neighboring points near the edge points: ', time.time() - time1)

                # 3.5 segment the cluster_planes_i
                sub_cluster_planes, _ = plane_extract(cluster_planes_i, params)

                # 3.6 grasp pose estimation
                if(len(sub_cluster_planes) > 1):
                    grasp_box = sub_cluster_planes[0]
                    # print("aaa")
                    #visualize_scene_with_pose(sub_cluster_planes, 0, R, t)
                    return True, [pose_estimation(sub_cluster_planes[0]), grasp_box, sub_cluster_planes, 0]
                else:
                    grasp_box = cluster_planes_i
                    # np.savetxt("cluster_planes_i.txt", cluster_planes_i)
                    #visualize_scene_with_pose(cluster_planes, idx, R, t)
                    # print("bbb")
                    return True, [pose_estimation(cluster_planes_i), grasp_box, cluster_planes, idx]

    else:
        print("No any plane structures!")
        return False, []





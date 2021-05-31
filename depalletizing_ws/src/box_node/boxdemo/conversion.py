import numpy as np
import numpy.ma as ma
import cv2
import time
from config import parser
params = parser.parse_args()

def depth2pc(depth, intrinsics):
    scale = np.array([1.0, 1.0, 1.0])
    fx, fy = intrinsics[0, 0], intrinsics[1, 1]
    cx, cy = intrinsics[0, 2], intrinsics[1, 2]

    non_zero_mask = (depth > 0)
    idxs = np.where(non_zero_mask)

    z = depth[idxs[0], idxs[1]]
    z = z * scale[2]
    x = (idxs[1] - cx) * z / fx
    y = (idxs[0] - cy) * z / fy
    pts = np.stack((x, y, z), axis=1)
    return pts


# def pc2depth(intrinsics, pc, img_shape):
#     h, w = img_shape
#     depth_map = np.zeros((h, w))
#     pc_non_zero = pc[np.where(np.all(pc[:, :] != [0.0, 0.0, 0.0], axis=-1) == True)[0]]
#     fx, fy = intrinsics[0, 0], intrinsics[1, 1]
#     cx, cy = intrinsics[0, 2], intrinsics[1, 2]
#     depth = pc_non_zero[:, 2]  # point_z
#     coords_x = pc_non_zero[:, 0] / pc_non_zero[:, 2] * fx + cx
#     coords_y = pc_non_zero[:, 1] / pc_non_zero[:, 2] * fy + cy
#     coords = np.stack([coords_x, coords_y], axis=-1)
#     depth_map[coords_y, coords_x] = depth
#     return coords.astype('int32'), depth


def pc2depth_from_file(pc, img_shape):
    depth_map = pc[:, 2].reshape(img_shape)
    return depth_map


# def get_edge_point_cloud(pts, intrinsics, edges):
#     # start = time.time()
#     edge_mask = ma.getmaskarray(ma.masked_greater_equal(edges, 0.25))
#     depth_map = pc2depth_from_file(pts, edges.shape)
#     masked_depth = depth_map * edge_mask
#     masked_point_cloud = depth2pc(masked_depth, intrinsics)
#     # print("edge point get function :", time.time() - start)
#     return masked_point_cloud



def boundary_collision_detect(selected_plane, pts):
    # 1. get the for plane corners
    corners = get_plane_corners(selected_plane)

    # 2. collision detection




def get_plane_corners(selected_pc):
    # 1. pre-process
    selected_pc = selected_pc[:, :3]
    pts_center = np.mean(selected_pc, axis=0)
    normalized_pts = selected_pc - pts_center  # normalization

    # 2. PCA analysis
    H = np.dot(normalized_pts.T, normalized_pts)
    eigenvectors, eigenvalues, eigenvectors_t = np.linalg.svd(H)  # H = U S V

    # 3. local projection
    projected_pts = np.dot(normalized_pts, eigenvectors)
    projected_pts = projected_pts.T
    # np.savetxt("ds.txt", projected_pts.T)
    projected_pts = projected_pts[:2, :]

    # 4. fit the obb of the projected point clouds
    rec = cv2.minAreaRect(projected_pts.T.astype(np.float32))
    box  = cv2.boxPoints(rec)
    box_3d = np.zeros((4, 3))
    box_3d[:, :2] = box
    box_pc = np.dot(box_3d, np.linalg.inv(eigenvectors)) + pts_center[:3]
    # print('box_pc:', box_pc)
    # np.savetxt("box.txt", box_pc)
    # np.savetxt("box_pc.txt", box_pc)

    # 5. get the corners
    return box_pc

def get_point_cloud_bb(selected_pc, pc_array, img_size):
    # 1. pre-process
    rec = cv2.minAreaRect(selected_pc[:, :2].astype(np.float32))
    points_array = cv2.boxPoints(rec)

    points_list = list(points_array)
    points_location_list = []
    xmap = np.tile(np.arange(img_size[0]).reshape(-1, 1), (1, img_size[1]))
    ymap = np.tile(np.arange(img_size[1]).reshape(1, -1), (img_size[0], 1))
    pc_array_aug = np.concatenate((xmap[:, :, np.newaxis], ymap[:, :, np.newaxis],
                                   pc_array.reshape((img_size[0], img_size[1], 3))), axis=2).reshape(-1, 5)
    for point in points_list:
        point_idx = np.argmin(np.sum(np.abs(pc_array[:, :2] - point[:2]), axis=1))
        points_location_list.append(pc_array_aug[point_idx, :2])

    bb = np.array(points_location_list)
    rmin, cmin = np.min(bb, axis=0).astype(np.uint16)
    rmax, cmax = np.max(bb, axis=0).astype(np.uint16)
    cmin = cmin-8
    rmin = rmin-8
    if(cmin < 0):
        cmin = 0
    if(rmin < 0):
        rmin = 0
    cmax = cmax+8
    rmax = rmax+8
    return rmin, rmax, cmin, cmax

def get_subimage_corners2(selected_pc, pc_array, img_size, params):
    # 1. get corners
    corners = get_plane_corners(selected_pc)

    # 2. get

    points_list = list(points_array)
    points_location_list = []
    ymap = np.tile(np.arange(img_size[0]).reshape(-1, 1), (1, img_size[1]))
    xmap = np.tile(np.arange(img_size[1]).reshape(1, -1), (img_size[0], 1))
    pc_array_aug = np.concatenate((xmap[:, :, np.newaxis], ymap[:, :, np.newaxis],
                                   pc_array.reshape((img_size[0], img_size[1], 3))), axis=2).reshape(-1, 5)
    for point in points_list:
        point_idx = np.argmin(np.sum(np.abs(pc_array[:, :2] - point[:2]), axis=1))
        points_location_list.append(pc_array_aug[point_idx, :2])
    bb = np.array(points_location_list, dtype = np.int32)

    return bb

def get_subimage_corners(selected_pc, pc_array, img_size, params):
    # 1. pre-process
    # selected_pc = selected_pc[:, :3]
    # R = np.identity(3)
    # z_axis = params.filter_ground_plane_para
    # R[:, 2] = z_axis[0:3]
    # selected_pc2 = np.dot(selected_pc, R)
    # selected_pc2 = selected_pc2[:, :2]
    #
    # rec = cv2.minAreaRect(selected_pc2.astype(np.float32))
    # points_array = cv2.boxPoints(rec)

    points_array = get_plane_corners(selected_pc)
    # points_array
    # np.savetxt("points_array.txt", points_array)


    points_list = list(points_array)
    points_location_list = []
    ymap = np.tile(np.arange(img_size[0]).reshape(-1, 1), (1, img_size[1]))
    xmap = np.tile(np.arange(img_size[1]).reshape(1, -1), (img_size[0], 1))
    pc_array_aug = np.concatenate((xmap[:, :, np.newaxis], ymap[:, :, np.newaxis],
                                   pc_array.reshape((img_size[0], img_size[1], 3))), axis=2).reshape(-1, 5)
    pc_array_aug_non_zero = pc_array_aug[np.where(np.all(pc_array_aug[:, 2:] != [0.0, 0.0, 0.0], axis=-1) == True)[0]]

    for point in points_list:
        point_idx = np.argmin(np.sum(np.abs(pc_array_aug_non_zero[:, 2:5] - point), axis=1))
        points_location_list.append(pc_array_aug_non_zero[point_idx, :2])
    #
    # ymap = np.tile(np.arange(img_size[0]).reshape(-1, 1), (1, img_size[1]))
    # xmap = np.tile(np.arange(img_size[1]).reshape(1, -1), (img_size[0], 1))
    # pc_array_aug = np.concatenate((xmap[:, :, np.newaxis], ymap[:, :, np.newaxis],
    #                                pc_array.reshape((img_size[0], img_size[1], 3))), axis=2).reshape(-1, 5)
    # for point in points_list:
    #     point_idx = np.argmin(np.sum(np.abs(pc_array[:, :2] - point[:2]), axis=1))
    #     points_location_list.append(pc_array_aug[point_idx, :2])
    bb = np.array(points_location_list, dtype = np.int32)

    return bb


def get_edge_point_cloud(pts, edges):
    # start = time.time()
    edge_mask = ma.getmaskarray(ma.masked_greater_equal(edges, 0.25))
    choose = list(edge_mask.flatten().nonzero()[0])
    masked_point_cloud = pts[choose]
    # print("edge point get function :", time.time() - start)
    return masked_point_cloud
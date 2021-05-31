import numpy as np
from config import parser

def generate_grasper_box (R, t, params):
    # 1. generate the grasper_box
    grasper_box_para = params.grasper_size_para
    grasper_box_x = grasper_box_para[0]
    grasper_box_y = grasper_box_para[1]
    grasper_box_z = grasper_box_para[2] + grasper_box_para[3]
    grapser_box_sucker = grasper_box_para[3]

    axis_x = R[:, 0]
    axis_y = R[:, 1]
    axis_z = R[:, 2]
    grasper_box_corner1 = t - grasper_box_x / 2 * axis_x - grasper_box_y / 2 * axis_y - grasper_box_z * axis_z
    grasper_box_corner2 = t + grasper_box_x / 2 * axis_x - grasper_box_y / 2 * axis_y - grasper_box_z * axis_z
    grasper_box_corner3 = t - grasper_box_x / 2 * axis_x + grasper_box_y / 2 * axis_y - grasper_box_z * axis_z
    grasper_box_corner4 = t + grasper_box_x / 2 * axis_x + grasper_box_y / 2 * axis_y - grasper_box_z * axis_z
    grasper_box_corner5 = t - grasper_box_x / 2 * axis_x - grasper_box_y / 2 * axis_y - grapser_box_sucker * axis_z
    grasper_box_corner6 = t + grasper_box_x / 2 * axis_x - grasper_box_y / 2 * axis_y - grapser_box_sucker * axis_z
    grasper_box_corner7 = t - grasper_box_x / 2 * axis_x + grasper_box_y / 2 * axis_y - grapser_box_sucker * axis_z
    grasper_box_corner8 = t + grasper_box_x / 2 * axis_x + grasper_box_y / 2 * axis_y - grapser_box_sucker * axis_z
    grasper_box_center = (grasper_box_corner1 + grasper_box_corner8) / 2
    grasper_box = [grasper_box_corner1, grasper_box_corner2, grasper_box_corner3, grasper_box_corner4,
                   grasper_box_corner5, grasper_box_corner6, grasper_box_corner7, grasper_box_corner8,
                   grasper_box_center]
    grasper_box = np.array(grasper_box)

    box_lines = [
        [0, 1],
        [0, 2],
        [1, 3],
        [2, 3],
        [4, 5],
        [4, 6],
        [5, 7],
        [6, 7],
        [0, 4],
        [1, 5],
        [2, 6],
        [3, 7],
        [1, 8],
        [2, 8],
    ]
    return grasper_box, box_lines

def generate_box_box (R, t, w, h, params):
    # 1. generate the box_range
    box_para = params.grasper_size_para
    box_range_x = w
    box_range_y = h
    box_offset = 0.008
    box_range_z = box_para[3] + box_offset

    axis_x = R[:, 0]
    axis_y = R[:, 1]
    axis_z = R[:, 2]
    box_corner1 = t - box_range_x / 2 * axis_x - box_range_y / 2 * axis_y - box_range_z * axis_z
    box_corner2 = t + box_range_x / 2 * axis_x - box_range_y / 2 * axis_y - box_range_z * axis_z
    box_corner3 = t - box_range_x / 2 * axis_x + box_range_y / 2 * axis_y - box_range_z * axis_z
    box_corner4 = t + box_range_x / 2 * axis_x + box_range_y / 2 * axis_y - box_range_z * axis_z
    box_corner5 = t - box_range_x / 2 * axis_x - box_range_y / 2 * axis_y - box_offset * axis_z
    box_corner6 = t + box_range_x / 2 * axis_x - box_range_y / 2 * axis_y - box_offset * axis_z
    box_corner7 = t - box_range_x / 2 * axis_x + box_range_y / 2 * axis_y - box_offset * axis_z
    box_corner8 = t + box_range_x / 2 * axis_x + box_range_y / 2 * axis_y - box_offset * axis_z
    box_center = (box_corner1 + box_corner8) / 2
    box = [box_corner1, box_corner2, box_corner3, box_corner4, box_corner5, box_corner6, box_corner7, box_corner8,
           box_center]
    box_range = np.array(box)
    # np.savetxt("ds.txt", box_range)
    box_range_lines = [
        [0, 1],
        [0, 2],
        [1, 3],
        [2, 3],
        [4, 5],
        [4, 6],
        [5, 7],
        [6, 7],
        [0, 4],
        [1, 5],
        [2, 6],
        [3, 7],
        [1, 8],
        [2, 8],
    ]
    return box_range, box_range_lines

def generate_gripper_plane (R, t, params):
    # 1. generate the grasper_box
    grasper_box_para = params.big_gripper_size_para
    grasper_box_x = big_gripper_size_para
    grasper_box_y = big_gripper_size_para
    grapser_box_sucker = grasper_box_para[3]

    axis_x = R[:, 0]
    axis_y = R[:, 1]
    axis_z = R[:, 2]
    grasper_box_corner1 = t - grasper_box_x / 2 * axis_x - grasper_box_y / 2 * axis_y - grasper_box_z * axis_z
    grasper_box_corner2 = t + grasper_box_x / 2 * axis_x - grasper_box_y / 2 * axis_y - grasper_box_z * axis_z
    grasper_box_corner3 = t - grasper_box_x / 2 * axis_x + grasper_box_y / 2 * axis_y - grasper_box_z * axis_z
    grasper_box_corner4 = t + grasper_box_x / 2 * axis_x + grasper_box_y / 2 * axis_y - grasper_box_z * axis_z
    grasper_box_corner5 = t - grasper_box_x / 2 * axis_x - grasper_box_y / 2 * axis_y - grapser_box_sucker * axis_z
    grasper_box_corner6 = t + grasper_box_x / 2 * axis_x - grasper_box_y / 2 * axis_y - grapser_box_sucker * axis_z
    grasper_box_corner7 = t - grasper_box_x / 2 * axis_x + grasper_box_y / 2 * axis_y - grapser_box_sucker * axis_z
    grasper_box_corner8 = t + grasper_box_x / 2 * axis_x + grasper_box_y / 2 * axis_y - grapser_box_sucker * axis_z
    grasper_box_center = (grasper_box_corner1 + grasper_box_corner8) / 2
    grasper_box = [grasper_box_corner1, grasper_box_corner2, grasper_box_corner3, grasper_box_corner4,
                   grasper_box_corner5, grasper_box_corner6, grasper_box_corner7, grasper_box_corner8,
                   grasper_box_center]
    grasper_box = np.array(grasper_box)

    box_lines = [
        [0, 1],
        [0, 2],
        [1, 3],
        [2, 3],
        [4, 5],
        [4, 6],
        [5, 7],
        [6, 7],
        [0, 4],
        [1, 5],
        [2, 6],
        [3, 7],
        [1, 8],
        [2, 8],
    ]
    return grasper_box, box_lines

# box collision test
def plane_gripper_collision_test2 (cluster_planes2, R, t, w, h, params):
    # 1.generate the box_range
    big_gripper_size_para = params.big_gripper_size_para
    axis_x = R[:, 0]
    axis_y = R[:, 1]
    axis_z = R[:, 2]

    # 2. generate 4 possible gripper centers
    center_0 = t + (big_gripper_size_para[0]/2 - w / 2) * axis_x + (big_gripper_size_para[1]/2 - h / 2) * axis_y
    center_1 = t - (big_gripper_size_para[0]/2 - w / 2) * axis_x - (big_gripper_size_para[1]/2 - h / 2) * axis_y
    center_2 = t - (big_gripper_size_para[0]/2 - w / 2) * axis_x + (big_gripper_size_para[1]/2 - h / 2) * axis_y
    center_3 = t + (big_gripper_size_para[0]/2 - w / 2) * axis_x - (big_gripper_size_para[1]/2 - h / 2) * axis_y
    gripper_centers = [center_0, center_1, center_2, center_3]
    # 3.
    final_gripper_center = center_0
    for i in range(4):
        box_center = gripper_centers[i]
        points_inbox = 0
        box_is_collision = 0
        for j in range(cluster_planes2.shape[0]):
            p_j = cluster_planes2[j, :]
            proj_x = abs((p_j - box_center).dot(axis_x))
            proj_y = abs((p_j - box_center).dot(axis_y))
            # proj_z = abs((p_j - box_center).dot(axis_z))
            if (proj_x < big_gripper_size_para[0]/2 ):
                if (proj_y < big_gripper_size_para[1]/2):
                    points_inbox = points_inbox + 1
                # print("i: ", i)
        if (points_inbox > 4):
            box_is_collision = 1
        else:
            final_gripper_center = box_center
            break

    if(box_is_collision == 1):
        return True, []
    else:
        return False, final_gripper_center

# box collision test
def plane_gripper_collision_test (all_corners, idx, R, t, w, h, params):
    # 1.generate the box_range
    big_gripper_size_para = params.big_gripper_size_para
    axis_x = R[:, 0]
    axis_y = R[:, 1]
    axis_z = R[:, 2]

    # np.savetxt("corners.txt", all_corners)

    # 2. get the corners besides the target plane
    plane_nums = int(all_corners.shape[0] / 4)
    all_other_corners = np.delete(all_corners, [idx*4, idx*4+1, idx*4+2, idx*4+3], axis = 0)
    # all_other_corners = np.vstack((all_corners[0:idx * 4 - 1, :], all_corners[(idx + 1)*4:plane_nums*4-1, :]))

    # 2. generate 4 possible gripper centers
    center_0 = t + (big_gripper_size_para[0]/2 - w / 2) * axis_x + (big_gripper_size_para[1]/2 - h / 2) * axis_y
    center_1 = t - (big_gripper_size_para[0]/2 - w / 2) * axis_x - (big_gripper_size_para[1]/2 - h / 2) * axis_y
    center_2 = t - (big_gripper_size_para[0]/2 - w / 2) * axis_x + (big_gripper_size_para[1]/2 - h / 2) * axis_y
    center_3 = t + (big_gripper_size_para[0]/2 - w / 2) * axis_x - (big_gripper_size_para[1]/2 - h / 2) * axis_y
    gripper_centers = [center_0, center_1, center_2, center_3]
    # 3.
    final_gripper_center = center_0
    for i in range(4):
        box_center = gripper_centers[i]
        points_inbox = 0
        box_is_collision = 0
        for j in range(all_other_corners.shape[0]):
            p_j = all_other_corners[j, :]
            proj_x = abs((p_j - box_center).dot(axis_x))
            proj_y = abs((p_j - box_center).dot(axis_y))
            # proj_z = abs((p_j - box_center).dot(axis_z))
            if (proj_x < big_gripper_size_para[0]/2 ):
                if (proj_y < big_gripper_size_para[1]/2):
                    points_inbox = points_inbox + 1
                # print("i: ", i)
        if (points_inbox > 0):
            box_is_collision = 1
        else:
            print("aaa")
            final_gripper_center = box_center
            break

    if(box_is_collision == 1):
        print("bbb")
        return True, []
    else:
        print("ccc")
        return False, final_gripper_center

# box collision test
def box_collision_test (cloud_ds, R, t, w, h, params):
    # 1.generate the box_range
    box_para = params.grasper_size_para
    box_range, box_range_lines = generate_box_box(R, t, w, h, params)
    box_z = box_para[3]
    axis_x = R[:, 0]
    axis_y = R[:, 1]
    axis_z = R[:, 2]

    # 2.collision detection
    box_center = box_range[8, :]
    points_inbox = 0
    box_is_collision = 0
    overlap_points = []
    for i in range(cloud_ds.shape[0]):
        p_i = cloud_ds[i, :]
        proj_x = abs((p_i - box_center).dot(axis_x))
        proj_y = abs((p_i - box_center).dot(axis_y))
        proj_z = abs((p_i - box_center).dot(axis_z))
        if (proj_x < w / 2):
            if (proj_y < h / 2):
                if (proj_z < box_z / 2):
                    points_inbox = points_inbox + 1
                    overlap_points.append(p_i)
            # print("i: ", i)
    if (points_inbox > 60):
        box_is_collision = 1
    overlap_points = np.array(overlap_points)
    print("points_inbox: ", points_inbox)
    print("box_is_collision: ", box_is_collision)

    return box_range, box_range_lines, points_inbox, box_is_collision, overlap_points


# grasper collision test
def grasper_collision_test (cloud_ds, R, t, params):
    # 1.generate the grasper_box
    grasper_box_para = params.grasper_size_para
    grasper_box, box_lines = generate_grasper_box(R, t, params)
    axis_x = R[:, 0]
    axis_y = R[:, 1]
    axis_z = R[:, 2]
    grasper_box_x = grasper_box_para[0]  # unit:m
    grasper_box_y = grasper_box_para[1]
    grasper_box_z = grasper_box_para[2] + grasper_box_para[3]
    grapser_box_sucker = grasper_box_para[3]

    # 2.collision detection
    grasper_box_center = grasper_box[8, :]
    points_inbox = 0
    box_graspable = 1
    overlap_points = []
    for i in range(cloud_ds.shape[0]):
        p_i = cloud_ds[i, :]
        proj_x = abs((p_i - grasper_box_center).dot(axis_x))
        proj_y = abs((p_i - grasper_box_center).dot(axis_y))
        proj_z = abs((p_i - grasper_box_center).dot(axis_z))
        if (proj_x < grasper_box_x / 2):
            if (proj_y < grasper_box_y / 2):
                if (proj_z < (grasper_box_z - grapser_box_sucker) / 2):
                    points_inbox = points_inbox + 1
                    overlap_points.append(p_i)
            # print("i: ", i)
    if (points_inbox > 0):
        box_graspable = 0
    overlap_points = np.array(overlap_points)
    print("points_inbox: ", points_inbox)
    print("box_graspable: ", box_graspable)

    return grasper_box, box_lines, points_inbox, box_graspable, overlap_points
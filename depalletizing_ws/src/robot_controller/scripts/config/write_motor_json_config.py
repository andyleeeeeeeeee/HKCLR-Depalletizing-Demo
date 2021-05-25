import json

data = {}
data['robot'] = []
data['robot'].append({
    'joint_size'    : 7,
    'joint_type'    : [0, 0, 0, 0, 0, 0, 0, 0],
    # DH parameter
    'DH_a'          : [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0930], # in meter
    'DH_alpha'      : [0.0, -90,  90, -90,  90, -90, -90], # in degree
    'DH_d_base'     : [0.1949, 0.0, 0.2980, 0.0, 0.2674, 0.0, 0.0],  # in miter
    'DH_q_base'     : [-90.0, 0.0, 0.0, 0.0, 0.0, -90.0, 0.0], # in degree
    'joint_L_limit' : [-360, -360, -360, -360, -360, -360, -360], # in degree
    'joint_R_limit' : [+360, +360, +360, +360, +360, +360, +360], # in degree
    'q_init'        : [0.0, +90, 0.0, -95, 0.0, 5.0, 0.0], # in degree
    'joint_name'    : ['CR2_joint1#', 'CR2_joint2#', 'CR2_joint3#', 'CR2_joint4#', 'CR2_joint5#', 'CR2_joint6#', 'CR2_joint7#'],
    # real robot or simulation
    'dt'            : 0.05,
    'robot_type'    : 'sim',
    'robot_ip'      : '127.0.0.1', 
    'robot_port'    : 20001,
    # local software
    'local_ip'      : "127.0.0.1", 
    'local_port'    : 10088,
    # remote port
    'remote_ip'     : "127.0.0.1",
    'remote_port'   : 10089
 })
data['display'] = []
data['display'].append({
    # display geometry
    'ui'            : ['CURI ROBOT UI', 0, 0, 1920, 1080],
    'ui_v_offset'   : -10,
    # joint group
    'joint_position': [0, -10],
    'joint_name'    : ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6', 'Joint7'],
    'joint_L_limit' : [-180, -105, -180, -105, -180, -105, -180], # in degree
    'joint_R_limit' : [+180, +105, +180, +105, +180, +105, +180], # in degree
    # task group
    'task_position' : [590, -10],
    'task_name'     : ['X', 'Y', 'Z', 'Roll', 'Pitch', 'Yaw', 'RCM'],
    'task_L_limit'  : [-1, -1, -1, -360, -360, -360, -0.5],
    'task_R_limit'  : [+1, +1, +1, +360, +360, +360, +0.5],
    # others
    'info_show_geometry'     : [1200,  10, 680, 850],
    'control_block_geometry' : [1200, 900, 680, 140],
    'connect_name_position'  : ['Robot', 1270, 950],
    'rcm_name_position'      : ['RCM', 1420, 950],
    'run_name_position'      : ['Run', 1570, 950],
    'demo_name_position'     : ['Demo', 1720, 950],
})

with open('motor_udp_config.json', 'w') as outfile:
    json.dump(data, outfile)

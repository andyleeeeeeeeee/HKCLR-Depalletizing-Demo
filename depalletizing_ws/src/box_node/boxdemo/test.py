import cv2
import math
import struct
import numpy as np
import time 
import os


texture_file_path = '/home/hkclr/Downloads/box_texture_template-20210412T041904Z-001/box_texture_template/existing texture'
texture_path_list = os.listdir(texture_file_path)
texture_path_list.sort()
print(texture_path_list)

for t in range(len(texture_path_list)):
    text_img_path = texture_file_path + '/' + texture_path_list[t]
    print(text_img_path)

    # 1. load images
    box_template = cv2.imread(text_img_path, 0)
    bgg_template = cv2.imread('bg_template2.jpg', 0)

    # cv2.namedWindow("bgg_template", cv2.WINDOW_NORMAL)
    # cv2.resizeWindow("bgg_template", 800, 600)
    # cv2.imshow("bgg_template", bgg_template)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()

    # 2. define stacking range
    print(box_template.shape)
    print(bgg_template.shape)
    start_pos = [272, 240]
    # 272 240   1256 240
    # 272 970   1256 970

    # 3. multi-scale
    for s in range(3):
        scale = np.random.uniform(0.9, 1.3, 1)
        box_template = cv2.resize(box_template, None, fx=scale[0], fy=scale[0], interpolation=cv2.INTER_AREA)

        rows = bgg_template.shape[0]/box_template.shape[0]
        rows = int(rows)
        cols = bgg_template.shape[1]/box_template.shape[1]
        cols = int(cols)

        syn_img = bgg_template.copy()
        syn_img_mask = np.zeros(bgg_template.shape)
        x_min = 272  # initialize a position
        y_min = 240  # initialize a position
        kk = 0
        for i in range(rows):
            for j in range(cols):
                if((i+1) * box_template.shape[1] + 272 < 1400 and (j+1) * box_template.shape[0] + 240 < 1200):
                    syn_img[j * box_template.shape[0] + y_min:(j+1) * box_template.shape[0] + y_min, i * box_template.shape[1] + x_min:(i+1) * box_template.shape[1] + x_min] = box_template
                    syn_img_mask[j * box_template.shape[0] + y_min:(j+1) * box_template.shape[0] + y_min, i * box_template.shape[1] + x_min:(i+1) * box_template.shape[1] + x_min] = 30+i*20+j*2
                    x_min = x_min + 1
                    y_min = y_min + 1
                    cv2.imwrite('./data/20210412/gray/texture_'+str(16+t)+'_scale_'+str(s)+'_'+'syn_img_'+str(kk)+'.png', syn_img)
                    cv2.imwrite('./data/20210412/label/texture_'+str(16+t)+'_scale_'+str(s)+'_'+'syn_img_mask_'+str(kk)+'.png', syn_img_mask)
                    kk = kk + 1





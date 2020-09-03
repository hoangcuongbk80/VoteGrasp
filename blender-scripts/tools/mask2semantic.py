import os
import numpy as np
import cv2
import matplotlib.pyplot as plt
import shutil

objects = {'007_tuna_fish_can':0, '008_pudding_box':1, '011_banana':2, '024_bowl':3, '025_mug':4, '044_flat_screwdriver':5,
            '051_large_clamp':6, '055_baseball':7, '061_foam_brick': 8, '065-h_cups':9}

data_dir = '/home/hoang/OSS/VoteGrasp/blender-scripts/data'
numOfScene = 10

for s in range(0, numOfScene):
    scene_dir = os.path.join(data_dir, str(s))
    semantic_dir = os.path.join(scene_dir, 'semantic')
    if os.path.isdir(semantic_dir):
        shutil.rmtree(semantic_dir)
    os.mkdir(semantic_dir)

    depth_dir = os.path.join(scene_dir, 'depth')
    num_samples = len(os.listdir(depth_dir))
    for i in range(0, num_samples):
        semantic_img = np.zeros((480, 640), dtype=np.uint8)
        for obj in objects:
            img_dir = os.path.join(scene_dir + '/mask/', obj, str(i) + '.png')
            if os.path.isfile(img_dir):  
                img = cv2.imread(img_dir, -1)
                semantic_img = np.where(img > 100, objects[obj], semantic_img)
        save_dir = os.path.join(semantic_dir, str(i) + '.png')
        cv2.imwrite(save_dir, semantic_img)
        print('saved: ', save_dir)
plt.imshow(semantic_img, aspect='auto')
plt.show()
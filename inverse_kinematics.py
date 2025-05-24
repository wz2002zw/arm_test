from kinematics import *
import numpy as np
import torch
from tqdm import tqdm

sin=lambda x:np.sin(x)
cos=lambda x:np.cos(x)
robot = RobotKinematics(link_lengths=[0.5, 0.4, 0.3, 0.2])

DIM_SIZE = 100
theta0s=np.linspace(0, 2*np.pi, DIM_SIZE)
# np.random.shuffle(theta0s)
theta1s=np.linspace(0, 2*np.pi, DIM_SIZE)
# np.random.shuffle(theta1s)
theta2s=np.linspace(0, 2*np.pi, DIM_SIZE)
# np.random.shuffle(theta2s)
theta3s=np.linspace(0, 2*np.pi, DIM_SIZE)
# np.random.shuffle(theta3s)
theta4s=np.linspace(0, 2*np.pi, DIM_SIZE)
# np.random.shuffle(theta4s)
total_iterations = len(theta0s) * len(theta1s) * len(theta2s) * len(theta3s) * len(theta4s)

# 初始化二进制存储结构
dtype = np.float32  # 使用32位浮点数减小体积
data = {
    'thetas': np.zeros((total_iterations, 5), dtype=dtype),    # 5个关节角度
    'poses': np.zeros((total_iterations, 3), dtype=dtype),     # x,y,z位置
    'rotations': np.zeros((total_iterations, 3, 3), dtype=dtype),  # 旋转矩阵
    'types': np.zeros((total_iterations, 2), dtype=dtype)      # type_1, type_2
}

# 分块处理参数
CHUNK_SIZE = 1_000_000  # 每100万条记录处理一次
num_chunks = int(np.ceil(total_iterations / CHUNK_SIZE))

# 分块生成并存储数据
with tqdm(total=total_iterations, desc="生成数据进度") as pbar:
    for chunk_idx in range(num_chunks):
        start_idx = chunk_idx * CHUNK_SIZE
        end_idx = min((chunk_idx + 1) * CHUNK_SIZE, total_iterations)
        
        # 当前块的索引范围
        current_size = end_idx - start_idx
        
        # 临时存储当前块的数据
        chunk_data = {
            'thetas': np.zeros((current_size, 5), dtype=dtype),
            'poses': np.zeros((current_size, 3), dtype=dtype),
            'rotations': np.zeros((current_size, 3, 3), dtype=dtype),
            'types': np.zeros((current_size, 2), dtype=dtype)
        }
        
        # 计算当前块的数据
        idx_in_chunk = 0
        for theta0 in theta0s:
            s0, c0 = sin(theta0), cos(theta0)
            for theta1 in theta1s:
                s1, c1 = sin(theta1), cos(theta1)
                for theta2 in theta2s:
                    s2, c2 = sin(theta2), cos(theta2)
                    for theta3 in theta3s:
                        s3, c3 = sin(theta3), cos(theta3)
                        for theta4 in theta4s:
                            s4, c4 = sin(theta4), cos(theta4)
                            
                            # 计算正运动学
                            x, y, z, r00, r01, r02, r10, r11, r12, r20, r21, r22, type_1, type_2 = \
                                robot.forward_kinematics_math_for_training(s0,c0,s1,c1,s2,c2,s3,c3,s4,c4)
                            
                            # 存储到当前块
                            chunk_data['thetas'][idx_in_chunk] = [theta0, theta1, theta2, theta3, theta4]
                            chunk_data['poses'][idx_in_chunk] = [x, y, z]
                            chunk_data['rotations'][idx_in_chunk] = [
                                [r00, r01, r02],
                                [r10, r11, r12],
                                [r20, r21, r22]
                            ]
                            chunk_data['types'][idx_in_chunk] = [type_1, type_2]
                            
                            idx_in_chunk += 1
                            pbar.update(1)
                            
                            # 如果当前块已满，跳出内层循环
                            if idx_in_chunk >= current_size:
                                break
                        else:
                            continue
                        break
                    else:
                        continue
                    break
                else:
                    continue
                break
            else:
                continue
            break
        
        # 将当前块保存到磁盘
        np.savez_compressed(
            f'./model/dataset_chunk_{chunk_idx}.npz',
            thetas=chunk_data['thetas'],
            poses=chunk_data['poses'],
            rotations=chunk_data['rotations'],
            types=chunk_data['types']
        )

print("所有数据块已生成到 ./model/ 目录")
#  正常文本方式输入
# # 创建数据文件
# BUFFER_SIZE = 10000
# with open('./model/inverse_kinematics_dataset.txt', 'w') as f:
#     # 写入表头
#     f.write("theta0,theta1,theta2,theta3,theta4,x,y,z,r00,r01,r02,r10,r11,r12,r20,r21,r22,type_1,type_2\n")
    
#     # 生成并写入数据
#     case_index  = 0
#     total_iterations = len(theta0s)*len(theta1s)*len(theta2s)*len(theta3s)*len(theta4s)
#     buffer = []
#     with tqdm(total=total_iterations, desc="生成数据进度") as pbar:
#         for theta0 in theta0s:
#             s0 = sin(theta0)
#             c0 = cos(theta0)
#             for theta1 in theta1s:
#                 s1 = sin(theta1)
#                 c1 = cos(theta1)
#                 for theta2 in theta2s:
#                     s2 = sin(theta2)
#                     c2 = cos(theta2)
#                     for theta3 in theta3s:
#                         s3 = sin(theta3)
#                         c3 = cos(theta3)
#                         for theta4 in theta4s:
#                             s4 = sin(theta4)
#                             c4 = cos(theta4)
#                             # 计算正运动学
#                             x, y, z, r00, r01, r02, r10, r11, r12, r20, r21, r22, type_1, type_2 = robot.forward_kinematics_math_for_training(s0,c0,s1,c1,s2,c2,s3,c3,s4,c4)
#                             # 写入数据行
#                             # 修改后的数据字符串格式
#                             data_str = (
#                                 f"case_id:{case_index},theta0:{theta0:.4f},theta1:{theta1:.4f},theta2:{theta2:.4f},"
#                                 f"theta3:{theta3:.4f},theta4:{theta4:.4f},x:{x:.4f},y:{y:.4f},"
#                                 f"z:{z:.4f},r00:{r00:.4f},r01:{r01:.4f},r02:{r02:.4f},"
#                                 f"r10:{r10:.4f},r11:{r11:.4f},r12:{r12:.4f},"
#                                 f"r20:{r20:.4f},r21:{r21:.4f},r22:{r22:.4f},"
#                                 f"type_1:{type_1:.4f},type_2:{type_2:.4f}\n"
#                             )
#                             buffer.append(data_str)
#                             case_index += 1
#                             if len(buffer) >= BUFFER_SIZE:
#                                 f.writelines(buffer)
#                                 buffer = []
                                
#                             pbar.update(1)  # 更新进度条
                            
#         if buffer:  # 确保最后一批数据也被写入
#             f.writelines(buffer)

# print("数据集已生成到 inverse_kinematics_dataset.txt")
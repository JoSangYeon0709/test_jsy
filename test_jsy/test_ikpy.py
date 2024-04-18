import ikpy.chain
import numpy as np
import ikpy.utils.plot as plot_utils

# URDF 파일로부터 로봇 체인 생성
my_chain = ikpy.chain.Chain.from_urdf_file(r"test_jsy/urdf/dyxel_arm_description.urdf")

# 목표 위치 설정
target_position = [0.1, 0.1, 0.1]

# 역기구학을 사용하여 각 조인트 각도 계산
joint_angles = my_chain.inverse_kinematics(target_position, orientation_mode='all')

# 로봇의 실제 위치 및 방향 계산
real_frame = my_chain.forward_kinematics(joint_angles)

# 엔드 이펙터의 위치
end_effector_position = real_frame[:3, 3]

# 그림 설정
fig, ax = plot_utils.init_3d_figure()

# 로봇 시각화
my_chain.plot(joint_angles, ax, target=target_position)

# 엔드 이펙터 위치 시각화
ax.scatter(*end_effector_position, color='r', marker='o', label='End-effector')

# 그래프 보여주기
plt.show()

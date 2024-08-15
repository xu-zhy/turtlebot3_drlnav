import os

# 文件目录路径（替换为你的文件路径）
directory = './'

# 要插入的<include>块
include_block = """
    <include>
      <pose>-2.5 2.5 0 0 0 0</pose>
      <uri>model://turtlebot3_burger</uri>
    </include>
"""

# 新的相机pose
new_camera_pose = "<pose frame=''>0.319654 -0.235002 9.29441 0 1.5138 0.009599</pose>"

# 遍历文件
for i in range(300):
    file_name = f'world_{i}.world'
    file_path = os.path.join(directory, file_name)

    # 读取文件内容
    with open(file_path, 'r') as file:
        file_content = file.read()

    file_content = file_content.replace(
        "<sdf version='1.6'>",
        "<?xml version=\"1.0\"?>\n<sdf version='1.6'>"
    )
    
    # 添加<include>块到<gui>标签之后
    file_content = file_content.replace("</gui>", f"</gui>{include_block}")

    # 修改相机的pose
    file_content = file_content.replace(
        "<pose frame=''>5 -5 2 0 0.275643 2.35619</pose>",
        new_camera_pose
    )

    # 将修改后的内容写回文件
    with open(file_path, 'w') as file:
        file.write(file_content)

print("所有文件已成功修改。")

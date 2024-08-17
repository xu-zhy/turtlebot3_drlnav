import subprocess
import time
import psutil
import sys

command = ['ros2 launch turtlebot3_gazebo turtlebot3_drl_stage11.launch.py world_file_name:=',
           'ros2 run turtlebot3_drl gazebo_goals',
           'ros2 run turtlebot3_drl environment',
           "ros2 run turtlebot3_drl test_agent td3 'examples/td3_0_stage9' 7400"]

def open_terminal_tabs(num):
    subprocess.Popen(['gnome-terminal', '--tab', '--title=gazebo', '--', 'bash', '-c', command[0] + f'world_{num}.world'])
    time.sleep(3)
    subprocess.Popen(['gnome-terminal', '--tab', '--title=goals', '--', 'bash', '-c', command[1]])
    subprocess.Popen(['gnome-terminal', '--tab', '--title=environment', '--', 'bash', '-c', command[2]])
    time.sleep(1)
    subprocess.Popen(['gnome-terminal', '--tab', '--title=agent', '--', 'bash', '-c', command[3]])

def close_terminal_tabs():
    # 这里使用 `xdotool` 来模拟关闭标签页的操作
    import os
    os.system('xdotool key ctrl+shift+w')
    
def is_gzserver_running():
    # 获取所有正在运行的进程
    for proc in psutil.process_iter(['pid', 'name']):
        if proc.info['name'] == 'gzserver':
            return True
    return False

if __name__ == "__main__":
    args = sys.argv[1:]
    num = int(args[0])
    # 打开 4 个标签页
    open_terminal_tabs(num)
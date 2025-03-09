##
# @File: parameter_tuning.launch.py
#

import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable

def generate_launch_description():
    return LaunchDescription([
        #SetEnvironmentVariable('ASAN_OPTIONS', 'detect_invalid_pointer_pairs=2:new_delete_type_mismatch=0:quarantine_size_mb=512:check_initialization_order=1:detect_stack_use_after_return=1:print_stats=1:atexit=1:detect_leaks=1'),
        #SetEnvironmentVariable('TSAN_OPTIONS', 'report_atomic_races=0 suppressions=/home/james/ros_ws/thread_sanitizer_suppress.txt'),
        #SetEnvironmentVariable('TSAN_OPTIONS', 'report_atomic_races=0 suppressions=/home/james/ros_ws/thread_sanitizer_suppress.txt history_size=7'),
        #SetEnvironmentVariable('CPUPROFILE', '/home/james/ros_ws/test.prof'),
        #SetEnvironmentVariable('CPUPROFILE_FREQUENCY', '1000'),
        Node(
            package='surveillance_planner',
            executable='planner_tuner_server',
            name='planner_tuner_server',
            output='screen',
            #prefix=['setarch x86_64 -R'],
            #prefix=['xterm -sl 9999999 -maximized -e '],
            #prefix=['xterm -sl 9999999 -maximized -e gdb -ex run -ex backtrace --args'],
            #prefix=['gnome-terminal --wait --maximize -- gdb -ex run --args'],
            #prefix=['valgrind --read-var-info=yes --error-limit=no --show-error-list=yes --max-threads=999999 --tool=drd --check-stack-var=yes'],
            #prefix=['valgrind --read-var-info=yes --error-limit=no --show-error-list=yes --max-threads=999999 --tool=helgrind'],
            #parameters=[]
            #prefix="nice -n 5",
            #respawn=True,
            respawn_delay=30,
            on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
        ),
        Node(
            package='surveillance_planner',
            executable='planner_tuner.py',
            name='planner_tuner',
            output='screen',
            prefix=['nice -n 5'],
            #prefix=['setarch x86_64 -R'],
            #prefix=['xterm -sl 9999999 -maximized -e '],
            #prefix=['xterm -sl 9999999 -maximized -e gdb -ex run -ex backtrace --args'],
            #prefix=['gnome-terminal --wait --maximize -- gdb -ex run --args'],
            #prefix=['valgrind --read-var-info=yes --error-limit=no --show-error-list=yes --max-threads=999999 --tool=drd --check-stack-var=yes'],
            #prefix=['valgrind --read-var-info=yes --error-limit=no --show-error-list=yes --max-threads=999999 --tool=helgrind'],
            #parameters=[]
        ),
    ])



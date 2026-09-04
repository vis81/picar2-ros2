from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    explorer = LaunchConfiguration('explorer')

    is_ours = PythonExpression(["'", explorer, "' != 'explore_lite'"])
    is_lite = PythonExpression(["'", explorer, "' == 'explore_lite'"])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        # Default comes from PICAR_EXPLORER so the web UI (which launches this
        # file with no arguments) can be pointed at either explorer without
        # editing code. Unset -> 'frontier'.
        DeclareLaunchArgument(
            'explorer',
            default_value=EnvironmentVariable('PICAR_EXPLORER',
                                              default_value='explore_lite'),
            description="'frontier' (ours, Ackermann-aware) or 'explore_lite'"),

        # Ours. Aims at drivable stand-off poses facing the unknown, scores
        # candidates with the real planner, and commits to a goal instead of
        # re-sending one every cycle. See frontier_explorer.py for why
        # explore_lite's model doesn't fit a car.
        Node(
            package='picar2_bringup',
            executable='frontier_explorer.py',
            name='frontier_explorer',
            output='screen',
            condition=IfCondition(is_ours),
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_base_frame': 'base_footprint',
                'costmap_topic': '/global_costmap/costmap',
                'plan_period': 2.0,
                'free_threshold': 50,
                'min_frontier_size': 0.4,
                'min_goal_distance': 0.5,
                'turn_radius': 0.5,
                'turn_weight': 1.0,
                'gain_weight': 2.0,
                'commit_seconds': 10.0,
                'switch_margin': 1.5,
                'useful_radius': 1.0,
                'arrive_radius': 0.4,
                'stuck_seconds': 20.0,
                'stuck_distance': 0.6,
                'futile_distance': 0.15,
                'verify_top_k': 3,
                'blacklist_seconds': 45.0,
                'blacklist_radius': 0.5,
                # after this many frontiers fail without the robot moving,
                # drive to open space instead of trying another frontier
                'escape_after': 3,
                'escape_radius': 4.0,
            }],
        ),

        # explore_lite needs a grid whose free space is *exactly* cost 0 and
        # connected; see explore_map_trinary.py for why neither Nav2's inflated
        # global costmap nor cartographer's graded /map qualifies. A standalone
        # nav2_costmap_2d was tried first and is the wrong tool: it is a
        # lifecycle node that blocks on its own TF buffer, and none of its
        # machinery is needed to answer "where is the unknown space".
        Node(
            package='picar2_bringup',
            executable='explore_map_trinary.py',
            name='explore_map_trinary',
            output='screen',
            condition=IfCondition(is_lite),
            parameters=[{
                'use_sim_time': use_sim_time,
                'lethal_threshold': 65,
                'in_topic': '/map',
                'out_topic': '/explore_costmap/costmap',
            }],
        ),

        # Kept for A/B comparison: explorer:=explore_lite
        Node(
            package='explore_lite',
            executable='explore',
            name='explore',
            output='screen',
            condition=IfCondition(is_lite),
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_base_frame': 'base_footprint',
                # NOT Nav2's global costmap. explore_lite's frontier search is
                # a *descending* BFS (frontier_search.cpp:70,
                # `map_[nbr] <= map_[idx]`), so from a FREE_SPACE cell it can
                # only step to other cost==0 cells — it explores the connected
                # component of zero-cost space around the robot. Nav2's
                # inflation (0.40 / scaling 10) shatters that into islands:
                # measured on hardware, the robot's component was 191 cells of
                # 21453 free (0.9%) and reached 0 of 901 frontier cells, so
                # explore_lite declared EXPLORATION_COMPLETE after 5 minutes
                # with the room half unmapped. Nav2 is immune because Smac
                # treats inflated cells as costly, not blocked.
                'costmap_topic': '/explore_costmap/costmap',
                'costmap_updates_topic': '/explore_costmap/costmap_updates',
                'visualize': True,
                # 0.1, not 0.25: explore_lite sends a new goal whenever the
                # frontier centroid moves >1 cm, and Nav2 is a single-goal
                # server, so every re-plan preempts the current goal.
                'planner_frequency': 0.1,
                'progress_timeout': 15.0,
                'potential_scale': 3.0,
                'gain_scale': 1.0,
                'transform_tolerance': 0.3,
                'min_frontier_size': 0.5,
            }],
        ),
    ])

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
navigation_layer_node.py — costmap + A* planning node.

Subscribes:
    /occupancy_grid      (nav_msgs/OccupancyGrid)     from occupancy_grid
    /dynamic_obstacles   (adaptive_amr_msgs/DynamicObstacleArray)
    /goal                (geometry_msgs/PoseStamped)  set the goal
Service:
    /navigation_layer/set_goal  (adaptive_amr_msgs/SetGoal)
Publishes:
    /navigation_costmap  (nav_msgs/OccupancyGrid, inflated)
    /planned_path        (nav_msgs/Path)
    /navigation_layer/statistics (DiagnosticArray)

Run:
    rosrun navigation_layer navigation_layer_node.py
"""

import numpy as np
import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from std_msgs.msg import Header

from adaptive_amr_msgs.srv import SetGoal, SetGoalResponse
from navigation_layer.costmap import CostmapBuilder
from navigation_layer.planner import AStarPlanner


class NavigationLayerNode:
    """Builds the costmap and plans A* paths."""

    def __init__(self):
        # ---- configuration ----------------------------------------------------
        self.grid_topic = rospy.get_param("~occupancy_topic", "/occupancy_grid")
        self.obstacles_topic = rospy.get_param("~obstacles_topic",
                                               "/dynamic_obstacles")
        self.goal_topic = rospy.get_param("~goal_topic", "/goal")
        self.costmap_topic = rospy.get_param("~costmap_topic",
                                             "/navigation_costmap")
        self.path_topic = rospy.get_param("~path_topic", "/planned_path")
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.inflation_radius = rospy.get_param("~inflation_radius", 1.0)
        self.lethal_threshold = rospy.get_param("~lethal_threshold", 50)
        self.unknown_penalty = rospy.get_param("~unknown_penalty", 0.5)
        self.plan_interval = rospy.get_param("~plan_interval", 1.0)   # s
        self.diagnostics_rate = rospy.get_param("~diagnostics_rate", 1.0)

        # ---- state ---------------------------------------------------------------
        self.latest_grid = None
        self.latest_obstacles = None
        self.goal = None                      # (x, y) world in map frame
        self.costmap = None
        self.last_plan_time = rospy.Time(0)

        self.builder = CostmapBuilder(self.inflation_radius, self.lethal_threshold)

        # ---- ROS -------------------------------------------------------------------
        self.pub_costmap = rospy.Publisher(self.costmap_topic, OccupancyGrid,
                                           queue_size=1, latch=True)
        self.pub_path = rospy.Publisher(self.path_topic, Path, queue_size=1)
        self.pub_stats = rospy.Publisher(self.navigation_stats_topic(),
                                         DiagnosticArray, queue_size=5)
        rospy.Subscriber(self.grid_topic, OccupancyGrid, self._on_grid)
        rospy.Subscriber(self.obstacles_topic, ObstacleArrayType(),
                         self._on_obstacles)
        rospy.Subscriber(self.goal_topic, PoseStamped, self._on_goal)
        self.srv_set_goal = rospy.Service("/navigation_layer/set_goal",
                                          SetGoal, self._on_set_goal)
        rospy.Timer(rospy.Duration(1.0 / max(self.diagnostics_rate, 0.1)),
                    self._publish_statistics)
        rospy.loginfo("navigation_layer: %s + %s -> %s + %s",
                      self.grid_topic, self.obstacles_topic, self.costmap_topic,
                      self.path_topic)

    def navigation_stats_topic(self):
        return "/navigation_layer/statistics"

    # ------------------------------------------------------------------ #
    def _on_grid(self, msg: OccupancyGrid) -> None:
        self.latest_grid = msg
        self._rebuild()

    def _on_obstacles(self, msg) -> None:
        self.latest_obstacles = msg
        self._rebuild()

    def _on_goal(self, msg: PoseStamped) -> None:
        self.goal = (msg.pose.position.x, msg.pose.position.y)
        rospy.loginfo("navigation_layer: goal set to (%.2f, %.2f)",
                      self.goal[0], self.goal[1])

    def _on_set_goal(self, req) -> SetGoalResponse:
        self.goal = (req.goal.pose.position.x, req.goal.pose.position.y)
        rospy.loginfo("navigation_layer: goal set via service to (%.2f, %.2f)",
                      self.goal[0], self.goal[1])
        return SetGoalResponse(success=True,
                               message="goal accepted ({:.2f}, {:.2f})".format(
                                   self.goal[0], self.goal[1]))

    # ------------------------------------------------------------------ #
    def _rebuild(self) -> None:
        if self.latest_grid is None:
            return
        grid = self.latest_grid
        occ = np.asarray(grid.data, dtype=np.int8).reshape(grid.info.height,
                                                           grid.info.width)

        # dynamic obstacles -> cell coordinates (transform to map frame)
        dynamic_cells = None
        if self.latest_obstacles is not None:
            dynamic_cells = self._obstacle_cells(self.latest_obstacles, grid)

        costmap = self.builder.build(occ, grid.info.resolution, dynamic_cells)
        self.costmap = costmap
        self._publish_costmap(grid, costmap)

        # plan when a goal exists and enough time passed
        now = rospy.Time.now()
        if (self.goal is not None and
                (now - self.last_plan_time).to_sec() >= self.plan_interval):
            self._plan(grid, costmap)

    # ------------------------------------------------------------------ #
    def _obstacle_cells(self, obstacles, grid):
        """Convert dynamic obstacle positions (laser frame) into map cells."""
        try:
            from tf2_ros import Buffer, TransformListener
            buf = Buffer()
            TransformListener(buf)
            stamped = buf.lookup_transform(self.map_frame, "laser",
                                          rospy.Time(0),
                                          timeout=rospy.Duration(1.0))
            t = stamped.transform
        except Exception:  # noqa: BLE001 - no TF: skip dynamic inflation
            rospy.logwarn_throttle(5.0, "navigation_layer: no map->laser TF, "
                                        "skipping dynamic obstacles")
            return None
        T = _transform_matrix(t)
        cells = []
        for obs in obstacles.obstacles:
            for pt in [obs.position] + list(obs.predicted_trajectory):
                p_laser = np.array([pt.x, pt.y, pt.z, 1.0])
                p_map = T @ p_laser
                row, col = CostmapBuilder.world_to_cell(
                    p_map[0], p_map[1],
                    grid.info.origin.position.x, grid.info.origin.position.y,
                    grid.info.resolution)
                cells.append((row, col))
        return np.asarray(cells, dtype=int) if cells else None

    # ------------------------------------------------------------------ #
    def _plan(self, grid, costmap) -> None:
        planner = AStarPlanner(costmap, grid.info.resolution,
                               grid.info.origin.position.x,
                               grid.info.origin.position.y,
                               self.unknown_penalty)
        # robot start: from TF map->base_link
        try:
            from tf2_ros import Buffer, TransformListener
            buf = Buffer()
            TransformListener(buf)
            stamped = buf.lookup_transform(self.map_frame, self.base_frame,
                                           rospy.Time(0),
                                           timeout=rospy.Duration(1.0))
            start = (stamped.transform.translation.x,
                     stamped.transform.translation.y)
        except Exception:  # noqa: BLE001
            start = (0.0, 0.0)
            rospy.logwarn_throttle(5.0, "navigation_layer: no robot TF, "
                                        "planning from origin")

        path = planner.plan(start, self.goal)
        self.last_plan_time = rospy.Time.now()
        self._publish_path(grid, path)

    # ------------------------------------------------------------------ #
    def _publish_costmap(self, grid, costmap) -> None:
        msg = OccupancyGrid()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.map_frame
        msg.info = grid.info
        msg.data = costmap.ravel(order="C").tolist()
        self.pub_costmap.publish(msg)

    def _publish_path(self, grid, path) -> None:
        msg = Path()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.map_frame
        if path is not None:
            for x, y in path:
                pose = PoseStamped()
                pose.header = msg.header
                pose.pose.position.x = float(x)
                pose.pose.position.y = float(y)
                pose.pose.orientation.w = 1.0
                msg.poses.append(pose)
        self.pub_path.publish(msg)

    # ------------------------------------------------------------------ #
    def _publish_statistics(self, _event=None) -> None:
        status = DiagnosticStatus()
        status.level = (DiagnosticStatus.OK if self.costmap is not None
                        else DiagnosticStatus.WARN)
        status.name = "navigation_layer"
        status.message = "costmap ready" if self.costmap is not None \
            else "waiting for /occupancy_grid"
        status.values = [
            KeyValue(key="goal", value=("({:.2f}, {:.2f})".format(*self.goal)
                                        if self.goal else "none")),
            KeyValue(key="costmap", value=str(self.costmap is not None)),
        ]
        array = DiagnosticArray()
        array.header.stamp = rospy.Time.now()
        array.status.append(status)
        self.pub_stats.publish(array)

    def run(self) -> None:
        rospy.spin()


def ObstacleArrayType():
    from adaptive_amr_msgs.msg import DynamicObstacleArray
    return DynamicObstacleArray


def _transform_matrix(t) -> np.ndarray:
    q = t.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    R = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [t.translation.x, t.translation.y, t.translation.z]
    return T


def main():
    rospy.init_node("navigation_layer_node", anonymous=False)
    try:
        NavigationLayerNode().run()
    except rospy.ROSInterruptException:
        rospy.loginfo("navigation_layer interrupted.")
    except Exception as exc:
        rospy.logfatal("navigation_layer failed: %s", exc)
        raise


if __name__ == "__main__":
    main()

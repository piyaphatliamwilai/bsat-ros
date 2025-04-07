#!/usr/bin/env python3
import rospy
import math
import sys
import heapq
import actionlib
import tf
from ired_refbox.msg import NavigationRoutesData, NavigationRoutes, NavigationRoutesPoint, GamePlay
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Point, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class moveBaseAction:
    def __init__(self):
        rospy.loginfo("Waiting for AMCL...")
        rospy.wait_for_message('/amcl_pose', PoseWithCovarianceStamped)
        rospy.loginfo("Waiting for REFBOX...")
        rospy.wait_for_message('/refbox/game_play', GamePlay)
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amclCallback)
        rospy.Subscriber("/refbox/game_play", GamePlay, self.gamePlayCallback)
        self.move_base_action = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        self.move_base_action.wait_for_server(rospy.Duration(5))
        self.game_play = False

    def amclCallback(self, msg):
        robot_pose_ = msg.pose.pose
        robot_theta_ = tf.transformations.euler_from_quaternion((robot_pose_.orientation.x, robot_pose_.orientation.y, robot_pose_.orientation.z, robot_pose_.orientation.w))
        rospy.loginfo(f"[AMCL] Robot Pose=> x:{robot_pose_.position.x} y:{robot_pose_.position.y} theta:{robot_theta_[2]}")

    def gamePlayCallback(self, msg):
        game_play_msg = msg
        self.game_play = game_play_msg.status

        if not self.game_play:
            rospy.loginfo_once("[REFBOX] Game Pause...")
            self.move_base_action.cancel_goal()
        else:
            rospy.loginfo_once("[REFBOX] Playing navigation task...")
            
    
    def createGoal(self, x:float, y:float, theta:float):
        quat = quaternion_from_euler(0, 0, theta)
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(Point(x, y, 0.0), Quaternion(quat[0], quat[1], quat[2], quat[3]))
        
        return goal
    
    def moveToPoint(self, x:float, y:float, theta:float):
        target_point = self.createGoal(x, y, theta)
        if self.game_play:
            return self.moveToGoal(target_point)
        return False
        
    def moveToGoal(self, goal):
        self.move_base_action.send_goal(goal)
        success = self.move_base_action.wait_for_result()
        state = self.move_base_action.get_state()
        quat_ = goal.target_pose.pose.orientation
        theta_ = tf.transformations.euler_from_quaternion((quat_.x, quat_.y, quat_.z, quat_.w))
        rospy.loginfo(f"Move to x: {goal.target_pose.pose.position.x} y: {goal.target_pose.pose.position.y} theta: {theta_[2]}")
        if success and state == GoalStatus.SUCCEEDED:
            rospy.loginfo("[MBA] Complete")
            return True
        else:
            rospy.loginfo("[MBA] Fail")
            self.move_base_action.cancel_goal()
            return False


class ShortestRoutePlanner:
    def __init__(self):
        rospy.init_node("shortest_route_planner")
        
        # Current robot position
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_angle = 0.0
        
        # Store all zones with their positions
        self.zone_positions = {}
        
        # Configuration for initial position
        self.initial_x = rospy.get_param('~initial_x', 0.5)
        self.initial_y = rospy.get_param('~initial_y', 0.5)
        self.initial_theta = rospy.get_param('~initial_theta', 0.0)
        self.initial_wait_time = rospy.get_param('~initial_wait_time', 5.0)
        
        # Initialize move base action
        self.move_base = moveBaseAction()
        
        # Subscribe to navigation routes data
        self.routes_sub = rospy.Subscriber(
            "/refbox/navigation_routes", 
            NavigationRoutesData, 
            self.routes_callback
        )
        
        # Subscribe to robot position
        self.pose_sub = rospy.Subscriber(
            "/amcl_pose", 
            PoseWithCovarianceStamped, 
            self.robot_position_callback
        )
        
        # Publisher for optimized routes
        self.optimized_routes_pub = rospy.Publisher(
            "/planner/optimized_routes", 
            NavigationRoutesData, 
            queue_size=10
        )
        
        rospy.loginfo("Shortest Route Planner: Initialized!")
        rospy.loginfo(f"Initial position set to: ({self.initial_x}, {self.initial_y}, {self.initial_theta})")
        rospy.loginfo(f"Will wait {self.initial_wait_time} seconds at initial position")
        rospy.loginfo("Waiting for navigation routes data...")

    def robot_position_callback(self, msg):
        """Callback for robot position updates"""
        pose = msg.pose.pose
        self.current_x = pose.position.x
        self.current_y = pose.position.y
        
        # Convert quaternion to euler angles
        quaternion = [
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w
        ]
        euler = euler_from_quaternion(quaternion)
        # Store in radians for move_base
        self.current_angle = euler[2]

    def routes_callback(self, routes_data):
        """Process navigation routes and optimize them"""
        rospy.loginfo("Received navigation routes data with %d routes", len(routes_data.routes))
        
        # Extract zone positions from routes
        for route in routes_data.routes:
            for point in route.route + route.remaining:
                zone_name = point.zone
                self.zone_positions[zone_name] = (point.position.x, point.position.y)
        
        # Create and publish optimized routes
        optimized_routes = self.optimize_routes(routes_data)
        self.optimized_routes_pub.publish(optimized_routes)
        rospy.loginfo("Published optimized routes")
        
        # Execute the first optimized route
        if len(optimized_routes.routes) > 0 and len(optimized_routes.routes[0].route) > 0:
            self.execute_route(optimized_routes.routes[0])

    def calculate_distance(self, point1, point2):
        """Calculate Euclidean distance between two points"""
        return math.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)

    def optimize_routes(self, routes_data):
        """Create optimized routes from the original routes data"""
        optimized_data = NavigationRoutesData()
        optimized_data.teamColor = routes_data.teamColor
        
        for route in routes_data.routes:
            optimized_route = NavigationRoutes()
            optimized_route.id = route.id
            
            # Combine route and remaining points to get all points in this route
            all_points = route.route + route.remaining
            if not all_points:
                continue
                
            # Start optimization from current robot position
            current_pos = (self.current_x, self.current_y)
            unvisited = list(range(len(all_points)))
            path = []
            
            # Greedy nearest neighbor algorithm
            while unvisited:
                nearest_idx = min(
                    unvisited,
                    key=lambda i: self.calculate_distance(
                        current_pos, 
                        (all_points[i].position.x, all_points[i].position.y)
                    )
                )
                path.append(nearest_idx)
                current_pos = (all_points[nearest_idx].position.x, all_points[nearest_idx].position.y)
                unvisited.remove(nearest_idx)
            
            # Create new route with optimized path
            for idx in path:
                optimized_route.route.append(all_points[idx])
            
            # For now, the remaining list is empty since we've included everything in the route
            optimized_data.routes.append(optimized_route)
            
            # Print route information
            route_names = [all_points[idx].zone for idx in path]
            rospy.loginfo(f"Optimized route {route.id}: {' -> '.join(route_names)}")
        
        return optimized_data

    def execute_route(self, route):
        """Navigate through all points in the given route"""
        rospy.loginfo(f"Executing route with ID: {route.id}")
        
        # Move to initial position first
        rospy.loginfo(f"Moving to initial position ({self.initial_x}, {self.initial_y}) before starting route")
        success = self.move_base.moveToPoint(self.initial_x, self.initial_y, self.initial_theta)
        
        if success:
            rospy.loginfo("Successfully reached initial position")
            # Wait for specified seconds
            rospy.loginfo(f"Waiting for {self.initial_wait_time} seconds before starting navigation...")
            rospy.sleep(self.initial_wait_time)
            rospy.loginfo("Starting route navigation")
        else:
            rospy.logwarn("Failed to reach initial position, starting navigation anyway")
        
        # Now navigate through all points in the optimized route
        for point in route.route:
            zone_name = point.zone
            x = point.position.x
            y = point.position.y
            
            # Calculate heading based on next point if available
            theta = self.current_angle  # Default to current heading
            
            # Log navigation start
            rospy.loginfo(f"Navigating to zone {zone_name} at ({x}, {y})")
            
            # Send move base command
            success = self.move_base.moveToPoint(x, y, theta)
            
            if success:
                rospy.loginfo(f"Successfully reached zone {zone_name}")
            else:
                rospy.logwarn(f"Failed to reach zone {zone_name}, continuing to next target")
        
        rospy.loginfo(f"Route {route.id} execution completed")

    def calculate_shortest_path(self, start_pos, zones):
        """Find shortest path through all zones using Dijkstra's algorithm"""
        if not zones:
            return []
            
        # Create a graph of all zones and distances
        graph = {}
        positions = {zone.zone: (zone.position.x, zone.position.y) for zone in zones}
        positions['start'] = start_pos
        
        # Create a fully connected graph with distances
        for zone1 in list(positions.keys()):
            graph[zone1] = {}
            for zone2 in list(positions.keys()):
                if zone1 != zone2:
                    dist = self.calculate_distance(positions[zone1], positions[zone2])
                    graph[zone1][zone2] = dist
        
        # Find shortest path through all zones
        # Using a simplified version of TSP with nearest neighbor
        current = 'start'
        path = []
        unvisited = set(positions.keys()) - {'start'}
        
        while unvisited:
            next_zone = min(unvisited, key=lambda x: graph[current][x])
            path.append(next_zone)
            current = next_zone
            unvisited.remove(next_zone)
            
        # Convert zone names back to indices
        zone_to_idx = {zone.zone: i for i, zone in enumerate(zones)}
        path_indices = [zone_to_idx[zone] for zone in path]
        
        return path_indices

def main():
    try:
        planner = ShortestRoutePlanner()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()

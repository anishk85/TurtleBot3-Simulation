#!/usr/bin/env python3
"""
Frontier Detection for Autonomous Exploration - Optimized for Small House
Detects frontiers (boundaries between known free space and unknown areas)
"""

import numpy as np
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point
import cv2
from typing import List, Tuple

class FrontierDetector:
    def __init__(self, min_frontier_size=3):  # Smaller for small house
        """
        Initialize frontier detector
        
        Args:
            min_frontier_size: Minimum number of cells for a valid frontier
        """
        self.min_frontier_size = min_frontier_size
        
    def detect_frontiers(self, occupancy_grid: OccupancyGrid) -> List[List[Point]]:
        """
        Detect frontiers in the occupancy grid
        
        Args:
            occupancy_grid: ROS OccupancyGrid message
            
        Returns:
            List of frontier regions, each containing list of Points
        """
        # Convert ROS message to numpy array
        width = occupancy_grid.info.width
        height = occupancy_grid.info.height
        resolution = occupancy_grid.info.resolution
        origin = occupancy_grid.info.origin
        
        # Reshape data to 2D array
        grid_data = np.array(occupancy_grid.data).reshape((height, width))
        
        # Find frontier cells
        frontier_mask = self._find_frontier_cells(grid_data)
        
        # Group frontier cells into regions
        frontier_regions = self._group_frontier_cells(frontier_mask)
        
        # Convert pixel coordinates to world coordinates
        world_frontiers = []
        for region in frontier_regions:
            if len(region) >= self.min_frontier_size:
                world_region = []
                for pixel in region:
                    world_point = self._pixel_to_world(
                        pixel, resolution, origin, height
                    )
                    world_region.append(world_point)
                world_frontiers.append(world_region)
        
        return world_frontiers
    
    def _find_frontier_cells(self, grid_data: np.ndarray) -> np.ndarray:
        """
        Find cells that are frontiers (adjacent to both free and unknown space)
        Enhanced for small house detection
        
        Args:
            grid_data: 2D numpy array of occupancy grid
            
        Returns:
            Binary mask of frontier cells
        """
        height, width = grid_data.shape
        frontier_mask = np.zeros((height, width), dtype=bool)
        
        # Define 8-connected neighborhood
        directions = [(-1, -1), (-1, 0), (-1, 1), (0, -1), 
                     (0, 1), (1, -1), (1, 0), (1, 1)]
        
        for i in range(1, height - 1):
            for j in range(1, width - 1):
                # Current cell must be unknown (-1)
                if grid_data[i, j] != -1:
                    continue
                
                # Check if adjacent to free space (0)
                has_free_neighbor = False
                occupied_neighbors = 0
                
                for di, dj in directions:
                    ni, nj = i + di, j + dj
                    if 0 <= ni < height and 0 <= nj < width:
                        if grid_data[ni, nj] == 0:  # Free space
                            has_free_neighbor = True
                        elif grid_data[ni, nj] == 100:  # Occupied space
                            occupied_neighbors += 1
                
                # For small house: be more aggressive in frontier detection
                # Accept frontiers with at least one free neighbor and not too many occupied neighbors
                if has_free_neighbor and occupied_neighbors < 6:
                    frontier_mask[i, j] = True
        
        return frontier_mask
    
    def _group_frontier_cells(self, frontier_mask: np.ndarray) -> List[List[Tuple[int, int]]]:
        """
        Group adjacent frontier cells into regions using connected components
        
        Args:
            frontier_mask: Binary mask of frontier cells
            
        Returns:
            List of frontier regions (each region is list of (row, col) tuples)
        """
        # Convert boolean mask to uint8 for OpenCV
        mask_uint8 = frontier_mask.astype(np.uint8) * 255
        
        # Find connected components
        num_labels, labels = cv2.connectedComponents(mask_uint8, connectivity=8)
        
        # Group pixels by label
        regions = []
        for label in range(1, num_labels):  # Skip background (label 0)
            region_pixels = np.where(labels == label)
            region = list(zip(region_pixels[0], region_pixels[1]))
            regions.append(region)
        
        return regions
    
    def _pixel_to_world(self, pixel: Tuple[int, int], resolution: float, 
                       origin, height: int) -> Point:
        """
        Convert pixel coordinates to world coordinates
        
        Args:
            pixel: (row, col) in image coordinates
            resolution: Map resolution (meters per pixel)
            origin: Map origin pose
            height: Map height in pixels
            
        Returns:
            Point in world coordinates
        """
        row, col = pixel
        
        # Convert image coordinates to map coordinates
        # Note: Image row 0 is at the top, but map y=0 is at the bottom
        map_x = col * resolution + origin.position.x
        map_y = (height - 1 - row) * resolution + origin.position.y
        
        point = Point()
        point.x = map_x
        point.y = map_y
        point.z = 0.0
        
        return point
    
    def get_frontier_centroids(self, frontier_regions: List[List[Point]]) -> List[Point]:
        """
        Calculate centroid of each frontier region
        
        Args:
            frontier_regions: List of frontier regions
            
        Returns:
            List of centroid points
        """
        centroids = []
        for region in frontier_regions:
            if not region:
                continue
                
            # Calculate centroid
            sum_x = sum(point.x for point in region)
            sum_y = sum(point.y for point in region)
            
            centroid = Point()
            centroid.x = sum_x / len(region)
            centroid.y = sum_y / len(region)
            centroid.z = 0.0
            
            centroids.append(centroid)
        
        return centroids
    
    def filter_frontiers_by_distance(self, frontier_centroids: List[Point], 
                                   robot_position: Point, min_distance: float = 0.3) -> List[Point]:
        """
        Filter out frontiers that are too close to robot (optimized for small house)
        
        Args:
            frontier_centroids: List of frontier centroid points
            robot_position: Current robot position
            min_distance: Minimum distance threshold (reduced for small house)
            
        Returns:
            Filtered list of frontier centroids
        """
        filtered_frontiers = []
        
        for frontier in frontier_centroids:
            distance = np.sqrt(
                (frontier.x - robot_position.x) ** 2 + 
                (frontier.y - robot_position.y) ** 2
            )
            
            # For small house, accept closer frontiers but with reasonable minimum
            if distance >= min_distance:
                filtered_frontiers.append(frontier)
        
        return filtered_frontiers
    
    def filter_frontiers_by_size_and_accessibility(self, frontier_regions: List[List[Point]], 
                                                 min_size: int = 2) -> List[List[Point]]:
        """
        Filter frontiers by size and potential accessibility for small house
        
        Args:
            frontier_regions: List of frontier regions
            min_size: Minimum region size (smaller for small house)
            
        Returns:
            Filtered frontier regions
        """
        filtered_regions = []
        
        for region in frontier_regions:
            # Size filter
            if len(region) < min_size:
                continue
            
            # For small house, be less restrictive about frontier size
            # since rooms are smaller and frontiers may be smaller too
            filtered_regions.append(region)
        
        return filtered_regions
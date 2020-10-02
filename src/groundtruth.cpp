#include "bev_converter/groundtruth.h"

GroundTruth::GroundTruth(void)
:local_nh("~")
{
}


void GroundTruth::

	for(int i=0;i<size;i++){
	pointcloud->points[i].x = 
	
	}

int GroundTruth::get_index_from_xy(const double x,const double y)
{
	int _x = floor(x / RESOLUTION + 0.5) + GRID_WIDTH_2;
	int _y = floor(y / RESOLUTION + 0.5) + GRID_WIDTH_2;
	return _y * GRID_WIDTH_2 + _x;
}
void GroundTruth::input_cloud_to_occupancy_grid_map(const CloudXYZIPtr& cloud_ptr)
{

    int cloud_size = cloud_ptr->points.size();

    for(int i=0;i<cloud_size;i++){
        auto p = cloud_ptr->points[i];
        if(!is_valid_point(p.x, p.y)){
            continue;
        }
        // occupancy_grid_map[get_index_from_xy(p.x, p.y)].add_log_odds(0.01);
        double distance = sqrt(p.x * p.x + p.y * p.y);
        double direction = atan2(p.y, p.x);
        
        obstacle_indices[get_index_from_xy(p.x, p.y)] = true;
    }

    for(int i=0;i<GRID_NUM;i++){
        if(obstacle_indices[i]){
            occupancy_grid_map[i].add_log_odds(0.4);
        }
    }

    set_clear_grid_cells(beam_list, obstacle_indices, occupancy_grid_map);
}
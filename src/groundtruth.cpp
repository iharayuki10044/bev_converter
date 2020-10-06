#include "bev_converter/groundtruth.h"

GroundTruth::GroundTruth(void)
:local_nh("~")
{
    local_nh.param("RESOLUTION", RESOLUTION, {});
    local_nh.param("WIDTH", WIDTH, {});
    local_nh.param("WIDTH_2", WIDTH_2,{});
    local_nh.param("GRID_WIDTH", GRID_WIDTH, {});
    local_nh.param("GRID_WIDTH", GRID_WIDTH_2, {});
    local_nh.param("GRID_NUM", GRID_NUM, {});
    local_nh.param("PEOPLE_NUM", PEOPLE_NUM, {});

}

void GroundTruth::executor(void)
{
    formatter();

    ros::Rate r(Hz);
	while(ros::ok()){




		if(){


        
		}

		r.sleep();
		ros::spinOnce();
    }


}

void GroundTruth::formatter(void)
{
	/* std::cout << "formatter" << std::endl; */

    dt = 1.0 / Hz;
    grid_size = RANGE / GRID_NUM;
}


void GroundTruth::calculation_peple_point(const CloudXYZIPtr& cloud_ptr)
{
    std::cout << "--- calculation people point ---" << std::endl;
    int cloud_size = cloud_ptr->points.size();

    Eigen::Matrix<double, 3, PEOPLE_NUM> people_point = MatrixXD::Zero(3, PEOPLE_NUM);
    //(x, y, hit_num)*people_num Zero init

    for(int i=0;i<cloud_size;i++){
        auto p = cloud_ptr->points[i];
        double id = cloud_ptr->intensity[i] - 1;
        people_point(1, id) += p.x;
        people_point(2, id) += p.y;
        people_point(3, id) += 1;
    }
    
    for(int i=0;i<PEOPLE_NUM;i++){
        
        if(people_point != 0){
            people_data[i].x = people_point(1, id) /people(3, id);
            people_data[i].y = people_point(2, id) /people(3, id);
        }
    }
}

void GroundTruth::calculation_people_vector(PeopleData &now, PeopleData &past)
{
    for(int i=0;i<PEOPLE_NUM;i++){
        past[i].move_vector_x = now[i].point_x - past[i].point.x;
        past[i].move_vector_y = now[i].point_y - past[i].point.y;
    }
}


int GroundTruth::get_index_from_xy(const double x,const double y)
{
	int _x = floor(x / RESOLUTION + 0.5) + GRID_WIDTH_2;
	int _y = floor(y / RESOLUTION + 0.5) + GRID_WIDTH_2;
	return _y * GRID_WIDTH_2 + _x;
}

int GroundTruth::get_x_index_from_index(const int index)
{
    return index % GRID_WIDTH;
}

int GroundTruth::get_y_index_from_index(const int index)
{
    return index / GRID_WIDTH;
}

double GroundTruth::get_x_from_index(const int index)
{
    return (get_x_index_from_index(index) - GRID_WIDTH_2) * RESOLUTION;
}

double GroundTruth::get_y_from_index(const int index)
{
    return (get_y_index_from_index(index) - GRID_WIDTH_2) * RESOLUTION;
}

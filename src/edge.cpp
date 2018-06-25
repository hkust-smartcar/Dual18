/*
 * edge.cpp
 *
 *  Created on: Feb 23, 2018
 *      Author: morristseng
 */
#include "edge.h"
#include "useful_functions.h"
inline bool ret_cam_bit(int x, int y, const Byte* camBuffer) {
    return ((camBuffer[y * 10 + x / 8] >> (7 - (x % 8))) & 1);//return 1 if black
}

bool check_if_fail(int topline, int bottomline, vector<pair<int,int>> intput_vector){
	bool fail = false;

	std::vector<float> temp;
	temp = linear_regression(intput_vector);
	float variation=0;
	for(int i=0; i<intput_vector.size(); i++){
		float desire_x = 1.0*(intput_vector[i].second-temp[0])/temp[1];
		variation += (abs(intput_vector[i].first - (int)desire_x))^2;
	}
	variation = variation/intput_vector.size();

	if((variation>7)){
		return fail;
	}

	return !fail;

}


bool check_left_edge(int topline, int bottomline,const Byte* camBuffer, vector<pair<int,int>> &edge_coord){
	int top_line = topline;//20
	bool fail = true;
	int bottom_line = bottomline;//60
	bool stop = false;
	int amount=0;
	for(int i=bottom_line-2; i>top_line-10; i--){
		for(int p=2; p<50; p++){
			amount += ret_cam_bit(p,i,camBuffer);
		}
	}
	if(amount>3){
		for(int i=bottom_line-2; i>top_line; i--){
			for(int p=79; p>2; p--){
				if(ret_cam_bit(79,i,camBuffer)==0){
					if((ret_cam_bit(p,i,camBuffer) != ret_cam_bit(p-1,i,camBuffer))&&(ret_cam_bit(p,i,camBuffer)==0)){
						edge_coord.emplace_back(std::make_pair(p,i));
						if(p>77){
							stop = true;
						}
						break;
					}
				}
				else
					break;
			}
			if(stop==true){
				break;
			}
		}
	}

	if(edge_coord.size()<3){
			int num_black=0;
			for(int i=60; i<80;i++){
				for(int j=56; j<60; j++){
					num_black += ret_cam_bit(i,j,camBuffer);
				}
			}
			if(num_black<2){
				return false;
			}
			else{
				return true;
			}
		}


	std::vector<float> temp;
	temp = linear_regression(edge_coord);
	float variation=0;
	for(int i=0; i<edge_coord.size(); i++){
		float desire_x = 1.0*(edge_coord[i].second-temp[0])/temp[1];
		variation += (abs(edge_coord[i].first - (int)desire_x))^2;
	}
	variation = variation/edge_coord.size();

	if((variation>7)){
		return fail;
	}

//
	return !fail;
}

bool check_right_edge(int topline, int bottomline, const Byte* camBuffer, vector<pair<int,int>> &edge_coord){
	int top_line = topline;//20
	bool fail = true;
	int bottom_line = bottomline;//60
	edge_coord.clear();
	bool stop = false;
	int amount = 0;
	for(int i=bottom_line-2; i>top_line-10; i--){
		for(int p=30; p<78; p++){
			amount += ret_cam_bit(p,i,camBuffer);
		}
	}
	if(amount>3){
		for(int i=bottom_line-2; i>top_line; i--){
			for(int p=1; p<78; p++){
				if(ret_cam_bit(0,i,camBuffer)==0){
					if((ret_cam_bit(p,i,camBuffer) != ret_cam_bit(p-1,i,camBuffer))&&(ret_cam_bit(p,i,camBuffer)==1)){
						edge_coord.push_back(make_pair(p,i));
						if(p<2){
							stop = true;
						}
						break;
					}
				}
				else
					break;
			}
			if(stop==true){
				break;
			}
		}
	}

	if(edge_coord.size()<3){
		int num_black=0;
		for(int i=0; i<3;i++){
			for(int j=56; j<60; j++){
				num_black += ret_cam_bit(i,j,camBuffer);
			}
		}
		if(num_black<2){
			return false;
		}
		else{
			return true;
		}
	}

	std::vector<float> temp;
	temp = linear_regression(edge_coord);
	float variation=0;
	for(int i=0; i<edge_coord.size(); i++){
		float desire_x = 1.0*(edge_coord[i].second-temp[0])/temp[1];
		variation += (abs(edge_coord[i].first - (int)desire_x))^2;
	}
	variation = variation/edge_coord.size();

	if((variation>7)){
		return fail;
	}



	return !fail;

}

vector<pair<int,int>> Edge::check_left_edge(const Byte* camBuffer){
	vector<pair<int,int>> edge;
	int black_sum =0;
	for(int j=55; j<60; j++){
		for(int i=0; i<80; i++){
			black_sum = ret_cam_bit(i,j,camBuffer);
		}
	}

	if(black_sum<5){ //on turn
		int init_x = 0;
		for(int j=58; j>40; j--){
			if(ret_cam_bit(78,j,camBuffer)==0){
				for(int i=78; i>0; i--){
					if(ret_cam_bit(i,j,camBuffer) != ret_cam_bit(i-1,j,camBuffer)){
						edge.push_back(make_pair(i,j));
						break;
					}
				}
			}
			else{
				break;
			}
		}
	}
	else{

	}

	return edge;
}

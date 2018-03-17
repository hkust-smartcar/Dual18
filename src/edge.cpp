/*
 * edge.cpp
 *
 *  Created on: Feb 23, 2018
 *      Author: morristseng
 */
#include "edge.h"
inline bool ret_cam_bit(int x, int y, const Byte* camBuffer) {
    return ((camBuffer[y * 10 + x / 8] >> (7 - (x % 8))) & 1);//return 1 if black
}

vector<pair<int,int>>check_edge(const Byte* camBuffer){
	vector<std::pair<int,int>> edge_coord{};;
	int top_line = 30;
	int bottom_line = 60;//height==60
	bool stop = false;
	for(int i=bottom_line-2; i>top_line; i--){
		for(int p=78; p>2; p--){
			if((ret_cam_bit(p,i,camBuffer) != ret_cam_bit(p-1,i,camBuffer))&&(ret_cam_bit(p,i,camBuffer)==0)){
				edge_coord.push_back(std::make_pair(p,i));
				if(p>=76){
					stop = true;
				}
				break;
			}
			else if (ret_cam_bit(p,i,camBuffer) != (ret_cam_bit(p,i-1,camBuffer))&&(ret_cam_bit(p,i,camBuffer)==0)){
				edge_coord.push_back(std::make_pair(p,i));
				if(p>=76){
					stop = true;
				}
				break;
			}
		}
		if(stop==true){
			break;
		}
	}
	return edge_coord;
}

void check_left_edge(const Byte* camBuffer, vector<pair<int,int>> &edge_coord){
	int top_line = 40;
	int bottom_line = 60;//height==60
	bool stop = false;
	for(int i=bottom_line-2; i>top_line; i--){
		for(int p=78; p>2; p--){
			if((ret_cam_bit(p,i,camBuffer) != ret_cam_bit(p-1,i,camBuffer))&&(ret_cam_bit(p,i,camBuffer)==0)){
				edge_coord.push_back(std::make_pair(p,i));
				if(p>=76){
					stop = true;
				}
				break;
			}
		}
		if(stop==true){
			break;
		}
	}
}

void check_right_edge(const Byte* camBuffer, vector<pair<int,int>> &edge_coord){
	int top_line = 40;
	int bottom_line = 60;//height==60
	edge_coord.clear();
	bool stop = false;
	for(int i=bottom_line-2; i>top_line; i--){
		for(int p=2; p<78; p++){
			if((ret_cam_bit(p,i,camBuffer) != ret_cam_bit(p-1,i,camBuffer))&&(ret_cam_bit(p,i,camBuffer)==1)){
				edge_coord.push_back(std::make_pair(p,i));
				if(p<4){
					stop = true;
				}
				break;
			}
		}
		if(stop==true){
			break;
		}
	}
}

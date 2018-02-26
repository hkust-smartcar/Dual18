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
	for(int i=top_line; i<bottom_line-2; i++){
		for(int p=78; p>2; p--){
			if((ret_cam_bit(p,i,camBuffer) != ret_cam_bit(p-1,i,camBuffer))&&(ret_cam_bit(p,i,camBuffer)==0)){
				edge_coord.push_back(std::make_pair(p,i));
				break;
			}
			else if (ret_cam_bit(p,i,camBuffer) != (ret_cam_bit(p,i-1,camBuffer))&&(ret_cam_bit(p,i,camBuffer)==0)){
				edge_coord.push_back(std::make_pair(p,i));
				break;
			}
		}
//		for(int j=60; j<78; j++){//width==80
//			if(ret_cam_bit(j,i,camBuffer) != ret_cam_bit(j-1,i,camBuffer)){
//				edge_coord.push_back(std::make_pair(j,i));
//				break;
//			}
//			else if (ret_cam_bit(j,i,camBuffer) != ret_cam_bit(j,i-1,camBuffer)){
//				edge_coord.push_back(std::make_pair(j,i));
//				break;
//			}
//		}
	}
	return edge_coord;
}


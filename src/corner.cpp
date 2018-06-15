/*
 * corner.cpp
 *
 *  Created on: Feb 21, 2018
 *      Author: morristseng
 */

#include "corner.h"
#include <vector>
using namespace libsc;
using namespace libbase::k60;
using namespace std;

inline bool ret_cam_bit(int x, int y, const Byte* camBuffer) {
    return ((camBuffer[y * 10 + x / 8] >> (7 - (x % 8))) & 1);//return 1 if black
}

vector<std::pair<int,int>> check_corner(const Byte* camBuffer){
	vector<std::pair<int,int>> x_edge_coord{};
	vector<std::pair<int,int>> y_edge_coord{};
	int cornerc = 0; // corner count
	vector< std::pair<int, int> > cornerv{};
	std::vector<float> percentage;
	int top_line = 40;
	int bottom_line = 60;//height==60
	for(int i=top_line; i<bottom_line-2; i++){
		for(int j=2; j<78; j++){//width==80
			if(ret_cam_bit(j,i,camBuffer) != ret_cam_bit(j-1,i,camBuffer)){
				x_edge_coord.push_back(std::make_pair(j,i));
			}
			else if (ret_cam_bit(j,i,camBuffer) != ret_cam_bit(j,i-1,camBuffer)){
				y_edge_coord.push_back(std::make_pair(j,i));
			}
		}
	}

	int du = 2;
	int dv = 2;
	for (int i =0; i<x_edge_coord.size(); i++){
		float percent = 0;
		if((x_edge_coord[i].second>=dv)&&(x_edge_coord[i].first>=du)&&(x_edge_coord[i].first<80-du)&&(x_edge_coord[i].second<20-dv)){//width = 80, height = 20
			for(int j= x_edge_coord[i].second-dv; j<=x_edge_coord[i].second+dv; j++){
				for(int k= x_edge_coord[i].first-du; k<=x_edge_coord[i].first+du; k++){
					percent += (ret_cam_bit(j,k,camBuffer));
				}
			}
		}
		percent = percent/25;
		if ((percent<0.2)){
			cornerv.push_back(std::make_pair(x_edge_coord[i].first,x_edge_coord[i].second));
			percentage.push_back(percent);
		}
	}

	for (int i =0; i<y_edge_coord.size(); i++){
		float percent = 0;
		if((y_edge_coord[i].second>=dv)&&(y_edge_coord[i].first>=du)&&(y_edge_coord[i].first<80-du)&&(y_edge_coord[i].second<60-dv)){
			for(int j= y_edge_coord[i].second-dv; j<=y_edge_coord[i].second+dv; j++){
				for(int k= y_edge_coord[i].first-du; k<=y_edge_coord[i].first+du; k++){
					percent += (ret_cam_bit(j,k,camBuffer));
				}
			}
		}
		percent = percent/25;
		if ((percent<0.2)){
			cornerv.push_back(std::make_pair(y_edge_coord[i].first,y_edge_coord[i].second));
			percentage.push_back(percent);
		}
	}

		//count = 0;
		int k =percentage.size();
		for (int i=0; i<k;){
			int flag = 0;
			for (int p=i+1; p<k; ){
				if((cornerv[i].first-cornerv[p].first<=5)&&(cornerv[i].first-cornerv[p].first>=-5)&&(cornerv[i].second-cornerv[p].second<=5)&&(cornerv[i].second-cornerv[p].second>=-5)){
					if(percentage[i] >= percentage[p]){
						percentage.erase(percentage.begin()+p);
						cornerv.erase(cornerv.begin()+p);
						k--;
					}
					else{
						percentage.erase(percentage.begin()+i);
						cornerv.erase(cornerv.begin()+i);
						k--;
						flag = 1;
						break;
					}
				}
				else{
					p++;
				}
			}
			if (flag == 0){
				i++;
			}
		}
	return cornerv;
}










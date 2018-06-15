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

vector<pair<int,int>>check_corner_edge(const Byte* camBuffer, int topline, int bottomline, bool type){//for type true == master, false == slave
	vector<pair<int,int>> m_edge;

	if(!type){//slave
		for(int j = topline; j<bottomline; j++){//scan from left
			for(int i=1; i<78; i++){
				if((ret_cam_bit(i,j,camBuffer)!=ret_cam_bit(i-1,j,camBuffer))&&(ret_cam_bit(i,j,camBuffer)==0)){
					m_edge.push_back(make_pair(i,j));
					break;
				}
			}
		}
		for(int j = bottomline; j>topline; j--){// scan from bottom
			for(int i=1; i<78; i++){
				if((ret_cam_bit(i,j,camBuffer)!=ret_cam_bit(i,j-1,camBuffer))&&(ret_cam_bit(i,j,camBuffer)==0)){
					bool repeat = false;
					for(int k=0; k<m_edge.size();k++){
						if((i==m_edge[k].second)&&(j==m_edge[k].second)){
							repeat = true;
							break;
						}
					}
					if(repeat == false){
						m_edge.push_back(make_pair(i,j));
					}
					break;
				}
			}
		}
	}

	else{
		for(int j = topline; j<bottomline; j++){//scan from right
			for(int i=78; i>1; i--){
				if((ret_cam_bit(i,j,camBuffer)!=ret_cam_bit(i-1,j,camBuffer))&&(ret_cam_bit(i,j,camBuffer)==0)){
					m_edge.push_back(make_pair(i,j));
					break;
				}
			}
		}
		for(int j = bottomline; j>topline; j--){// scan from bottom
			for(int i=1; i<78; i++){
				if((ret_cam_bit(i,j,camBuffer)!=ret_cam_bit(i,j-1,camBuffer))&&(ret_cam_bit(i,j,camBuffer)==0)){
					bool repeat = false;
					for(int k=0; k<m_edge.size();k++){
						if((i==m_edge[k].second)&&(j==m_edge[k].second)){
							repeat = true;
							break;
						}
					}
					if(repeat == false){
						m_edge.push_back(make_pair(i,j));
					}
					break;
				}
			}
		}
	}

	return m_edge;
}

vector<Corner> check_corner(const Byte* camBuffer, int topline, int bottomline, bool type){
	vector<std::pair<int,int>> x_edge_coord{};
	vector<std::pair<int,int>> y_edge_coord{};
	vector<std::pair<int,int>> edge{};
	int cornerc = 0; // corner count
	vector< std::pair<int, int> > cornerv{};
	vector<Corner> m_corner;

	std::vector<float> percentage;
	int top_line = topline;
	int bottom_line = bottomline;//height==60
//	for(int i=top_line; i<bottom_line-2; i++){
//		for(int j=2; j<78; j++){//width==80
//			if(ret_cam_bit(j,i,camBuffer) != ret_cam_bit(j-1,i,camBuffer)){
//				x_edge_coord.push_back(std::make_pair(j,i));
//			}
//			else if (ret_cam_bit(j,i,camBuffer) != ret_cam_bit(j,i-1,camBuffer)){
//				y_edge_coord.push_back(std::make_pair(j,i));
//			}
//		}
//	}

	edge = check_corner_edge(camBuffer, topline, bottomline, type);


	int du = 2;
	int dv = 2;
	for (int i =0; i<edge.size(); i++){
		float percent = 0;
		if((edge[i].second>=top_line+dv)&&(edge[i].first>=du)&&(edge[i].first<80-du)&&(edge[i].second<bottom_line-dv)){
			for(int j= edge[i].second-dv; j<=edge[i].second+dv; j++){
				for(int k= edge[i].first-du; k<=edge[i].first+du; k++){
					percent += (ret_cam_bit(k,j,camBuffer));
				}
			}
			percent = percent/25.0;
			if ((percent<0.28)&&(percent!=0)){
				Corner temp(edge[i].first,edge[i].second, percent);
				m_corner.push_back(temp);
			}
		}
	}
//	for (int i =0; i<x_edge_coord.size(); i++){
//		float percent = 0;
//		if((x_edge_coord[i].second>=top_line+dv)&&(x_edge_coord[i].first>=du)&&(x_edge_coord[i].first<80-du)&&(x_edge_coord[i].second<bottom_line-dv)){//width = 80, height = 20
//			for(int j= x_edge_coord[i].second-dv; j<=x_edge_coord[i].second+dv; j++){
//				for(int k= x_edge_coord[i].first-du; k<=x_edge_coord[i].first+du; k++){
//					percent += (ret_cam_bit(k,j,camBuffer));
//				}
//			}
//			percent = percent/25.0;
//			if ((percent<0.28)&&(percent!=0)){
//				Corner temp(x_edge_coord[i].first,x_edge_coord[i].second, percent);
//				m_corner.push_back(temp);
//			}
//		}
//	}
//
//	for (int i =0; i<y_edge_coord.size(); i++){
//		float percent = 0;
//		if((y_edge_coord[i].second>=top_line+dv)&&(y_edge_coord[i].first>=du)&&(y_edge_coord[i].first<80-du)&&(y_edge_coord[i].second<bottom_line-dv)){
//			for(int j= y_edge_coord[i].second-dv; j<=y_edge_coord[i].second+dv; j++){
//				for(int k= y_edge_coord[i].first-du; k<=y_edge_coord[i].first+du; k++){
//					percent += (ret_cam_bit(k,j,camBuffer));
//				}
//			}
//			percent = percent/25.0;
//			if ((percent<0.28)&&(percent!=0)){
//				Corner temp(x_edge_coord[i].first,x_edge_coord[i].second, percent);
//				m_corner.push_back(temp);
//			}
//		}
//	}

		//count = 0;

		int k = m_corner.size();




		for (int i=0; i<k;){
			int flag = 0;
			for (int p=i+1; p<k; ){
				if((m_corner[i].get_xcoord()-m_corner[p].get_xcoord()<=5)&&(m_corner[i].get_xcoord()-m_corner[p].get_xcoord()>=-5)&&(m_corner[i].get_ycoord()-m_corner[p].get_ycoord()<=5)&&(m_corner[i].get_ycoord()-m_corner[p].get_ycoord()>=-5)){
					if(m_corner[i].get_percentage() >= m_corner[p].get_percentage()){
						m_corner.erase(m_corner.begin()+p);
						k--;
					}
					else{
						m_corner.erase(m_corner.begin()+i);
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
	return m_corner;
}

Corner find_min(vector<Corner> m_corner){
	Corner min_percentage(0,0,0);

	if(m_corner.size()==0){
		return min_percentage;
	}
	else if(m_corner.size()==1){
		return m_corner[0];
	}
	else{
		min_percentage = m_corner[0];
		for(int i=1; i<m_corner.size(); i++){
			if(m_corner[i].get_percentage() < min_percentage.get_percentage()){
				min_percentage = m_corner[i];
			}
		}
	}
	return min_percentage;
}











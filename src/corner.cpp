/*
 * corner.cpp
 *
 *  Created on: Feb 21, 2018
 *      Author: morristseng
 */

#include "corner.h"
#include "edge.h"
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
		for(int j = bottomline; j>topline; j--){//scan from left
			for(int i=0; i<78; i++){
				if(ret_cam_bit(0,j,camBuffer)==0){
					if((ret_cam_bit(i,j,camBuffer)!=ret_cam_bit(i+1,j,camBuffer))&&(ret_cam_bit(i,j,camBuffer)==0)){
						m_edge.push_back(make_pair(i,j));
						break;
					}
				}
				else
					break;
			}
		}
		int search_from = 0;
		if(m_edge.size()>0)
			search_from = m_edge[m_edge.size()-1].first;
		for(int i=1; i<=search_from; i++){// scan from bottom
			for(int j = bottomline; j>topline; j--){
				if(ret_cam_bit(i,bottomline,camBuffer)==0){
					if((ret_cam_bit(i,j,camBuffer)!=ret_cam_bit(i,j-1,camBuffer))&&(ret_cam_bit(i,j,camBuffer)==0)){
						bool repeat = false;
						for(int k=0; k<m_edge.size();k++){
							if((i==m_edge[k].first)&&(j==m_edge[k].second)){
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
				else
					break;
			}
		}
	}

	else{
		for(int j = bottomline; j>topline; j--){//scan from right
			for(int i=79; i>1; i--){
				if(ret_cam_bit(79,j,camBuffer)==0){
					if((ret_cam_bit(i,j,camBuffer)!=ret_cam_bit(i-1,j,camBuffer))&&(ret_cam_bit(i,j,camBuffer)==0)){
						m_edge.push_back(make_pair(i,j));
						break;
					}
				}
				else
					break;
			}
		}
		int search_from = 0;
		if(m_edge.size()>0)
			search_from = m_edge[m_edge.size()-1].first;
		for(int i=78; i>=search_from; i--){// scan from bottom
			for(int j = bottomline; j>topline; j--){
				if(ret_cam_bit(i,bottomline,camBuffer)==0){
					if((ret_cam_bit(i,j,camBuffer)!=ret_cam_bit(i,j-1,camBuffer))&&(ret_cam_bit(i,j,camBuffer)==0)){
						bool repeat = false;
						for(int k=0; k<m_edge.size();k++){
							if((i==m_edge[k].first)&&(j==m_edge[k].second)){
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
				else
					break;
			}
		}
	}

	return m_edge;
}

vector<Corner> check_corner(const Byte* camBuffer, int topline, int bottomline, bool type){
	vector<std::pair<int,int>> edge{};
	vector<Corner> m_corner;

	std::vector<float> percentage;
	int top_line = topline;
	int bottom_line = bottomline;//height==60

	edge = check_corner_edge(camBuffer, topline, bottomline, type);
//
//	for(int i=0; i<edge.size(); i++){
//		Corner temp(edge[i].first, edge[i].second, 0);
//		m_corner.push_back(temp);
//	}

	int du = 3;
	int dv = 3;
	for (int i =0; i<edge.size(); i++){
		float percent = 0;
		if((edge[i].second>=top_line+dv)&&(edge[i].first>=du)&&(edge[i].first<80-du)&&(edge[i].second<bottom_line-dv)){
			for(int j= edge[i].second-dv; j<=edge[i].second+dv; j++){
				for(int k= edge[i].first-du; k<=edge[i].first+du; k++){
					percent += (ret_cam_bit(k,j,camBuffer));
				}
			}
			percent = percent/49.0;
			if ((percent<0.25)&&(percent>0.1)){
				Corner temp(edge[i].first,edge[i].second, percent);
				m_corner.push_back(temp);
			}
		}
	}


		int k = m_corner.size();




		for (int i=0; i<k;){
			int flag = 0;
			for (int p=i+1; p<k; ){
				if((m_corner[i].get_xcoord()-m_corner[p].get_xcoord()<=5)&&(m_corner[i].get_xcoord()-m_corner[p].get_xcoord()>=-5)&&(m_corner[i].get_ycoord()-m_corner[p].get_ycoord()<=5)&&(m_corner[i].get_ycoord()-m_corner[p].get_ycoord()>=-5)){
					if(m_corner[i].get_percentage() > m_corner[p].get_percentage()){
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










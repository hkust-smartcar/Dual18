/*
 * corner.cpp
 *
 *  Created on: Feb 21, 2018
 *      Author: morristseng
 */

#include "corner.h"
#include "edge.h"
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
		bool found = false;
		bool quit = false;
		for(int i=1; i<=78; i++){// scan from bottom
			for(int j = bottomline-1; j>topline; j--){
				if(ret_cam_bit(i,bottomline-1,camBuffer)==0){
					if((ret_cam_bit(i,j,camBuffer)!=ret_cam_bit(i,j-1,camBuffer))&&(ret_cam_bit(i,j,camBuffer)==0)){
						found = true;
						m_edge.push_back(make_pair(i,j));
						if(j>=77){
							quit = true;
						}

						break;
					}
					else{
						found = false;
					}
				}
				else{
					quit = true;
					break;
				}
				if(found)
					break;
			}
			if(quit)
				break;
		}
	}

	else{
		bool found = false;
		bool quit = false;
		for(int i=78; i>=1; i--){// scan from bottom
			for(int j = bottomline-1; j>topline; j--){
				if(ret_cam_bit(i,bottomline-1,camBuffer)==0){
					if((ret_cam_bit(i,j,camBuffer)!=ret_cam_bit(i,j-1,camBuffer))&&(ret_cam_bit(i,j,camBuffer)==0)){
						found = true;
						m_edge.push_back(make_pair(i,j));
						if(j>=77){
							quit = true;
						}

						break;
					}
					else{
						found = false;
					}
				}
				else{
					quit = true;
					break;
				}
				if(found)
					break;
			}
			if(quit)
				break;
		}
	}

	return m_edge;
}

vector<Corner> check_corner(const Byte* camBuffer, int topline, int bottomline, bool type, vector<pair<int,int>> &edge){
	vector<Corner> m_corner;

	std::vector<float> percentage;
	int top_line = topline;
	int bottom_line = bottomline;//height==60
	bool left_fail = false;
	bool right_fail = false;

	edge = check_corner_edge(camBuffer, topline, bottomline, type);

	if(type==true){
		left_fail = check_if_fail(topline, bottomline, edge);
		if(!left_fail){
			edge.clear();
			return m_corner;
		}
	}
	else{
		right_fail = check_if_fail(topline, bottomline, edge);
		if(!right_fail){
			edge.clear();
			return m_corner;
		}
	}


//		for(int i=0; i<edge.size(); i++){
//			Corner temp(edge[i].first, edge[i].second, 0);
//			m_corner.push_back(temp);
//		}

	int du = 4;
	int dv = 4;
	for (int i =0; i<edge.size(); i++){
		float percent = 0;

		if((edge[i].second>=(top_line+dv))&&(edge[i].first>=du)&&(edge[i].first<(80-du))&&(edge[i].second<(bottom_line-dv))){
			if(ret_cam_bit(edge[i].first-1,edge[i].second,camBuffer)==0&&ret_cam_bit(edge[i].first+1,edge[i].second,camBuffer)==0&&ret_cam_bit(edge[i].first,edge[i].second-1,camBuffer)==0&&ret_cam_bit(edge[i].first,edge[i].second+1,camBuffer)==0){
				continue;
			}
			for(int j= edge[i].second-dv; j<=edge[i].second+dv; j++){
				for(int k= edge[i].first-du; k<=edge[i].first+du; k++){
					percent += (ret_cam_bit(k,j,camBuffer));
				}
			}
			percent = percent/81.0;
			if ((percent<0.24)&&(percent>0.15)){
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
//		if(m_corner.size()>0){
//			int is_breakpoint;
//			is_breakpoint =1;
//		}
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










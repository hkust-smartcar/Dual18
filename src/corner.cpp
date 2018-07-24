/*
 * corner.cpp
 *
 *  Created on: Feb 21, 2018
 *      Author: morristseng
 */

#include "corner.h"
#include "edge.h"
#include <vector>
#include "useful_functions.h"
using namespace libsc;
using namespace libbase::k60;
using namespace std;

inline bool ret_cam_bit(int x, int y, const Byte* camBuffer) {
    return ((camBuffer[y * 10 + x / 8] >> (7 - (x % 8))) & 1);//return 1 if black
}


//uint8_t convolution(uint8_t xcoord, uint8_t ycoord, const Byte* camBuffer){
//	uint8_t threshold = 0;
//	uint8_t value = 0;
//	int g1[3][3] = {{-1,0,1},
//				{-2,0,2},
//				{-1,0,1}};
//	int g2[3][3] = {{-1,-2,-1},
//				  {0,0,0},
//				  {1,2,1}};
//	int Q1 = 0;
//	int Q2 = 0;
//	for(int j=0; j<3; j++){
//		for(int i=0; i<3; i++){
//			Q1 += (ret_cam_bit(xcoord+(i-1), ycoord+(j-1), camBuffer))*(g1[i][j]);
//			Q2 += ret_cam_bit(xcoord+(i-1), ycoord+(j-1), camBuffer)*(g2[i][j]);
//
//		}
//	}
//	if(Q1<0)
//		Q1 = -Q1;
//	if(Q2<0)
//		Q2 = -Q2;
//
//	return (Q1+Q2);
//}


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

vector<Corner> check_corner(const Byte* camBuffer, int topline, int bottomline, vector<pair<int,int>> edge){
	vector<Corner> m_corner;

	std::vector<float> percentage;
	int top_line = topline;
	int bottom_line = bottomline;//height==60

	if(edge.size()<5){
		return m_corner;
	}
//	bool left_fail = false;
//	bool right_fail = false;
//	if(type==true){
//		left_fail = check_if_fail(topline, bottomline, edge);
//		if(!left_fail){
//			edge.clear();
//			return m_corner;
//		}
//	}
//	else{
//		right_fail = check_if_fail(topline, bottomline, edge);
//		if(!right_fail){
//			edge.clear();
//			return m_corner;
//		}
//	}


//		for(int i=0; i<edge.size(); i++){
//			Corner temp(edge[i].first, edge[i].second, 0);
//			m_corner.push_back(temp);
//		}

	int du = 3;
	int dv = 3;
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
			percent = percent/49.0;
			if (((percent<0.25)&&(percent>0.1))||((percent>0.75)&&(percent<0.85))){
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

float slope(uint8_t originx, uint8_t originy, uint8_t x2, uint8_t y2){
	float slope = 0;
	int dx = 1;
	int dy = 0;

	dy = y2-originy;
	dx = x2-originx;

	if(dx==0){
		return 100;
	}

	slope = (dy*(1.0)/dx);

	return slope;
}


vector<Corner> check_cornerv2(const Byte* camBuffer, int topline, int bottomline, vector<pair<int,int>> edge){
	vector<Corner> m_corner;
	if(edge.size()<=6){
		return m_corner;
	}
	for(int i=3; i<edge.size()-3; i++){
		float distance1 = 0;
		float distance2 = 0;
		float distance3 = 0;
		float cos = 0;
		distance1 = distance(edge[i].first, edge[i].second, (edge[i-3].first+edge[i-2].first+edge[i-1].first)/3, (edge[i-3].second+edge[i-2].second+edge[i-1].second)/3);
		distance2 = distance(edge[i].first, edge[i].second, (edge[i+3].first+edge[i+2].first+edge[i+1].first)/3, (edge[i+3].second+edge[i+2].second+edge[i+1].second)/3);
		distance3 = distance((edge[i-3].first+edge[i-2].first+edge[i-1].first)/3, (edge[i-3].second+edge[i-2].second+edge[i-1].second)/3, (edge[i+3].first+edge[i+2].first+edge[i+1].first)/3, (edge[i+3].second+edge[i+2].second+edge[i+1].second)/3);

		if(distance1==0||distance2==0){
			continue;
		}

		cos = ((distance1*distance1)+(distance2*distance2)-(distance3*distance3))/(2*((distance1*distance2)));

		if(cos>-0.5){
			Corner temp(edge[i].first,edge[i].second, cos);
			m_corner.push_back(temp);
		}
	}

	if(edge[0].first>3&& edge[0].first<76 && edge[0].second<58){
		int check = 0;
		for(int j = 0; j<2; j++){
			for(int i=-3; i<4; i++){
				check += ret_cam_bit((edge[0].first)+i,edge[0].second+j, camBuffer);
			}
		}
		if(check<3){
			Corner temp(edge[0].first,edge[0].second, 0);
			m_corner.push_back(temp);
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







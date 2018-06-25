/*
 * useful_functions.cpp
 *
 *  Created on: Apr 12, 2018
 *      Author: morristseng
 */
#include "useful_functions.h"
#include <array>
#include <cmath>

inline bool ret_cam_bit(int x, int y, const Byte* camBuffer) {
    return ((camBuffer[y * 10 + x / 8] >> (7 - (x % 8))) & 1);//return 1 if black
}


std::vector<float> linear_regression(std::vector<std::pair<int,int>> m_vector){
	float avg_x=0;
	float avg_y=0;
	float b1_numerator=0;
	float b1_denominator=0;
	float b0;
	float b1;//slope
	std::vector<float> b0_b1_avgx_avgy;
	for(int i=0; i<m_vector.size(); i++){
		avg_x += m_vector[i].first;
		avg_y += m_vector[i].second;
	}
	avg_x /= (m_vector.size())*1.0;
	avg_y /= (m_vector.size())*1.0;
	for(int i=0; i<m_vector.size(); i++){
		b1_numerator += ((m_vector[i].first-avg_x)*(m_vector[i].second-avg_y)*1.0);
		b1_denominator += (((m_vector[i].first-avg_x)*(m_vector[i].first-avg_x))*1.0);
	}
	b1 = b1_numerator/b1_denominator;
	b0 = avg_y - b1*avg_x;
	b0_b1_avgx_avgy.push_back(b0);
	b0_b1_avgx_avgy.push_back(b1);
	b0_b1_avgx_avgy.push_back(avg_x);
	b0_b1_avgx_avgy.push_back(avg_y);

	return b0_b1_avgx_avgy;
}

double find_slope(std::vector<std::pair<int,int>> m_vector){
	bool min_find = false;
	bool max_find = false;
	std::vector<std::pair<int,int>> min_xy;
	std::vector<std::pair<int,int>> max_xy;
	double slope;

	if(m_vector.size()==0){
		return 0;
	}

	for(int i=(m_vector.size()/2); i<m_vector.size(); i++){

		if((m_vector[i].first<=78)&&(min_find == false)){
			min_xy.push_back(std::make_pair(m_vector[i].first, m_vector[i].second));
			min_find = true;
		}
		if((m_vector[i].first<=78)){
			if(i==(m_vector.size()/2)){
				max_xy.push_back(std::make_pair(m_vector[0].first, m_vector[0].second));
			}
			else if(m_vector[i].second>=m_vector[i-1].second){
				max_xy.erase(max_xy.begin());
				max_xy.push_back(std::make_pair(m_vector[i].first, m_vector[i].second));
			}
			max_find = true;
		}
	}
	if(min_find==false){
		min_xy.push_back(std::make_pair(0,0));
	}
	if(max_find==false){
		max_xy.push_back(std::make_pair(0,0));
	}
	if((min_find==true)&&(max_find==true)){
		if(min_xy[0].second == max_xy[0].second){
			slope = 0.0;
		}
		else{
			slope = ((max_xy[0].second - min_xy[0].second)/(1.0*min_xy[0].first - max_xy[0].first));
		}
	}
	return slope;
}

std::vector<std::pair<int,int>> harris_corner_detection(const Byte* camBuffer){
	std::vector<std::pair<int,int>> corner;
	int width = 80;
	int height = 40;
	int A[height][width];
    float Ixfilter[height][width];
    float Iyfilter[height][width];
    float Ix2filter[height][width];
    float Iy2filter[height][width];
    float Ixyfilter[height][width];
    float R_val;
    std::vector<float> R;
    std::vector<float> R_value;
    std::vector<std::pair<int,int>> Coord;

	for(int i=0; i<height; i++){
		for(int j=0; j<width; j++){
            float dGx[3] = {-0.606531, 0, 0.606531};
            float dGy[3] = {-0.606531, 0, 0.606531};
            int du = 2; //dx
            int dv = 2; //dy

            for (int y=dv; y<height-dv; y++){
				for (int x=du; x<width-du; x++){
					Ixfilter[y][x] = (A[y][x-1]*dGx[0]+A[y][x+1]*dGx[2]);
					Iyfilter[y][x] = (A[y-1][x]*dGy[0]+A[y+1][x]*dGy[2]);
					Ix2filter[y][x] = Ixfilter[y][x]*Ixfilter[y][x];
					Iy2filter[y][x] = Iyfilter[y][x]*Iyfilter[y][x];
					Ixyfilter[y][x] = Ixfilter[y][x]*Iyfilter[y][x];
				}
			}

			float M[2][2] = {0,0,0,0};


			for (int y=dv; y<height-dv; y++){
				for (int x=du; x<width-du; x++){
					for(int u=y-dv; u<=y+dv; u++){
						for(int i=x-du; i<=x+du; i++){
							M[0][0] += Ix2filter[u][i];
							M[0][1] += Ixyfilter[u][i];
							M[1][0] += Ixyfilter[u][i];
							M[1][1] += Iy2filter[u][i];
						}
					}
					R_val = ((M[0][0]*M[1][1])-(M[0][1]*M[1][0]))-(0.05)*pow((M[0][0]+M[1][1]),2);
					R.push_back(R_val);
					Coord.push_back(std::make_pair(x,y));
					for (int i=0; i<2; i++){
						for (int u=0; u<2; u++){
							M[i][u] = 0;
						}
					}
				}
			}
		}
	}
	float threshold = 3;
	for (int i = 0; i<R.size(); i++ ){
		if (R[i] > threshold){
			R_value.push_back(R[i]);
			corner.push_back(Coord[i]);
		}
	}
	R.clear();
	Coord.clear();

	int k = R_value.size();
	for (int i=0; i<k;){
		int flag = 0;
		for (int p=i+1; p<k; ){
			if((corner[i].first-corner[p].first<=5)&&(corner[i].first-corner[p].first>=-5)&&(corner[i].second-corner[p].second<=5)&&(corner[i].second-corner[p].second>=-5)){
				if(R_value[i] >= R_value[p]){
					R_value.erase(R_value.begin()+p);
					corner.erase(corner.begin()+p);
					k--;
				}
				else{
					R_value.erase(R_value.begin()+i);
					corner.erase(corner.begin()+i);
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

	return corner;
}

std::vector<std::pair<int,int>> susan_corner_detection(const Byte* camBuffer){
	std::vector<std::pair<int,int>> x_edge_coord{};
	std::vector<std::pair<int,int>> y_edge_coord{};
	int cornerc = 0; // corner count
	std::vector< std::pair<int, int> > cornerv{};
	std::vector<float> percentage;
	int top_line = 5;
	int bottom_line = 40;//height==35
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

	int du = 3;
	int dv = 3;
	for (int i =0; i<x_edge_coord.size(); i++){
		float percent = 0;
		if((x_edge_coord[i].second>=dv)&&(x_edge_coord[i].first>=du)&&(x_edge_coord[i].first<80-du)&&(x_edge_coord[i].second<40-dv)){//width = 80, height = 20
			for(int j= x_edge_coord[i].second-dv; j<=x_edge_coord[i].second+dv; j++){
				for(int k= x_edge_coord[i].first-du; k<=x_edge_coord[i].first+du; k++){
					percent += (ret_cam_bit(k,j,camBuffer));
				}
			}
			percent = percent/49.0;
			if ((percent < 0.3)&&(percent>0.1)){
				cornerv.push_back(std::make_pair(x_edge_coord[i].first,x_edge_coord[i].second));
				percentage.push_back(percent);
			}
		}

	}

	for (int i =0; i<y_edge_coord.size(); i++){
		float percent = 0;
		if((y_edge_coord[i].second>dv)&&(y_edge_coord[i].first>du)&&(y_edge_coord[i].first<80-du)&&(y_edge_coord[i].second<40-dv)){
			for(int j= y_edge_coord[i].second-dv; j<=y_edge_coord[i].second+dv; j++){
				for(int k= y_edge_coord[i].first-du; k<=y_edge_coord[i].first+du; k++){
					percent += (ret_cam_bit(k,j,camBuffer));
				}
			}
		}
		percent = percent/49.0;
		if ((percent < 0.3)&&(percent>0.1)){
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


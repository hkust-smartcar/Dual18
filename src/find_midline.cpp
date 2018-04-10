/*
 * find_midline.cpp
 *
 *  Created on: Mar 31, 2018
 *      Author: morristseng
 */


#include "find_midline.h"
#include <vector>

using namespace std;

void find_midline(vector<pair<int,int>>m_master_vector, vector<pair<int,int>> temp, vector<pair<int,int>> &midline){
	if(m_master_vector.size()==temp.size()){
		int k=0;
//					led2.Switch();
		vector<int> index;
		for(int i = m_master_vector[0].second; i>(*m_master_vector.end()).second; i--){
			for(int j=0; j<temp.size(); j++){
				if(temp[j].second==i){
					midline.push_back(make_pair((m_master_vector[k].first+temp[j].first)/2,i));
					index.push_back(j);
					break;
				}
			}
			k++;
		}
	}

	else if(m_master_vector.size()>temp.size()){
		vector<int> index;

		for(int i =0; i<temp.size(); i++){
			for(int j=0; j<m_master_vector.size(); j++){
				if(m_master_vector[j].second==temp[i].second){
					midline.push_back(make_pair((m_master_vector[j].first+temp[i].first)/2,temp[i].second));
					index.push_back(j);
					break;
				}
			}
		}
		for(int i=0; i<m_master_vector.size(); i++){
			bool repeat = false;
			for(int j=0; j<index.size(); j++){
				if((i==index[j])){
					repeat = true;
					break;
				}
			}
			if(!repeat){
				midline.push_back(make_pair((m_master_vector[i].first+78)/2,m_master_vector[i].second));
			}
		}
		index.clear();
	}

	else if(m_master_vector.size() < temp.size()){
		vector<int> index;
		for(int i = 0; i<m_master_vector.size(); i++){
			for(int j=0; j<temp.size(); j++){
				if(temp[j].second==m_master_vector[i].second){
					midline.push_back(make_pair((m_master_vector[i].first+temp[j].first)/2,m_master_vector[i].second));
					index.push_back(j);
					break;
				}
			}
		}

		for(int i=0; i<temp.size(); i++){
			bool repeat = false;
			for(int j=0; j<index.size(); j++){
				if((i==index[j])){
					repeat = true;
					break;
				}
			}
			if(!repeat){
				midline.push_back(make_pair((temp[i].first+0)/2,temp[i].second));
			}
		}
		index.clear();
	}
}

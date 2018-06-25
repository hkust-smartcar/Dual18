/*
 * find_midline.cpp
 *
 *  Created on: Mar 31, 2018
 *      Author: morristseng
 */


#include "find_midline.h"
#include <vector>

using namespace std;

vector<pair<int,int>> find_midline(vector<pair<int,int>>master_edge, vector<pair<int,int>> slave_edge){

	vector<pair<int,int>> midline;

	if(master_edge.size()==slave_edge.size()){
		int k=0;
//					led2.Switch();
		vector<int> index;
		for(int i = master_edge[0].second; i>(*master_edge.end()).second; i--){
			for(int j=0; j<slave_edge.size(); j++){
				if(slave_edge[j].second==i){
					midline.push_back(make_pair((master_edge[k].first+slave_edge[j].first)/2,i));
					index.push_back(j);
					break;
				}
			}
			k++;
		}
	}

	else if(master_edge.size()>slave_edge.size()){
		vector<int> index;

		for(int i =0; i<slave_edge.size(); i++){
			for(int j=0; j<master_edge.size(); j++){
				if(master_edge[j].second==slave_edge[i].second){
					midline.push_back(make_pair((master_edge[j].first+slave_edge[i].first)/2,slave_edge[i].second));
					index.push_back(j);
					break;
				}
			}
		}
		for(int i=0; i<master_edge.size(); i++){
			bool repeat = false;
			for(int j=0; j<index.size(); j++){
				if((i==index[j])){
					repeat = true;
					break;
				}
			}
			if(!repeat){
				midline.push_back(make_pair((master_edge[i].first+78)/2,master_edge[i].second));
			}
		}
		index.clear();
	}

	else if(master_edge.size() < slave_edge.size()){
		vector<int> index;
		for(int i = 0; i<master_edge.size(); i++){
			for(int j=0; j<slave_edge.size(); j++){
				if(slave_edge[j].second==master_edge[i].second){
					midline.push_back(make_pair((master_edge[i].first+slave_edge[j].first)/2,master_edge[i].second));
					index.push_back(j);
					break;
				}
			}
		}

		for(int i=0; i<slave_edge.size(); i++){
			bool repeat = false;
			for(int j=0; j<index.size(); j++){
				if((i==index[j])){
					repeat = true;
					break;
				}
			}
			if(!repeat){
				midline.push_back(make_pair((slave_edge[i].first+0)/2,slave_edge[i].second));
			}
		}
		index.clear();
	}
	return midline;
}

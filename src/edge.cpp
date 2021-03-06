/*
 * edge.cpp
 *
 *  Created on: Feb 23, 2018
 *      Author: morristseng
 */
#include "edge.h"
#include "useful_functions.h"
inline bool ret_cam_bit(int x, int y, const Byte* camBuffer) {
    return ((camBuffer[y * 10 + x / 8] >> (7 - (x % 8))) & 1);//return 1 if black
}

void Edge::traveling_right(uint8_t xcoord, uint8_t ycoord, uint8_t last_direction_from, const Byte* camBuffer){
	uint8_t wrong_way1 = 0;
	uint8_t wrong_way2 = 0;
	uint8_t wrong_way3 = 0;
	uint8_t start_point = 0;
	uint8_t end_point = 0;

	uint8_t junctions = check_junctions(ycoord);

	if(junctions>4){
		fail = true;
		return;
	}

	if((ycoord>bottomline-2)||(ycoord<=topline)||(xcoord>77)||(xcoord<2)){
		return;
	}


	if((last_direction_from==Direction::Up)){//Up = 0
		wrong_way1 = Direction::UpLeft;
		wrong_way2 = Direction::Up;
		wrong_way3 = Direction::UpRight;
	}
	else{
		wrong_way1 = last_direction_from-1;
		wrong_way2 = last_direction_from;
		wrong_way3 = last_direction_from+1;
		if(last_direction_from==Direction::UpLeft){
			wrong_way3 = 0;
		}
	}

	if(last_direction_from<=4){
		start_point = last_direction_from+3;
	}
	else{
		if(last_direction_from==5){
			start_point = Direction::Up;
		}
		else if(last_direction_from==6){
			start_point = Direction::UpRight;
		}
		else if(last_direction_from==7){
			start_point = Direction::Right;
		}
	}

	end_point = start_point -1;
	if(start_point == Direction::Up){
		end_point = Direction::UpLeft;
	}
	for(int i=start_point; ; i++){
		if(i>7){
			i-=8;
		}
		if((i==wrong_way1)||(i==wrong_way2)||(i==wrong_way3)){
			continue;
		}
		else{
			if(i==Direction::Right){
				int tempx = xcoord+1;
				int tempy = ycoord;
				if((ret_cam_bit(tempx, tempy, camBuffer)!=ret_cam_bit(tempx, tempy+1, camBuffer))&&(ret_cam_bit(tempx, tempy,camBuffer)==0)){
					if(!canvas[tempx][tempy-30]){
						canvas[tempx][tempy-30] = true;
						m_edge.push_back(make_pair(tempx,tempy));
						traveling_right(tempx, tempy, Direction::Left, camBuffer);
					}
				}
			}
			else if(i==Direction::UpRight){
				uint8_t tempx = xcoord+1;
				uint8_t tempy = ycoord-1;
				if((ret_cam_bit(tempx, tempy, camBuffer)!=ret_cam_bit(tempx, tempy+1, camBuffer))&&(ret_cam_bit(tempx, tempy,camBuffer)==0)){
					if(!canvas[tempx][tempy-30]){
						canvas[tempx][tempy-30] = true;
						m_edge.push_back(make_pair(tempx,tempy));
						traveling_right(tempx, tempy, Direction::DownLeft, camBuffer);
					}
				}
			}
			else if(i==Direction::Up){
				uint8_t tempx = xcoord;
				uint8_t tempy = ycoord-1;
				if((ret_cam_bit(tempx, tempy, camBuffer)!=ret_cam_bit(tempx+1, tempy, camBuffer))&&(ret_cam_bit(tempx, tempy,camBuffer)==0)){
					if(!canvas[tempx][tempy-30]){
						canvas[tempx][tempy-30] = true;
						m_edge.push_back(make_pair(tempx,tempy));
						traveling_right(tempx, tempy, Direction::Down, camBuffer);
					}
				}
			}
			else if(i==Direction::UpLeft){
				uint8_t tempx = xcoord-1;
				uint8_t tempy = ycoord-1;
				if((ret_cam_bit(tempx, tempy, camBuffer)!=ret_cam_bit(tempx+1, tempy, camBuffer))&&(ret_cam_bit(tempx, tempy,camBuffer)==0)){
					if(!canvas[tempx][tempy-30]){
						canvas[tempx][tempy-30] = true;
						m_edge.push_back(make_pair(tempx,tempy));
						traveling_right(tempx, tempy, Direction::DownRight, camBuffer);
					}
				}
			}
			else if(i==Direction::Left){
				uint8_t tempx = xcoord-1;
				uint8_t tempy = ycoord;
				if((ret_cam_bit(tempx, tempy, camBuffer)!=ret_cam_bit(tempx, tempy-1, camBuffer))&&(ret_cam_bit(tempx, tempy,camBuffer)==0)){
					if(!canvas[tempx][tempy-30]){
						canvas[tempx][tempy-30] = true;
						m_edge.push_back(make_pair(tempx,tempy));
						traveling_right(tempx, tempy, Direction::Right, camBuffer);
					}
				}
			}
			else if(i==Direction::DownLeft){
				uint8_t tempx = xcoord-1;
				uint8_t tempy = ycoord+1;
				if((ret_cam_bit(tempx, tempy, camBuffer)!=ret_cam_bit(tempx, tempy-1, camBuffer))&&(ret_cam_bit(tempx, tempy,camBuffer)==0)){
					if(!canvas[tempx][tempy-30]){
						canvas[tempx][tempy-30] = true;
						m_edge.push_back(make_pair(tempx,tempy));
						traveling_right(tempx, tempy, Direction::UpRight, camBuffer);
					}
				}
			}
			else if(i==Direction::Down){
				uint8_t tempx = xcoord;
				uint8_t tempy = ycoord+1;
				if((ret_cam_bit(tempx, tempy, camBuffer)!=ret_cam_bit(tempx-1, tempy, camBuffer))&&(ret_cam_bit(tempx, tempy,camBuffer)==0)){
					if(!canvas[tempx][tempy-30]){
						canvas[tempx][tempy-30] = true;
						m_edge.push_back(make_pair(tempx,tempy));
						traveling_right(tempx, tempy, Direction::Up, camBuffer);
					}
				}
			}
			else if(i==Direction::DownRight){
				uint8_t tempx = xcoord+1;
				uint8_t tempy = ycoord+1;
				if((ret_cam_bit(tempx, tempy, camBuffer)!=ret_cam_bit(tempx-1, tempy, camBuffer))&&(ret_cam_bit(tempx, tempy,camBuffer)==0)){
					if(!canvas[tempx][tempy-30]){
						canvas[tempx][tempy-30] = true;
						m_edge.push_back(make_pair(tempx,tempy));
						traveling_right(tempx, tempy, Direction::UpLeft, camBuffer);
					}
				}
			}
		}
		if(i==end_point){
			return;
		}
	}
	return;
}

//master
void Edge::traveling_left(uint8_t xcoord, uint8_t ycoord, uint8_t last_direction_from, const Byte* camBuffer){
		uint8_t wrong_way1 = 0;
		uint8_t wrong_way2 = 0;
		uint8_t wrong_way3 = 0;
		uint8_t start_point = 0;
		uint8_t end_point = 0;

		uint8_t junctions = check_junctions(ycoord);
		if(junctions>4){
			fail = true;
			return;
		}

		if((ycoord>bottomline-2)||(ycoord<=topline)||(xcoord>77)||(xcoord<2)){
			return;
		}


		if((last_direction_from==Direction::Up)){//Up = 0
			wrong_way1 = Direction::UpLeft;
			wrong_way2 = Direction::Up;
			wrong_way3 = Direction::UpRight;
		}
		else{
			wrong_way1 = last_direction_from-1;
			wrong_way2 = last_direction_from;
			wrong_way3 = last_direction_from+1;
			if(last_direction_from==Direction::UpLeft){
				wrong_way3 = 0;
			}
		}

		if(last_direction_from>=3){
			start_point = last_direction_from-3;
		}
		else{
			if(last_direction_from==2){
				start_point = Direction::UpLeft;
			}
			else if(last_direction_from==1){
				start_point = Direction::Left;
			}
			else if(last_direction_from==0){
				start_point = Direction::DownLeft;
			}
		}

		end_point = start_point +1;
		if(start_point == Direction::UpLeft){
			end_point = Direction::Up;
		}
		for(int i=start_point; ; i--){
			if(i<0){
				i+=8;
			}
			if((i==wrong_way1)||(i==wrong_way2)||(i==wrong_way3)){
				continue;
			}
			else{
				if(i==Direction::Right){
					uint8_t tempx = xcoord+1;
					uint8_t tempy = ycoord;
					if((ret_cam_bit(tempx, tempy, camBuffer)!=ret_cam_bit(tempx, tempy-1, camBuffer))&&(ret_cam_bit(tempx, tempy,camBuffer)==0)){
						if(!canvas[tempx][tempy-30]){
							canvas[tempx][tempy-30] = true;
							m_edge.push_back(make_pair(tempx,tempy));
							traveling_left(tempx, tempy, Direction::Left, camBuffer);
						}
					}
				}
				else if(i==Direction::UpRight){
					uint8_t tempx = xcoord+1;
					uint8_t tempy = ycoord-1;
					if((ret_cam_bit(tempx, tempy, camBuffer)!=ret_cam_bit(tempx-1, tempy, camBuffer))&&(ret_cam_bit(tempx, tempy,camBuffer)==0)){
						if(!canvas[tempx][tempy-30]){
							canvas[tempx][tempy-30] = true;
							m_edge.push_back(make_pair(tempx,tempy));
							traveling_left(tempx, tempy, Direction::DownLeft, camBuffer);
						}
					}
				}
				else if(i==Direction::Up){
					uint8_t tempx = xcoord;
					uint8_t tempy = ycoord-1;
					if((ret_cam_bit(tempx, tempy, camBuffer)!=ret_cam_bit(tempx-1, tempy, camBuffer))&&(ret_cam_bit(tempx, tempy,camBuffer)==0)){
						if(!canvas[tempx][tempy-30]){
							canvas[tempx][tempy-30] = true;
							m_edge.push_back(make_pair(tempx,tempy));
							traveling_left(tempx, tempy, Direction::Down,camBuffer);
						}
					}
				}
				else if(i==Direction::UpLeft){
					uint8_t tempx = xcoord-1;
					uint8_t tempy = ycoord-1;
					if((ret_cam_bit(tempx, tempy, camBuffer)!=ret_cam_bit(tempx, tempy+1, camBuffer))&&(ret_cam_bit(tempx, tempy,camBuffer)==0)){
						if(!canvas[tempx][tempy-30]){
							canvas[tempx][tempy-30] = true;
							m_edge.push_back(make_pair(tempx,tempy));
							traveling_left(tempx, tempy, Direction::DownRight, camBuffer);
						}
					}
				}
				else if(i==Direction::Left){
					uint8_t tempx = xcoord-1;
					uint8_t tempy = ycoord;
					if((ret_cam_bit(tempx, tempy, camBuffer)!=ret_cam_bit(tempx, tempy+1, camBuffer))&&(ret_cam_bit(tempx, tempy,camBuffer)==0)){
						if(!canvas[tempx][tempy-30]){
							canvas[tempx][tempy-30] = true;
							m_edge.push_back(make_pair(tempx,tempy));
							traveling_left(tempx, tempy, Direction::Right, camBuffer);
						}
					}
				}
				else if(i==Direction::DownLeft){
					uint8_t tempx = xcoord-1;
					uint8_t tempy = ycoord+1;
					if((ret_cam_bit(tempx, tempy, camBuffer)!=ret_cam_bit(tempx+1, tempy, camBuffer))&&(ret_cam_bit(tempx, tempy,camBuffer)==0)){
						if(!canvas[tempx][tempy-30]){
							canvas[tempx][tempy-30] = true;
							m_edge.push_back(make_pair(tempx,tempy));
							traveling_left(tempx, tempy, Direction::UpRight,camBuffer);
						}
					}
				}
				else if(i==Direction::Down){
					uint8_t tempx = xcoord;
					uint8_t tempy = ycoord+1;
					if((ret_cam_bit(tempx, tempy, camBuffer)!=ret_cam_bit(tempx+1, tempy, camBuffer))&&(ret_cam_bit(tempx, tempy,camBuffer)==0)){
						if(!canvas[tempx][tempy-30]){
							canvas[tempx][tempy-30] = true;
							m_edge.push_back(make_pair(tempx,tempy));
							traveling_left(tempx, tempy, Direction::Up,camBuffer);
						}
					}
				}
				else if(i==Direction::DownRight){
					uint8_t tempx = xcoord+1;
					uint8_t tempy = ycoord+1;
					if((ret_cam_bit(tempx, tempy, camBuffer)!=ret_cam_bit(tempx, tempy-1, camBuffer))&&(ret_cam_bit(tempx, tempy,camBuffer)==0)){
						if(!canvas[tempx][tempy-30]){
							canvas[tempx][tempy-30] = true;
							m_edge.push_back(make_pair(tempx,tempy));
							traveling_left(tempx, tempy, Direction::UpLeft, camBuffer);
						}
					}
				}
			}

			if(i==end_point){
				return;
			}
		}
	return;
}


vector<pair<int,int>> Edge::check_edge(const Byte* camBuffer){
	m_edge.clear();
	reset_canvas();
	reset_junction_arry();
	uint8_t top_line = topline;
	uint8_t bottom_line = bottomline;
	if(bottomline>=59)
		{bottom_line = 58;}
	if(!type){//master
		bool found = false;
		uint8_t start_point_i = 78;
		uint8_t start_point_j = bottom_line;
		if(ret_cam_bit(78, bottom_line,camBuffer)==0){//white
			for(int j = bottom_line; j>top_line; j--){
				for(int i=78; i>1; i--){
					if((ret_cam_bit(i, j,camBuffer)!=(ret_cam_bit(i-1, j,camBuffer)))&&(ret_cam_bit(i, j,camBuffer)==0)){
						start_point_i = i;
						start_point_j = j;
						m_edge.clear();
						m_edge.push_back(make_pair(start_point_i,start_point_j));
						found = true;
						break;
					}
				}
				if(found)
					break;
			}
			traveling_left(start_point_i, start_point_j, Direction::Down, camBuffer);

			int m_edge_size = m_edge.size();
			if(m_edge_size != 0 && m_edge[m_edge_size-1].first<3){
				found = false;
				int last = m_edge[m_edge.size()-1].second;
				for(int j =last; j>top_line; j--){
					for(int i=78; i>2; i--){
						if((ret_cam_bit(i, j,camBuffer)!=(ret_cam_bit(i-1, j,camBuffer)))&&(ret_cam_bit(i, j,camBuffer)==0)){
							start_point_i = i;
							start_point_j = j;
							m_edge.push_back(make_pair(start_point_i,start_point_j));
							found = true;
							break;
						}
					}
					if(found)
						break;
				}
				traveling_left(start_point_i, start_point_j, Direction::Down, camBuffer);
			}
			if(m_edge.size()>0){
				if(distance(m_edge[0].first, m_edge[0].second, m_edge[m_edge.size()-1].first, m_edge[m_edge.size()-1].second)<5){
					m_edge.clear();
				}else if(abs(m_edge[0].second - m_edge[m_edge.size()-1].second)<3){
					m_edge.clear();
				}
			}
			if(fail){
				fail = false;
				m_edge.clear();
			}

		}
		else{
			return m_edge;
		}
	}
	else{
		bool found = false;
		uint8_t start_point_i = 1;
		uint8_t start_point_j = bottom_line;
		if(ret_cam_bit(1, bottom_line,camBuffer)==0){//white
			for(int j = bottom_line; j>top_line; j--){
				for(int i=1; i<78; i++){
					if((ret_cam_bit(i, j,camBuffer)!=(ret_cam_bit(i+1, j,camBuffer)))&&(ret_cam_bit(i, j,camBuffer)==0)){
						start_point_i = i;
						start_point_j = j;
						m_edge.clear();
						m_edge.push_back(make_pair(start_point_i,start_point_j));
						found = true;
						break;
					}
				}
				if(found)
					break;
			}
			traveling_right(start_point_i, start_point_j, Direction::Down, camBuffer);
			if(m_edge[m_edge.size()-1].first>76){
				found = false;
				int last = m_edge[m_edge.size()-1].second;
				for(int j =last; j>top_line; j--){
					int here;
					for(int i=1; i<78; i++){
						if((ret_cam_bit(i, j,camBuffer)!=(ret_cam_bit(i+1, j,camBuffer)))&&(ret_cam_bit(i, j,camBuffer)==0)){
							start_point_i = i;
							start_point_j = j;
							m_edge.push_back(make_pair(start_point_i,start_point_j));
							found = true;
							break;
						}
					}
					if(found)
						break;
				}
				traveling_right(start_point_i, start_point_j, Direction::Down, camBuffer);
			}


			if(m_edge.size()>0){
				if(distance(m_edge[0].first, m_edge[0].second, m_edge[m_edge.size()-1].first, m_edge[m_edge.size()-1].second)<5){
					m_edge.clear();
				}else if(abs(m_edge[0].second - m_edge[m_edge.size()-1].second)<3){
					m_edge.clear();
				}
			}
			if(fail){
				fail = false;
				m_edge.clear();
			}

		}
		else{
			return m_edge;
		}
	}

	return m_edge;
}

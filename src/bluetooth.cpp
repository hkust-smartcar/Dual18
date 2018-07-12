/*
 * bluetooth.cpp
 *
 *  Created on: Mar 9, 2018
 *      Author: morristseng
 */
#include "bluetooth.h"
#include "corner.h"
#include <vector>
#define width 80
#define height 60

void M_Bluetooth::change_message(const Byte* m_buff) {
	if (((*m_buff) == Informations::edge) && (buffer.size() == 0)) {
		buffer.clear();
		information_types = Informations::edge;
	}
	if ((*m_buff == Informations::slope) && (buffer.size() == 0)) {
		buffer.clear();
		information_types = Informations::slope;
	}
	if ((*m_buff == Informations::edge_size) && (buffer.size() == 0)) {
		buffer.clear();
		information_types = Informations::edge_size;
	}

	if ((*m_buff == Informations::edge_xmid) && (buffer.size() == 0)) {
		buffer.clear();
		information_types = Informations::edge_xmid;
	}

	if (((*m_buff) == Informations::mpu) && (buffer.size() == 0)) {
		buffer.clear();
		information_types = Informations::mpu;
	}

	if ((*m_buff == Informations::corner) && (buffer.size() == 0)) {
		buffer.clear();
		information_types = Informations::corner;
	}

}

void M_Bluetooth::build_message(const Byte* m_buff) {
	if (information_types == Informations::edge) {
		this->buffer.push_back(*m_buff);
		if (((*m_buff) == Informations::end)) {
			m_edge.clear();
			int size = buffer[1];
			if (size != 0) {
				if (buffer.size() == size) {
					set_edge(size);
					buffer.clear();
				} else {
					buffer.clear();
				}
			} else {
				buffer.clear();
			}
		}
	}

	else if ((information_types == Informations::slope)) {
		this->buffer.push_back(*m_buff);
		if (((*m_buff) == Informations::end)) {
			m_slope = 0;
			int size = buffer[1];
			if (buffer.size() == size) {
				if (buffer[3] == true) {
					m_slope = (-1) * buffer[2] / 10.0;
				} else {
					m_slope = (-1) * buffer[2] / 100.0;
				}
			} else {
				m_slope = 0;
			}
			buffer.clear();
		}
	} else if ((information_types == Informations::edge_xmid)) {
		this->buffer.push_back(*m_buff);
		if (((*m_buff) == Informations::end)) {
			edge_mid = 0;
			int size = buffer[1];
			if (buffer.size() == size) {
				edge_mid = buffer[2];
			} else {
				edge_mid = 0;
			}
			buffer.clear();
		}
	}

	else if ((information_types == Informations::edge_size)) {
		this->buffer.push_back(*m_buff);
		if (((*m_buff) == Informations::end)) {
			edge_size = 0;
			int size = buffer[1];
			if (buffer.size() == size) {
				edge_size = buffer[2];
			} else {
				edge_size = 0;
			}
			buffer.clear();
		}
	}

	else if ((information_types == Informations::mpu)) {
		this->buffer.push_back(*m_buff);
		if (((*m_buff) == Informations::end)) {
			mpu_data = 0;
			int size = buffer[1];
			if (buffer.size() == size) {
				mpu_data = ((buffer[5] << 24) & 0xFF000000) +
					     ((buffer[4] << 16) & 0x00FF0000) +
						((buffer[3] << 8) & 0x0000FF00) +
						       (buffer[2] & 0x000000FF);
			} else {
				mpu_data = 0;
			}
			buffer.clear();
		}
	}

	else if (information_types == Informations::corner) {
		this->buffer.push_back(*m_buff);
		if (((*m_buff) == Informations::end)) {
			slave_corner.clear();
			uint8_t size = buffer[1];
			if (size != 3) {
				if (buffer.size() == size) {
					slave_corner.clear();
					set_corner(size);
				} else {
					slave_corner.clear();
				}
			} else {
				slave_corner.clear();
			}
			buffer.clear();
		}
	}

	else {
		buffer.clear();
	}
}

void M_Bluetooth::set_edge(int size) {
	m_edge.clear();
	for (int i = 2; i < size - 1; i += 2) {
		m_edge.push_back(std::make_pair(buffer[i], buffer[i + 1]));
	}
	buffer.clear();
}

void M_Bluetooth::set_corner(int size) {
	slave_corner.clear();
	for (int i = 2; i < size - 1; i += 3) {
		int x_coord = buffer[i];
		int y_coord = buffer[i + 1];
		float percentage = buffer[i + 2] / 100.0;
		if ((x_coord < width) && (x_coord >= 0) && (y_coord >= 0) && (y_coord < height)) {
			Corner temp(x_coord, y_coord, percentage);
			slave_corner.push_back(temp);
		}
	}
	buffer.clear();
}

void S_Bluetooth::send_edge(std::vector<std::pair<int, int>> slave_edge) {
	if (slave_edge.size() == 0) {
		Byte *buffer = new Byte[3];
		buffer[0] = Informations::edge;
		buffer[1] = 0; //size
		buffer[2] = Informations::end;
		m_bt.SendBuffer(buffer, 3);
		delete[] buffer;
	}
	if (slave_edge.size() > 0) {
		int how_many_lines = slave_edge.size();
		int size = 2 * how_many_lines + 3;
		Byte *buffer = new Byte[size];
		buffer[0] = Informations::edge;
		buffer[1] = size;
		buffer[size - 1] = Informations::end;

		for (int i = 0; i < how_many_lines; i++) {
			buffer[2 + 2 * i] = slave_edge[i].first;
			buffer[3 + 2 * i] = slave_edge[i].second;
		}

		m_bt.SendBuffer(buffer, size);
		delete[] buffer;
	}
}

void S_Bluetooth::send_slope(float m_slope) {
	int size = 5;
	Byte* buffer = new Byte[size];
	buffer[0] = Informations::slope;
	buffer[1] = size;
	if (m_slope < 0) {
		m_slope *= -1;
	}

	if (m_slope < 1.9) {
		buffer[2] = m_slope * 100;
		buffer[3] = false;
	}

	else if (m_slope > 1.9) {
		buffer[2] = m_slope * 10;
		buffer[3] = true;
	}

	buffer[size - 1] = Informations::end;

	m_bt.SendBuffer(buffer, size);
	delete[] buffer;
}

void S_Bluetooth::send_int_data(int int_data, int type) {
	int size = 7;
	Byte* buffer = new Byte[size];
	buffer[0] = type;
	buffer[1] = size;
	// little-endian encoding
	buffer[5] = int_data >> 24;
	buffer[4] = int_data >> 16;
	buffer[3] = int_data >> 8;
	buffer[2] = int_data;
	buffer[size - 1] = Informations::end;

	m_bt.SendBuffer(buffer, size);
	delete[] buffer;
}

void S_Bluetooth::send_corner(vector<Corner> slave_corner) {
	if (slave_corner.size() == 0) {
		Byte* buffer = new Byte[3];
		buffer[0] = Informations::corner;
		buffer[1] = 0;
		buffer[2] = Informations::end;
		m_bt.SendBuffer(buffer, 3);
		delete[] buffer;
	}

	if (slave_corner.size() > 0) {
		int size = 3 * slave_corner.size() + 3;
		int num_of_corner = slave_corner.size();
		Byte* buffer = new Byte[size];
		buffer[0] = Informations::corner;
		buffer[1] = size;
		buffer[size - 1] = Informations::end;
		for (int i = 0; i < num_of_corner; i++) {
			buffer[2 + 3 * i] = slave_corner[i].get_xcoord();
			buffer[3 + 3 * i] = slave_corner[i].get_ycoord();
			buffer[4 + 3 * i] = (uint8_t)(slave_corner[i].get_percentage()) * 100;
		}

		m_bt.SendBuffer(buffer, size);
		delete[] buffer;
	}
}

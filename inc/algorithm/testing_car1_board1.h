/*
 * testing_car1_board1.h
 *
 *  Created on: 24 May 2018
 *      Author: Job
 */

#ifndef INC_ALGORITHM_TESTING_CAR1_BOARD1_H_
#define INC_ALGORITHM_TESTING_CAR1_BOARD1_H_


#include <cstdint>
#include <list>
#include <vector>
#include <utility>

#include "libsc/system.h"

namespace algorithm {
namespace testing {
namespace car1 {
namespace board1{


#define MAG_MEAN_FILTER_NUM_OF_SMAPLES 8

float inv_sqrt(float);
float _sqrt(float);
int max(int, int);
int min(int, int);
bool IsOneLine(uint8_t maxVal, uint8_t leftVal, uint8_t rightVal, float multi);
bool IsTwoLine(uint8_t maxVal, uint8_t leftVal, uint8_t rightVal, float multi);




int main_car1_board1(bool debug_ = false);

}  // namespace board1
}  // namespace car1
}  // namespace optimal
}  // namespace algorithm





#endif /* INC_ALGORITHM_TESTING_CAR1_BOARD1_H_ */

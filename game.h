#ifndef __GAME_H__
#define __GAME_H__


bool instruction(int itype, int deadline);

bool operate_switch_polling(int deadline, int button);

bool operate_light_sensor_polling(int deadline);

bool instruction(int itype, int deadline);

int gen_instruction();

int deadline_calc(int score);

bool accelerometer_test(int deadline);

#endif
